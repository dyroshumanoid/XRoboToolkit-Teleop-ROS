#include <chrono>
#include <memory>
#include <iostream>
#include <functional>
#include <mutex>
#include <vector>
#include <string>
#include <array>
#include <cstdlib>
#include <cmath>

// JSON (nlohmann/json)
#include <nlohmann/json.hpp>

// ROS 1 Headers
#include <ros/ros.h>
#include <xr_msgs/Custom.h>
#include <xr_msgs/Head.h>
#include <xr_msgs/Controller.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>

// SDK Header
#include "PXREARobotSDK.h"

using json = nlohmann::json;

// [안전장치] SDK callback 스레드와 ROS 메인 스레드 간 충돌 방지
std::mutex g_data_mutex;
std::function<void(void* context, PXREAClientCallbackType type, int status, void* userData)> g_callback;

void callbackForwarder(void* context, PXREAClientCallbackType type, int status, void* userData) {
    if (g_callback) {
        g_callback(context, type, status, userData);
    }
}

// C-Style 파싱 (Latency 최적화)
inline bool parse_pose_safe(const std::string& input, boost::array<float, 7>& out_pose) {
    if (input.empty()) return false;
    const char* ptr = input.c_str();
    char* end_ptr = nullptr;
    int index = 0;
    while (index < 7 && *ptr != '\0') {
        out_pose[index] = std::strtof(ptr, &end_ptr);
        if (ptr == end_ptr) { ptr++; continue; }
        ptr = end_ptr;
        if (*ptr == ',') ptr++;
        index++;
    }
    return (index >= 6);
}

static const std::vector<std::string> JOINT_NAMES = {
    "pico_pelvis", "pico_left_hip", "pico_right_hip", "pico_spine1",
    "pico_left_knee", "pico_right_knee", "pico_spine2", "pico_left_ankle",
    "pico_right_ankle", "pico_spine3", "pico_left_foot", "pico_right_foot",
    "pico_neck", "pico_left_collar", "pico_right_collar", "pico_head_joint",
    "pico_left_shoulder", "pico_right_shoulder", "pico_left_elbow", "pico_right_elbow",
    "pico_left_wrist", "pico_right_wrist", "pico_left_hand", "pico_right_hand"
};

class XRNode {
public:
    XRNode(ros::NodeHandle& nh) : nh_(nh) {
        // ROS 1 Publisher 설정
        publisher_ = nh_.advertise<xr_msgs::Custom>("xr_pose", 1);
        
        for (auto& joint_data : body_joints_data_) {
            joint_data.fill(0.0f);
            joint_data[6] = 1.0f; // [Fix 1] 쿼터니언 W 기본값을 1.0으로 초기화 (TF 에러 방지)
        }
        tf_msg_buffer_.reserve(30);
        reset_accumulation();

        ROS_INFO("========================================");
        ROS_INFO(" XRNode V6: TF Normalized & Latency Test");
        ROS_INFO("========================================");
    }

    void reset_accumulation() {
        has_head_ = false;
        has_left_controller_ = false;
        has_right_controller_ = false;
        has_body_ = false;
    }

    inline void fill_transform(
        geometry_msgs::TransformStamped& t,
        const std::string& child_frame_id,
        const boost::array<float, 7>& pose,
        const ros::Time& stamp) 
    {
        t.header.stamp = stamp;
        t.header.frame_id = "map"; 
        t.child_frame_id = child_frame_id;

        // PICO (Unity/Right-handed) -> ROS (Right-handed, Z-up) 좌표 변환 적용
        t.transform.translation.x = -pose[2]; 
        t.transform.translation.y = -pose[0]; 
        t.transform.translation.z =  pose[1]; 

        // [Fix 2] 쿼터니언 추출 및 정규화(Normalization)
        float qx = -pose[5];
        float qy = -pose[3];
        float qz =  pose[4];
        float qw =  pose[6];

        float norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
        if (norm < 0.0001f) {
            t.transform.rotation.x = 0.0;
            t.transform.rotation.y = 0.0;
            t.transform.rotation.z = 0.0;
            t.transform.rotation.w = 1.0;
        } else {
            t.transform.rotation.x = qx / norm;
            t.transform.rotation.y = qy / norm;
            t.transform.rotation.z = qz / norm;
            t.transform.rotation.w = qw / norm;
        }
    }

    void broadcast_all_tf() {
        tf_msg_buffer_.clear();
        ros::Time stamp = ros::Time::now();
        geometry_msgs::TransformStamped t_temp;

        if (has_head_) {
            fill_transform(t_temp, "pico_head", accumulated_msg_.head.pose, stamp);
            tf_msg_buffer_.push_back(t_temp);
        }
        if (has_left_controller_) {
            fill_transform(t_temp, "pico_left_controller", accumulated_msg_.left_controller.pose, stamp);
            tf_msg_buffer_.push_back(t_temp);
        }
        if (has_right_controller_) {
            fill_transform(t_temp, "pico_right_controller", accumulated_msg_.right_controller.pose, stamp);
            tf_msg_buffer_.push_back(t_temp);
        }
        if (has_body_) {
            for (size_t i = 0; i < JOINT_NAMES.size(); ++i) {
                fill_transform(t_temp, JOINT_NAMES[i], body_joints_data_[i], stamp);
                tf_msg_buffer_.push_back(t_temp);
            }
        }

        if (!tf_msg_buffer_.empty()) {
            tf_broadcaster_.sendTransform(tf_msg_buffer_);
        }
    }

    void OnPXREAClientCallback(void* context, PXREAClientCallbackType type, int status, void* userData) {
        std::lock_guard<std::mutex> lock(g_data_mutex);
        if (type != PXREADeviceStateJson) return;
        
        // [측정 시작] 1. 콜백이 호출된 직후의 고정밀 시간 기록 (오타 수정됨)
        auto t_recv = std::chrono::high_resolution_clock::now();
        uint64_t current_ros_time_ns = ros::Time::now().toNSec();

        auto& dsj = *((PXREADevStateJson*)userData);
        try {
            auto json_root = json::parse(dsj.stateJson, nullptr, false);
            if (json_root.is_discarded() || !json_root.contains("value")) return;

            std::string value_str = json_root["value"].get<std::string>();
            auto value_obj = json::parse(value_str, nullptr, false);
            if (value_obj.is_discarded()) return;

            // PICO가 데이터를 보낸 시간 (타임스탬프 추출)
            accumulated_msg_.timestamp_ns = value_obj.value("timeStampNs", current_ros_time_ns);
            accumulated_msg_.input = value_obj.value("Input", 0);

            // Head Parsing
            if (value_obj.contains("Head")) {
                auto& head_j = value_obj["Head"];
                if (parse_pose_safe(head_j.value("pose", ""), accumulated_msg_.head.pose)) {
                    accumulated_msg_.head.status = head_j.value("status", 0);
                    has_head_ = true;
                }
            }

            // Controller Parsing
            if (value_obj.contains("Controller")) {
                for (auto& element : value_obj["Controller"].items()) {
                    auto& ctrl_j = element.value();
                    xr_msgs::Controller* target_msg = (element.key() == "left") ? &accumulated_msg_.left_controller : &accumulated_msg_.right_controller;
                    
                    if (element.key() == "left") has_left_controller_ = true;
                    else has_right_controller_ = true;

                    target_msg->primary_button = ctrl_j.value("primaryButton", false) || 
                                                ctrl_j.value("buttonA", false) || 
                                                ctrl_j.value("A", false);
        
                    target_msg->secondary_button = ctrl_j.value("secondaryButton", false) || 
                                                ctrl_j.value("buttonB", false) || 
                                                ctrl_j.value("B", false);

                    target_msg->axis_x = ctrl_j.value("axisX", 0.0f);
                    target_msg->axis_y = ctrl_j.value("axisY", 0.0f);
                    target_msg->gripper = ctrl_j.value("grip", 0.0f);
                    target_msg->trigger = ctrl_j.value("trigger", 0.0f);
                    parse_pose_safe(ctrl_j.value("pose", ""), target_msg->pose);
                    target_msg->status = 3;
                }
            }

            // Body Parsing
            if (value_obj.contains("Body") && value_obj["Body"].contains("joints")) {
                auto joints_j = value_obj["Body"]["joints"];
                size_t limit = std::min(joints_j.size(), JOINT_NAMES.size());
                for (size_t i = 0; i < limit; ++i) {
                    parse_pose_safe(joints_j[i].value("p", ""), body_joints_data_[i]);
                }
                if (limit > 0) has_body_ = true;
            }

            if (has_head_) {
                publisher_.publish(accumulated_msg_);
                broadcast_all_tf();
                reset_accumulation();
                
                // [측정 종료] 2. TF Publish 직후의 고정밀 시간 기록
                auto t_pub = std::chrono::high_resolution_clock::now();

                // 1. 연산 지연 시간 (T_proc) 계산
                double t_proc_ms = std::chrono::duration<double, std::milli>(t_pub - t_recv).count();
                
                // 2. 네트워크 지연(Net) 영점 조절을 위한 정적 변수 선언
                static bool is_time_synced = false;
                static int64_t time_offset_ns = 0;

                // 언더플로우 방지를 위해 부호 있는 정수(int64_t)로 강제 형변환
                int64_t current_ros_ns = (int64_t)current_ros_time_ns;
                int64_t pico_ns = (int64_t)accumulated_msg_.timestamp_ns;

                // 3. 첫 패킷 수신 시, 두 기기의 시계 오차를 영점(0)으로 맞춤
                if (!is_time_synced) {
                    time_offset_ns = current_ros_ns - pico_ns;
                    is_time_synced = true;
                    ROS_INFO("Time synchronization complete. Offset: %ld ns", time_offset_ns);
                }

                // 4. 영점이 맞춰진 순수 네트워크 레이턴시 변동폭(Jitter) 계산
                int64_t current_diff_ns = (current_ros_ns - pico_ns) - time_offset_ns;
                double t_net_ms = current_diff_ns / 1000000.0;
                
                // 5. 총 레이턴시 합산
                double total_latency_ms = t_net_ms + t_proc_ms;

                // 6. 터미널 결과 출력
                ROS_INFO("[Latency] Net Jitter: %6.2f ms | Proc: %6.2f ms | Total: %6.2f ms", 
                         t_net_ms, t_proc_ms, total_latency_ms);
            }
        } catch (const std::exception& e) { 
            ROS_ERROR_STREAM("TF / JSON Exception: " << e.what());
        } catch (...) {
            ROS_ERROR("Unknown exception in PXREA callback");
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    xr_msgs::Custom accumulated_msg_;
    std::array<boost::array<float, 7>, 24> body_joints_data_;
    std::vector<geometry_msgs::TransformStamped> tf_msg_buffer_;
    bool has_head_, has_left_controller_, has_right_controller_, has_body_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "xr_publisher");
    ros::NodeHandle nh;
    auto xrNode = std::make_shared<XRNode>(nh);

    g_callback = [&xrNode](void* context, PXREAClientCallbackType type, int status, void* userData) {
        xrNode->OnPXREAClientCallback(context, type, status, userData);
    };

    PXREAInit(NULL, callbackForwarder, PXREAFullMask);
    ros::spin();
    PXREADeinit();
    return 0;
}