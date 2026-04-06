#include <chrono>
#include <memory>
#include <iostream>
#include <functional>
#include <mutex>
#include <nlohmann/json.hpp>
#include <vector>
#include <sstream>
#include <string>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

#include "xr_msgs/Custom.h"
#include "xr_msgs/Head.h"
#include "xr_msgs/Controller.h"

#include "PXREARobotSDK.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

using json = nlohmann::json;

static const std::vector<std::string> JOINT_NAMES = {
    "pico_pelvis", "pico_left_hip", "pico_right_hip", "pico_spine1",
    "pico_left_knee", "pico_right_knee", "pico_spine2", "pico_left_ankle",
    "pico_right_ankle", "pico_spine3", "pico_left_foot", "pico_right_foot",
    "pico_neck", "pico_left_collar", "pico_right_collar", "pico_head",
    "pico_left_shoulder", "pico_right_shoulder", "pico_left_elbow", "pico_right_elbow",
    "pico_left_wrist", "pico_right_wrist", "pico_left_hand", "pico_right_hand"
};

static const std::vector<std::string> SELECTED_JOINT_NAMES = {
    "pico_pelvis",      
    "pico_spine3",      

    "pico_left_foot",  
    "pico_right_foot",  
    "pico_head",

    "pico_left_shoulder",  
    "pico_right_shoulder",  

    "pico_left_elbow",  
    "pico_right_elbow",  

    "pico_left_hand",     
    "pico_right_hand",
};

std::function<void(void* context, PXREAClientCallbackType type, int status, void* userData)> g_callback;
std::mutex g_callback_mutex;
ros::Publisher g_publisher;
ros::Publisher g_posearray_publisher;

void printAvailableJointNames()
{
    std::cout << "Available XR Body Joints:" << std::endl;
    for (size_t i = 0; i < JOINT_NAMES.size(); ++i) {
        std::cout << i << " : " << JOINT_NAMES[i] << std::endl;
    }
}

void callbackForwarder(void* context, PXREAClientCallbackType type, int status, void* userData) {
  std::lock_guard<std::mutex> lock(g_callback_mutex);
  if (g_callback) {
    g_callback(context, type, status, userData);
  }
}

void print_json(const json& j, int indent=1) {
    std::string indent_str(indent * 2, ' ');
    
    if (j.is_object()) {
        std::cout << indent_str << "{\n";
        for (const auto& [key, value] : j.items()) {
            std::cout << indent_str << "  \"" << key << "\": ";
            print_json(value, indent + 1);
            std::cout << ",\n";
        }
        if (!j.empty()) {
            std::cout << "\b\b" << std::endl;
        }
        std::cout << indent_str << "}";
    } else if (j.is_array()) {
        std::cout << indent_str << "[\n";
        for (const auto& item : j) {
            std::cout << indent_str << "  ";
            print_json(item, indent + 1);
            std::cout << ",\n";
        }
        if (!j.empty()) {
            std::cout << "\b\b" << std::endl;
        }
        std::cout << indent_str << "]";
    } else if (j.is_string()) {
        std::cout << "\"" << j.get<std::string>() << "\"";
    } else if (j.is_boolean()) {
        std::cout << (j.get<bool>() ? "true" : "false");
    } else if (j.is_number_integer()) {
        std::cout << j.get<int64_t>();
    } else if (j.is_number_unsigned()) {
        std::cout << j.get<uint64_t>();
    } else if (j.is_number_float()) {
        std::cout << j.get<double>();
    } else if (j.is_null()) {
        std::cout << "null";
    } else {
        std::cout << "unknown type";
    }
}

std::vector<float> stringToFloatVector(const std::string& input) {
    std::vector<float> result;
    std::stringstream ss(input);
    std::string token;
    while (std::getline(ss, token, ',')) {
        try {
            result.push_back(std::stof(token));
        } catch (const std::exception& e) {
            std::cerr << "转换错误: " << token << " -> " << e.what() << std::endl;
        }
    }
    return result;
}

void publishSelectedPoseArrayByName(const json& value_obj)
{
    if (!value_obj.contains("Body")) return;
    if (!value_obj["Body"].contains("joints")) return;

    auto joints = value_obj["Body"]["joints"];

    geometry_msgs::PoseArray pose_array;
    pose_array.header.stamp = ros::Time::now();
    pose_array.header.frame_id = "map";

    for (size_t i = 0; i < JOINT_NAMES.size() && i < joints.size(); ++i) {
        const std::string& joint_name = JOINT_NAMES[i];

        if (std::find(SELECTED_JOINT_NAMES.begin(),
                      SELECTED_JOINT_NAMES.end(),
                      joint_name) == SELECTED_JOINT_NAMES.end()) {
            continue;
        }

        std::vector<float> pose_vec = stringToFloatVector(joints[i]["p"].get<std::string>());
        if (pose_vec.size() < 7) continue;

        geometry_msgs::Pose p;

        p.position.x = -pose_vec[2];
        p.position.y = -pose_vec[0];
        p.position.z =  pose_vec[1];

        float qx = -pose_vec[5];
        float qy = -pose_vec[3];
        float qz =  pose_vec[4];
        float qw =  pose_vec[6];

        float norm = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
        if (norm < 1e-6f) {
            p.orientation.x = 0.0;
            p.orientation.y = 0.0;
            p.orientation.z = 0.0;
            p.orientation.w = 1.0;
        } else {
            p.orientation.x = qx / norm;
            p.orientation.y = qy / norm;
            p.orientation.z = qz / norm;
            p.orientation.w = qw / norm;
        }

        pose_array.poses.push_back(p);
    }

    if (!pose_array.poses.empty()) {
        g_posearray_publisher.publish(pose_array);
    }
}

void OnPXREAClientCallback(void* context, PXREAClientCallbackType type, int status, void* userData)
{
  switch (type)
  {
    case PXREAServerConnect:
      std::cout << "server connect" << std::endl;
      break;
    case PXREAServerDisconnect:
      std::cout << "server disconnect" << std::endl;
      break;
    case PXREADeviceFind:
      std::cout << "device find" << (const char*)userData << std::endl;
      break;
    case PXREADeviceMissing:
      std::cout << "device missing" << (const char*)userData << std::endl;
      break;
    case PXREADeviceConnect:
      std::cout << "device connect" << (const char*)userData << status << std::endl;
      break;
    case PXREADeviceStateJson:
      {
        auto& dsj = *((PXREADevStateJson*)userData);
        try {
          auto json_obj = json::parse(dsj.stateJson);
          auto value_str = json_obj["value"].get<std::string>();
          auto value_obj = json::parse(value_str);

          xr_msgs::Custom custom_msg;
          custom_msg.timestamp_ns = value_obj["timeStampNs"].get<uint64_t>();
          custom_msg.input = value_obj["Input"].get<int>();

          xr_msgs::Head head_msg;
          if (value_obj.contains("Head")) {
            auto head_j = value_obj["Head"];
            std::vector<float> head_pose = stringToFloatVector(head_j["pose"].get<std::string>());
            if (head_pose.size() != 7) {
              std::cerr << "Parse failed: head pose data length != 7" << std::endl;
            }
            for (size_t i = 0; i < std::min(head_pose.size(), size_t(7)); ++i) {
              head_msg.pose[i] = head_pose[i];
            }
            head_msg.status = head_j["status"].get<int>();
          } else {
            head_msg.status = -1;
          }
          custom_msg.head = head_msg;

          if (value_obj.contains("Controller")) {
            for (auto& element : value_obj["Controller"].items()) {
              xr_msgs::Controller controller_msg;
              auto ctrl_j = element.value();

              controller_msg.axis_x = ctrl_j["axisX"].get<float>();
              controller_msg.axis_y = ctrl_j["axisY"].get<float>();
              controller_msg.axis_click = ctrl_j["axisClick"].get<bool>();
              controller_msg.gripper = ctrl_j["grip"].get<float>();
              controller_msg.trigger = ctrl_j["trigger"].get<float>();
              controller_msg.primary_button = ctrl_j["primaryButton"].get<bool>();
              controller_msg.secondary_button = ctrl_j["secondaryButton"].get<bool>();
              controller_msg.menu_button = ctrl_j["menuButton"].get<bool>();
              std::vector<float> ctrl_pose = stringToFloatVector(ctrl_j["pose"].get<std::string>());
              if (ctrl_pose.size() != 7) {
                std::cerr << "Parse failed: ctrl pose data length != 7" << std::endl;
              }
              for (size_t i = 0; i < std::min(ctrl_pose.size(), size_t(7)); ++i) {
                controller_msg.pose[i] = ctrl_pose[i];
              }
              controller_msg.status = 3;

              if (element.key() == "left") {
                custom_msg.left_controller = controller_msg;
              } else {
                custom_msg.right_controller = controller_msg;
              }
            }
          } else {
            xr_msgs::Controller left_controller_msg;
            xr_msgs::Controller right_controller_msg;
            left_controller_msg.status = -1;
            right_controller_msg.status = -1;
            custom_msg.left_controller = left_controller_msg;
            custom_msg.right_controller = right_controller_msg;
          }

          g_publisher.publish(custom_msg);
          publishSelectedPoseArrayByName(value_obj);
          ROS_INFO("Publishing timestamp_ns '%ld'", (long int)custom_msg.timestamp_ns);
        } catch (const std::exception& e) {
          std::cerr << "Parse failed: " << e.what() << std::endl;
        }
      }
      break;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "xr_publisher");
  ros::NodeHandle nh;
  g_publisher = nh.advertise<xr_msgs::Custom>("xr_pose", 10);
  g_posearray_publisher = nh.advertise<geometry_msgs::PoseArray>("tracker_pose", 10);

  printAvailableJointNames();

  g_callback = [](void* context, PXREAClientCallbackType type, int status, void* userData) {
    OnPXREAClientCallback(context, type, status, userData);
  };
  PXREAInit(NULL, callbackForwarder, PXREAFullMask);

  ros::spin();

  PXREADeinit();
  return 0;
}