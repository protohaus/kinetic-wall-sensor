#pragma once

#include <k4a/k4a.h>
#include <k4abt.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <nlohmann/json.hpp>
#include <vector>

#include "ws_client.h"

class BodyHandler {
 private:
  enum class Mode { kWalking, kHands } mode_;

 public:
  BodyHandler(websocket_endpoint& client, const uint box_count)
      : client_(client), box_count_(box_count), mode_(Mode::kWalking) {
    box_activation_.resize(box_count);
  }

  void interpret(k4abt_body_t& body) {
    printf("l_x: %.00f r_x: %.00f\n",
           body.skeleton.joints[K4ABT_JOINT_HAND_LEFT].position.xyz.x,
           body.skeleton.joints[K4ABT_JOINT_HAND_RIGHT].position.xyz.x);

    k4a_float3_t position_average = get_average_position(body);

    // Calculate movement
    last_x_positions_.push_front(position_average.xyz.x);
    if (last_x_positions_.size() > 5) {
      last_x_positions_.pop_back();
    }
    float x_min = last_x_positions_.front();
    float x_max = last_x_positions_.front();
    for (const float i : last_x_positions_) {
      if (x_min > i) {
        x_min = i;
      }
      if (x_max < i) {
        x_max = i;
      }
    }
    if (x_max - x_min < 200) {
      mode_ = Mode::kHands;
    } else {
      mode_ = Mode::kWalking;
    }

    if (mode_ == Mode::kWalking) {
      const float x = position_average.xyz.x;
      for (int i = 0; i < box_activation_.size(); i++) {
        const float abs_x_distance = fabs(x - box_x_[i]);
        const float x_activation_limited = fmax(500 - abs_x_distance, 0);
        box_activation_[i] = x_activation_limited / 500;
      }
    }
    if (mode_ == Mode::kHands) {
      const auto left_joints = std::vector<k4abt_joint_id_t>{
          K4ABT_JOINT_HAND_LEFT, K4ABT_JOINT_HANDTIP_LEFT,
          K4ABT_JOINT_THUMB_LEFT, K4ABT_JOINT_WRIST_LEFT};
      k4abt_joint_t left_side = body.skeleton.joints[left_joints[0]];
      // for (const auto& joint : body.skeleton.joints) {
      //   if (left_side.confidence_level < joint.confidence_level) {
      //     left_side = joint;
      //   }
      // }

      const auto right_joints = std::vector<k4abt_joint_id_t>{
          K4ABT_JOINT_HAND_RIGHT, K4ABT_JOINT_HANDTIP_RIGHT,
          K4ABT_JOINT_THUMB_RIGHT, K4ABT_JOINT_WRIST_RIGHT};
      k4abt_joint_t right_side = body.skeleton.joints[right_joints[0]];
      // for (const auto& joint : body.skeleton.joints) {
      //   if (right_side.confidence_level < joint.confidence_level) {
      //     right_side = joint;
      //   }
      // }

      // printf("l: %d r: %d\n", left_side.confidence_level,
      //        right_side.confidence_level);
      // if (left_side.confidence_level >= K4ABT_JOINT_CONFIDENCE_LOW &&
      //     right_side.confidence_level >= K4ABT_JOINT_CONFIDENCE_LOW) {
      const float abs_x_distance =
          fabs(left_side.position.xyz.x - right_side.position.xyz.x);
      float activation = fmin(1000, abs_x_distance) / 1000;
      printf("dist: %.00f act: %.00f\n", abs_x_distance, activation);
      for (float& i : box_activation_) {
        if (activation < 0.2) {
          i = 0;
        } else {
          i = activation;
        }
      }
      // }
    }
    // if (position_average.xyz.x < -1000) {
    //   box_activation_[0] = 1;
    // } else if (position_average.xyz.x < 0) {
    //   box_activation_[1] = 1;
    // } else if (position_average.xyz.x < 1000) {
    //   box_activation_[2] = 1;
    // } else if (position_average.xyz.x < 2000) {
    //   box_activation_[3] = 1;
    // }
    printf("%f %f %f %f %f %f\n", box_activation_[0], box_activation_[1],
           box_activation_[2], box_activation_[3], box_activation_[4], box_activation_[5]);
  }

  void send() {
    nlohmann::json commands = nlohmann::json::array();
    for (int i = 0; i < box_activation_.size(); i++) {
      nlohmann::json command = nlohmann::json::object();
      command["id"] = i;
      if (box_activation_[i] > 0) {
        command["mode"] = "pwm";
        command["frequency_hz"] = 1 + box_activation_[i] * 10;
        command["duty_cycle"] = box_activation_[i] * 0.5;
      } else {
        command["mode"] = "off";
      }
      commands.push_back(command);
    }
    client_.send(0, commands.dump());
    // Reset all box activation values
    for (float& i : box_activation_) {
      i = 0;
    }
  }

 private:
  k4a_float3_t get_average_position(k4abt_body_t& body) {
    k4a_float3_t position_average = {0, 0, 0};
    uint joint_count = 0;
    for (const k4abt_joint_t joint : body.skeleton.joints) {
      if (joint.confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM) {
        position_average.v[0] += joint.position.v[0];
        position_average.v[1] += joint.position.v[1];
        position_average.v[2] += joint.position.v[2];
        joint_count++;
      }
    }
    position_average.v[0] /= joint_count;
    position_average.v[1] /= joint_count;
    position_average.v[2] /= joint_count;
    return position_average;
  }

  websocket_endpoint& client_;
  const uint box_count_;

  std::vector<float> box_x_ = {900, -900, 0, -900, 0, 900};
  // std::vector<float> box_x_ = {-300, 300, 900, 1500, -900, -1500};
  // std::vector<float> box_x_ = {-1500, -900, -300, 300, 900, 1500};
  std::vector<float> box_activation_;
  std::deque<float> last_x_positions_;
};