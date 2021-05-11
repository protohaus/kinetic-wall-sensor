#pragma once

#include <k4a/k4a.h>
#include <k4abt.h>

#include <cmath>
#include <nlohmann/json.hpp>

#include "ws_client.h"

class BodyHandler {
 public:
  BodyHandler(websocket_endpoint& client, const uint box_count)
      : client_(client), box_count_(box_count) {
    box_activation_.resize(box_count);
  }

  void interpret(k4abt_body_t& body) {
    k4a_float3_t position_average = get_average_position(body);
    const float x = position_average.xyz.x;
    for (int i = 0; i < box_activation_.size(); i++) {
      const float abs_x_distance = fabs(x - box_x_[i]);
      const float x_activation_limited = fmax(1000 - abs_x_distance, 0);
      box_activation_[i] = x_activation_limited / 1000;
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
    printf("%f %f %f %f\n", box_activation_[0], box_activation_[1],
           box_activation_[2], box_activation_[3]);
  }
  void send() {
    nlohmann::json commands = nlohmann::json::array();
    for (int i = 0; i < box_activation_.size(); i++) {
      nlohmann::json command = nlohmann::json::object();
      command["id"] = box_count_ - i - 1;
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
    for (const k4abt_joint_t& joint : body.skeleton.joints) {
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

  std::vector<float> box_x_ = {-1500, -500, 500, 1500};
  std::vector<float> box_activation_;
};