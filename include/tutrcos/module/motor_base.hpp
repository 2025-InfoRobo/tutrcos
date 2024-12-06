#pragma once

#include "main.h"

#include "tutrcos.hpp"
#include "tutrcos/module/encoder_base.hpp"
#include <cmath>

using namespace tutrcos;
using namespace tutrcos::module;

class MotorBase : public EncoderBase {
#define FLT_MAX std::numeric_limits<float>::max()
public:
  enum class Dir : int8_t {
    FORWARD = 1,
    REVERSE = -1,
  };

  enum class ControlType { PI_D_RAD, PI_D_RADPS, RAW, ZERO };

  MotorBase(Dir dir, float reduction_ratio, float offset, int64_t cpr,
            EncoderBase *enc = nullptr)
      : EncoderBase(cpr), dir_{utility::to_underlying(dir)},
        reduction_ratio_{reduction_ratio} {
    enc_ = enc;
  }

  virtual bool update() { return control(); };
  virtual void stop() { set_input(0); }
  virtual void set_input(float value) { input_ = value; };

  bool control() {
    // bool return_value = false;
    uint32_t dt = tutrcos::core::Kernel::get_ticks() - prev_ticks_;
    if (enc_ != nullptr) {
      if (!enc_->update()) {
        return false;
      }
      this->set_count(enc_->get_count());
    }

    switch (mode_) {
    case ControlType::PI_D_RAD: {
      x_ = get_rad();
      p_ = ref_ - x_;
      i_ += p_ * (dt / tickrate_);
      i_ = (abs(p_) < i_min_) ? 0 : std::clamp<float>(i_, -i_max_, i_max_);
      d_ = (pre_x_ - x_) / (dt / tickrate_);
      current_ = (p_ * kp_ + i_ * ki_ + d_ * kd_);
      pre_x_ = x_;
      set_input(current_ * dir_);
      break;
    }
    case ControlType::RAW: {
      i_ = 0;
      set_input(ref_ * dir_);
      break;
    }
    case ControlType::ZERO: {
      i_ = 0;
      set_input(0);
      break;
    }
    default: {
      i_ = 0;
    }
    }
    prev_ticks_ += dt;
    return true;
  }

  void set_ref(float ref) {
    uint32_t dt = tutrcos::core::Kernel::get_ticks() - prev_ticks_ref_;
    ref = pre_ref_ + std::clamp<float>(ref - pre_ref_, -ref_vel_limit_ * dt,
                                       ref_vel_limit_ * dt);
    ref_ = std::clamp<float>(ref, ref_limit_min_, ref_limit_max_);
    pre_ref_ = ref_;
    prev_ticks_ref_ += dt;
  }

  // min, max, /s
  void set_ref_limit_rad(float ref_min, float ref_max,
                         float ref_vel = FLT_MAX) {
    ref_limit_min_ = ref_min;
    ref_limit_max_ = ref_max;
    ref_vel_limit_ = abs(ref_vel);
  }

  void set_mode(ControlType mode) { mode_ = mode; }

  // Kp, Ki, Kd, maximum value of Isum, minimum value of p to calc Isum
  void set_pid_gain(float kp, float ki, float kd, float i_max = FLT_MAX,
                    float i_min = 0) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    i_max_ = i_max;
    i_min_ = i_min;
  }

  void set_offset_degree(float value) {
    offset_ = value / 360. / reduction_ratio_ * get_cpr();
  }

  EncoderBase *enc_ = nullptr;

private:
  const float tickrate_ = 1000;
  const int8_t dir_;
  const float reduction_ratio_;
  int offset_;

  ControlType mode_;
  float input_;

  uint32_t prev_ticks_;
  uint32_t prev_ticks_ref_;

  float prev_p_;
  float p_, i_, d_;
  float kp_, ki_, kd_, i_max_, i_min_;
  float x_, pre_x_;

  float ref_, current_;
  float ref_vel_limit_ = FLT_MAX;
  float ref_limit_min_ = -FLT_MAX;
  float ref_limit_max_ = FLT_MAX;
  float pre_ref_ = 0;
};