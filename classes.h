#ifndef __CLASSES_H
#define __CLASSES_H

#include "Arduino.h"
#include <inttypes.h>
#include <stdbool.h>

class N_ShapeInterpolator {
protected:
  float phase;
  float low_freq, high_freq;
  int number;
  float* frequency;
  float* frequency_mod;
  float* current_value;

public:
  N_ShapeInterpolator(int number, float low_freq, float high_freq);
  float add_to_frequency(uint8_t idx, float added_amt);
  void update();
  float get(uint8_t idx);
};

class N_SineInterpolator : public N_ShapeInterpolator {

public:
  N_SineInterpolator(int number, float low_freq, float high_freq);
  void update();
};

class N_VolumeInterpolator : public N_ShapeInterpolator {
  float symmetry;

public:
  N_VolumeInterpolator(int number, float low_freq, float high_freq);
  float output(uint8_t i);
  void update();
};


class HAL_SensorTracker {
  uint8_t pin;
  uint16_t low;
  uint16_t high;
  bool is_invert;
  float scale_amount;
  uint16_t raw_value;
  float scaled_value;
  float old_scaled_value;
  float delta_value;

public:
  HAL_SensorTracker(uint8_t pin, uint16_t low, uint16_t high, bool is_invert, float scale_amount);
  void set_value(uint16_t new_value);
  void calculate_delta();
  float get_scaled_value();
  float get_delta();
  uint8_t get_pin();
  uint16_t get_low();
  uint16_t get_high();
  float scale_HAL_values(uint16_t reading, float range);
};

class LPF {
  float lowpassed;
  float lowpass_amount;
  float low_thresh, high_thresh;
  float gated_value;
  bool b_gate_changed;

public:
  LPF(float lowpass_amount);
  void set_value(float new_val);
  void set_value_force(float new_val);
  void set_if_greater(float new_val);

  void set_thresholds(float low, float high);
  void slow_tail_to_zero();
  void set_lowpass_amount(float amount);
  float get();
  
  float get_gated_value();
  void update_gate_value();
  bool gate_changed();
};

#endif
