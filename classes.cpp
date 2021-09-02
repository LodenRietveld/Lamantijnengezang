#include "classes.h"

N_ShapeInterpolator::N_ShapeInterpolator(int number, float low_freq, float high_freq){
    this->number = number;
    this->low_freq = low_freq;
    this->high_freq = high_freq;
    frequency = (float*) malloc(sizeof(float)* number);
    frequency_mod = (float*) malloc(sizeof(float)* number);
    current_value = (float*) malloc(sizeof(float)* number);

    for (int i = 0; i < number; i++){
      frequency[i] = random(low_freq, high_freq);
    }
}

float N_ShapeInterpolator::add_to_frequency(uint8_t idx, float added_amt){
    frequency_mod[idx] = added_amt;
}

void N_ShapeInterpolator::update(){
    phase += 0.0001;
}

float N_ShapeInterpolator::get(uint8_t idx){
    if (idx < number){
      return current_value[idx];
    }

    return -1;
}







N_SineInterpolator::N_SineInterpolator(int number, float low_freq, float high_freq) : N_ShapeInterpolator(number, low_freq, high_freq){

}

void N_SineInterpolator::update(){
    this->phase += 0.0001;

    for (int i = 0; i < number; i++){
        current_value[i] = (sin(this->phase * (frequency[i] + frequency_mod[i])) + 1) / 2.;
    }
}






N_VolumeInterpolator::N_VolumeInterpolator(int number, float low_freq, float high_freq) : N_ShapeInterpolator(number, low_freq, high_freq){

}

float N_VolumeInterpolator::output(uint8_t i){
    float internal_phase = this->phase * (frequency[i] + frequency_mod[i]);
    if (internal_phase < .05){
      return ((1. - cos(internal_phase / .05)) + 1) / 2.;
    } else if (internal_phase >= .05 && internal_phase < .475){
      return 1;
    } else if (internal_phase >= .475 && internal_phase < .525){
      internal_phase -= .475;
      return ((cos(internal_phase / .05)) + 1) / 2.;
    } else {
      return 0;
    }
}

void N_VolumeInterpolator::update(){
    this->phase += 0.0001;

    for (int i = 0; i < number; i++){
      current_value[i] = this->output(i);
    }
}





HAL_SensorTracker::HAL_SensorTracker(uint8_t pin, uint16_t low, uint16_t high, bool is_invert, float scale_amount){
    this->pin = pin;
    this->low = low;
    this->high = high;
    this->is_invert = is_invert;
    this->scale_amount = scale_amount;
}

void HAL_SensorTracker::set_value(uint16_t new_value){
    scale_HAL_values(new_value, scale_amount);
    calculate_delta();
    raw_value = new_value;
}

void HAL_SensorTracker::calculate_delta(){
    delta_value = abs(scaled_value - old_scaled_value);
    old_scaled_value = scaled_value;
}

float HAL_SensorTracker::get_scaled_value(){
    return scaled_value;
}

float HAL_SensorTracker::get_delta(){
    return delta_value;
}

uint8_t HAL_SensorTracker::get_pin(){
    return pin;
}

uint16_t HAL_SensorTracker::get_low(){
    return low;
}

uint16_t HAL_SensorTracker::get_high(){
    return high;
}

float HAL_SensorTracker::scale_HAL_values(uint16_t reading, float range){
    float t_scaled_val = ((reading - low) / ((float) (high - low))) * range;
    if (is_invert){
      scaled_value = 1. - t_scaled_val;
    } else {
      scaled_value = t_scaled_val;
    }
    return t_scaled_val;
}




LPF::LPF(float lowpass_amount){
    this->lowpass_amount = lowpass_amount;
}

void LPF::set_value(float new_val){
    lowpassed = (lowpassed * lowpass_amount) + ((1. - lowpass_amount) * new_val);
}

void LPF::set_value_force(float new_val){
    lowpassed = new_val;
}

void LPF::set_if_greater(float new_val){
    if (new_val > lowpassed){
      lowpassed = new_val;
    }
}

void LPF::slow_tail_to_zero(){
    lowpassed = lowpassed * lowpass_amount;
}

void LPF::set_lowpass_amount(float amount){
    this->lowpass_amount = amount;
}

float LPF::get(){
    return lowpassed;
}
