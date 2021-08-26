#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>


class N_SineInterpolator{
  float phase;
  float low_freq, high_freq;
  int number;
  float* frequency;
  float* frequency_mod;
  float* current_value;

public:
  N_SineInterpolator(int number, float low_freq, float high_freq){
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

  float add_to_frequency(uint8_t idx, float added_amt){
    frequency_mod[idx] = added_amt;
  }

  void update(){
    phase += 0.0001;

    for (int i = 0; i < number; i++){
      current_value[i] = (sin(phase * (frequency[i] + frequency_mod[i])) + 1) / 2.;
    }
  }

  float get(uint8_t idx){
    if (idx < number){
      return current_value[idx];
    }
    
    return -1;
  }
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
  HAL_SensorTracker(uint8_t pin, uint16_t low, uint16_t high, bool is_invert, float scale_amount){
    this->pin = pin;
    this->low = low;
    this->high = high;
    this->is_invert = is_invert;
    this->scale_amount = scale_amount;
  }

  void set_value(uint16_t new_value){
    scale_HAL_values(new_value, scale_amount);
    calculate_delta();
    raw_value = new_value;
  }

  void calculate_delta(){
    delta_value = abs(scaled_value - old_scaled_value);
    old_scaled_value = scaled_value;
  }

  float get_scaled_value(){
    return scaled_value;
  }

  float get_delta(){
    return delta_value;
  }

  uint8_t get_pin(){
    return pin;
  }

  uint16_t get_low(){
    return low;
  }

  uint16_t get_high(){
    return high;
  }

  float scale_HAL_values(uint16_t reading, float range){
    float t_scaled_val = ((reading - low) / ((float) (high - low))) * range;
    if (is_invert){
      scaled_value = 1. - t_scaled_val;
    } else {
      scaled_value = t_scaled_val;
    }
    return t_scaled_val;
  }
};

class LPF {
  float lowpassed = 0;
  float lowpass_amount = 0.97;

public:
  LPF(float lowpass_amount){
    this->lowpass_amount = lowpass_amount;
  }

  void set_value(float new_val){
    lowpassed = (lowpassed * lowpass_amount) + ((1. - lowpass_amount) * new_val);
  }

  void set_value_force(float new_val){
    lowpassed = new_val;
  }

  void set_if_greater(float new_val){
    if (new_val > lowpassed){
      lowpassed = new_val;
    }
  }

  void slow_tail_to_zero(){
#ifdef DEBUG
    Serial.print("Pre: ");
    Serial.print(lowpassed);
    Serial.print("\tfactor: ");
    Serial.println(lowpass_amount);
#endif
    lowpassed = lowpassed * lowpass_amount;
  }

  void set_lowpass_amount(float amount){
    this->lowpass_amount = amount;
  }

  float get(){
    return lowpassed;
  }
};


#define HAL1_PIN                A0
#define HAL2_PIN                A3
#define HAL3_PIN                A4
#define STRETCH_PIN             A2

#define NUM_HAL_SENSORS         3

#define NUM_OSCILLATORS         4
#define NUM_WAVEFORMS           4
#define NUM_WAVE_OSCILLATORS    NUM_WAVEFORMS * NUM_OSCILLATORS
#define NUM_CHORDS              5
#define CHORUS_DELAY_LENGTH     (100*AUDIO_BLOCK_SAMPLES)

#define VIBRATO_UPDATE_TIME     1
#define VIBRATO_AMT             0.01f

#define FILTER_CHANGE_AMT       4.5f

#define HAL_DELTA_PERIOD        10

HAL_SensorTracker hal1(HAL1_PIN, 520, 800, false, 1);
HAL_SensorTracker hal2(HAL2_PIN, 240, 500, true, 1);
HAL_SensorTracker hal3(HAL3_PIN, 520, 800, false, 3);

HAL_SensorTracker* sensors[NUM_HAL_SENSORS] = {&hal1, &hal2, &hal3};

N_SineInterpolator waveform_interpolator_sine(NUM_OSCILLATORS, 0.2, 0.7);

LPF hal1_deltaSmoother(0.9999);
LPF hal2_deltaSmoother(0.9999);

elapsedMillis vibrato_timer = 0;
float vibrato_phase = 0;
float vibrato_speed = 0.0001;
float freq_mult = 1;
float filter_freq_mult = 0;

elapsedMillis hal_delta_time = 0;


short chorus_array_l[CHORUS_DELAY_LENGTH];
short chorus_array_r[CHORUS_DELAY_LENGTH];

uint8_t notes[NUM_CHORDS][NUM_OSCILLATORS] = {{48, 52, 55, 59}, {50, 52, 55, 57}, {50, 52, 57, 62}, {52, 55, 57, 60}, {50, 52, 55, 60}};
float note_freqs[NUM_OSCILLATORS];
float filter_freqs[NUM_OSCILLATORS];

uint8_t waveforms[] = {WAVEFORM_SINE, WAVEFORM_TRIANGLE, WAVEFORM_SAWTOOTH, WAVEFORM_SQUARE};

// GUItool: begin automatically generated code
AudioSynthWaveform       waveform16;     //xy=159,594
AudioSynthWaveform       waveform15;     //xy=162,558
AudioSynthWaveform       waveform14;     //xy=163,523
AudioSynthWaveform       waveform13;     //xy=168,488
AudioSynthWaveform       waveform9;      //xy=171,353
AudioSynthWaveform       waveform12;     //xy=171,451
AudioSynthWaveform       waveform11;     //xy=172,419
AudioSynthWaveform       waveform7;      //xy=173,283
AudioSynthWaveform       waveform8;      //xy=173,315
AudioSynthWaveform       waveform6;      //xy=174,246
AudioSynthWaveform       waveform1;      //xy=175,70
AudioSynthWaveform       waveform2;      //xy=175,103
AudioSynthWaveform       waveform3;      //xy=175,135
AudioSynthWaveform       waveform10;     //xy=174,387
AudioSynthWaveform       waveform4;      //xy=175,167
AudioSynthWaveform       waveform5;      //xy=175,210
AudioMixer4              mixer2;         //xy=323,249
AudioMixer4              mixer3;         //xy=325,407
AudioMixer4              mixer4;         //xy=326,537
AudioMixer4              mixer1;         //xy=331,116
AudioMixer4              mixer5;         //xy=547,330

AudioMixer4              verbMixerL;
AudioMixer4              verbMixerR;
AudioFilterBiquad        filter1;
AudioFilterBiquad        filter2;
AudioFilterBiquad        filter3;
AudioFilterBiquad        filter4;

AudioSynthNoiseWhite     noiseAM;
AudioEffectMultiply      AMEffect;
AudioSynthWaveformDc     noise_offset;
AudioMixer4              noise_mixer;


AudioEffectChorus        chorus1;        //xy=712,275
AudioEffectChorus        chorus2;        //xy=714,355

AudioMixer4              chorusMixer1;
AudioMixer4              chorusMixer2;

AudioMixer4              mixer8;         //xy=875,374
AudioMixer4              mixer7;         //xy=876,294
AudioEffectGranular      granular1;      //xy=1014,272
AudioEffectGranular      granular2;      //xy=1015,389
AudioEffectFreeverb      freeverb1;      //xy=1152,272
AudioEffectFreeverb      freeverb2;      //xy=1152,389
AudioMixer4              mixer6;         //xy=1288,326
AudioOutputI2S           i2s1;           //xy=1508,322

AudioAnalyzePeak peak;

AudioMixer4*             osc_mixers[4] = {&mixer1, &mixer2, &mixer3, &mixer4};
AudioSynthWaveform*      wave_oscillators[NUM_WAVE_OSCILLATORS] = {&waveform1, &waveform2, &waveform3, &waveform4, &waveform5, &waveform6, &waveform7, &waveform8, &waveform9, &waveform10, &waveform11, &waveform12, &waveform13, &waveform14, &waveform15, &waveform16};
AudioFilterBiquad*       filters[4] = {&filter1, &filter2, &filter3, &filter4};


AudioConnection          patchCord1(waveform16, 0, mixer4, 3);
AudioConnection          patchCord2(waveform15, 0, mixer4, 2);
AudioConnection          patchCord3(waveform14, 0, mixer4, 1);
AudioConnection          patchCord4(waveform13, 0, mixer4, 0);
AudioConnection          patchCord5(waveform9, 0, mixer3, 0);
AudioConnection          patchCord6(waveform12, 0, mixer3, 3);
AudioConnection          patchCord7(waveform11, 0, mixer3, 2);
AudioConnection          patchCord8(waveform7, 0, mixer2, 2);
AudioConnection          patchCord9(waveform8, 0, mixer2, 3);
AudioConnection          patchCord10(waveform6, 0, mixer2, 1);
AudioConnection          patchCord11(waveform1, 0, mixer1, 0);
AudioConnection          patchCord12(waveform2, 0, mixer1, 1);
AudioConnection          patchCord13(waveform3, 0, mixer1, 2);
AudioConnection          patchCord14(waveform10, 0, mixer3, 1);
AudioConnection          patchCord15(waveform4, 0, mixer1, 3);
AudioConnection          patchCord16(waveform5, 0, mixer2, 0);

AudioConnection          patchCord51(mixer1, 0, filter1, 0);
AudioConnection          patchCord52(mixer2, 0, filter2, 0);
AudioConnection          patchCord53(mixer3, 0, filter3, 0);
AudioConnection          patchCord54(mixer4, 0, filter4, 0);



AudioConnection          patchCord20(filter1, 0, mixer5, 0);
AudioConnection          patchCord17(filter2, 0, mixer5, 1);
AudioConnection          patchCord18(filter3, 0, mixer5, 2);
AudioConnection          patchCord19(filter4, 0, mixer5, 3);

//AudioConnection          patchCord50(waveform1, 0, peak, 0);
AudioConnection          patchCord21(mixer5, 0, AMEffect, 0);
AudioConnection          patchCord60(noise_offset, 0, noise_mixer, 0);
AudioConnection          patchCord61(noiseAM, 0, noise_mixer, 1);
AudioConnection          patchCord63(noise_mixer, 0, AMEffect, 1);
AudioConnection          patchCord64(AMEffect, chorus1);
AudioConnection          patchCord65(AMEffect, chorus2);

AudioConnection          patchCord70(AMEffect, 0, chorusMixer1, 0);
AudioConnection          patchCord71(chorus1, 0, chorusMixer1, 1);
AudioConnection          patchCord72(AMEffect, 0, chorusMixer2, 0);
AudioConnection          patchCord73(chorus2, 0, chorusMixer2, 1);

AudioConnection          patchCord23(chorusMixer1, 0, mixer7, 0);
AudioConnection          patchCord24(chorusMixer2, 0, mixer8, 0);
AudioConnection          patchCord25(mixer8, freeverb2);
AudioConnection          patchCord26(mixer7, freeverb1);
//AudioConnection          patchCord25(mixer8, mixer6);
//AudioConnection          patchCord26(mixer7, mixer6);
//AudioConnection          patchCord27(granular1, freeverb1);
//AudioConnection          patchCord28(granular2, freeverb2);

AudioConnection          verbMixCordL(freeverb1, 0, verbMixerL, 0);
AudioConnection          verbMixCordR(freeverb2, 0, verbMixerR, 0);
AudioConnection          dryMixCordL(mixer7, 0, verbMixerL, 1);
AudioConnection          dryMixCordR(mixer8, 0, verbMixerR, 1);

AudioConnection          patchCord29(freeverb1, 0, mixer6, 1);
AudioConnection          patchCord30(freeverb2, 0, mixer6, 2);
//AudioConnection          patchCord31(mixer6, 0, mixer7, 1);
//AudioConnection          patchCord32(mixer6, 0, mixer8, 1);
AudioConnection          patchCord33(mixer6, 0, i2s1, 0);
AudioConnection          patchCord34(mixer6, 0, i2s1, 1);
AudioControlSGTL5000     sgtl5000_1;     //xy=1382,676


void setup() {
  // put your setup code here, to run once:
  AudioMemory(20);
  
  sgtl5000_1.enable();
  sgtl5000_1.volume(0.5);

  Serial.begin(115200);
  

  for (int i = 0; i < NUM_OSCILLATORS; i++){
#ifdef DEBUG
    Serial.print("oscillator: ");
    Serial.print(i);
    Serial.print("\t note: ");
    Serial.print(notes[i]);
    Serial.print("\t frequency: ");
    Serial.println(mtof(notes[i]));
#endif
    note_freqs[i] = mtof(notes[0][i]);
    filter_freqs[i] = note_freqs[i];
    filters[i]->setLowpass(0, filter_freqs[i] , 0.7);
    filters[i]->setLowpass(1, filter_freqs[i] , 0.7);
    filters[i]->setLowpass(2, 15000., 0.7);
    filters[i]->setLowpass(3, 15000., 0.7);
  }

  for (int i = 0; i < NUM_WAVE_OSCILLATORS; i++){
    wave_oscillators[i]->begin(0.2, note_freqs[i / NUM_OSCILLATORS], waveforms[i % NUM_WAVEFORMS]);
#ifdef DEBUG
    Serial.print("oscillator: ");
    Serial.print(i/NUM_OSCILLATORS);
    Serial.print("\t frequency: ");
    Serial.println(note_freqs[i/NUM_OSCILLATORS]);
#endif
    osc_mixers[i / NUM_OSCILLATORS]->gain(i % NUM_WAVEFORMS, (i % NUM_WAVEFORMS) == 0 ? 1 : 0);
  }

  mixer5.gain(0, .3);
  mixer5.gain(1, .3);
  mixer5.gain(2, .3);
  mixer5.gain(3, .3);

  noise_mixer.gain(0, .8);
  noise_mixer.gain(1, .2);
  noiseAM.amplitude(1.);
  noise_offset.amplitude(1.);

  verbMixerL.gain(0, 0.3);
  verbMixerL.gain(1, 0.3);
  verbMixerR.gain(0, 0.3);
  verbMixerR.gain(1, 0.3);

  chorus1.begin(chorus_array_l, CHORUS_DELAY_LENGTH, 2);
  chorus2.begin(chorus_array_r, CHORUS_DELAY_LENGTH, 2);

  chorusMixer1.gain(0, 0.0);
  chorusMixer1.gain(1, 0.5);

  chorusMixer2.gain(0, 0.0);
  chorusMixer2.gain(1, 0.5);  

  freeverb1.roomsize(1.);
  freeverb2.roomsize(1.);

  mixer7.gain(0, 1.);
  mixer7.gain(1, 0.);
  mixer8.gain(0, 1.);
  mixer8.gain(1, 0.);

  mixer6.gain(1, 0.5);
  mixer6.gain(2, 0.5);
  
  waveform_interpolator_sine.update();
}

uint16_t loopcount = 0;


void loop() {
  update_vibrato();
  AudioNoInterrupts();

  set_filter_oscillator_freqs();
  set_effect_mixers();

  AudioInterrupts();

  read_sensors_update_data_structures();
  set_values_and_interpolators();
  update_trackers();
}

void interpolate_waveforms(N_SineInterpolator* interpolator){
  for (int i = 0; i < NUM_OSCILLATORS; i++){
    float waveform_interpolator = interpolator->get(i);
    
    uint8_t this_waveform = waveform_interpolator;
    uint8_t next_waveform = this_waveform + 1;
    for (int j = 0; j < NUM_WAVEFORMS; j++){
      if (j == this_waveform){
        osc_mixers[i]->gain(j, 1. - (waveform_interpolator - this_waveform));
      } else if (j == next_waveform){
        osc_mixers[i]->gain(j, waveform_interpolator - this_waveform);
      } else {
        osc_mixers[i]->gain(j, 0);
      }
    }
  }
}

void interpolate_notes(float interpolator){
  int idx_1 = (int)interpolator;
  int idx_2 = idx_1 + 1;
  
  float bounded_interpolator = constrain(((interpolator - idx_1) * 2) - .5, 0., 1.);

  for (int i = 0; i < NUM_OSCILLATORS; i++){
    float base_freq = mtof(notes[idx_1][i]);
    float freq_diff = abs(mtof(notes[idx_1][i]) - mtof(notes[idx_2][i]));
    
    note_freqs[i] = base_freq + (freq_diff * bounded_interpolator);
  }
}



void update_vibrato(){
  if (vibrato_timer > VIBRATO_UPDATE_TIME-1){
    freq_mult = (sin(vibrato_phase) * (VIBRATO_AMT + (hal2_deltaSmoother.get() * 0.02))) + 1.;
    vibrato_phase += vibrato_speed;
    vibrato_timer = 0;
  }
}

void set_filter_oscillator_freqs(){
  for (int i = 0; i < NUM_OSCILLATORS; i++){
    filters[i]->setLowpass(0, filter_freqs[i] * (filter_freq_mult + 1) * FILTER_CHANGE_AMT , 0.7);
    filters[i]->setLowpass(1, filter_freqs[i] * (filter_freq_mult + 1) * FILTER_CHANGE_AMT , 0.7);
  }

  for (int i = 0; i < NUM_WAVE_OSCILLATORS; i++){
    wave_oscillators[i]->frequency(note_freqs[i / NUM_OSCILLATORS] * freq_mult);
  }
}

void set_effect_mixers(){
  float delta_diff = constrain(abs(hal1_deltaSmoother.get() - hal2_deltaSmoother.get()), 0., 1.);
  
  chorusMixer1.gain(0, delta_diff);
  chorusMixer1.gain(1, 1. - delta_diff);
  chorusMixer2.gain(0, delta_diff);
  chorusMixer2.gain(1, 1. - delta_diff);
  verbMixerL.gain(1, 1. - hal1.get_scaled_value());
  verbMixerL.gain(0, hal1.get_scaled_value());
  verbMixerR.gain(1, 1. - hal1.get_scaled_value());
  verbMixerR.gain(0, hal1.get_scaled_value());
  noise_mixer.gain(0, 1. - (hal1.get_scaled_value() / 5.));
  noise_mixer.gain(1, hal1.get_scaled_value() / 5.);
}

void read_sensors_update_data_structures(){
    if (hal_delta_time > HAL_DELTA_PERIOD){
    for (int i = 0; i < NUM_HAL_SENSORS; i++){
      sensors[i]->set_value(analogRead(sensors[i]->get_pin()));
      hal_delta_time = 0;
    }
  }

  hal1_deltaSmoother.set_if_greater(hal1.get_delta());
  hal2_deltaSmoother.set_if_greater(hal2.get_delta());
}

void set_values_and_interpolators(){
  float chord_interpolator = constrain(hal2.get_scaled_value(), 0., 0.999999) * 5;
  filter_freq_mult = hal1.get_delta() + hal1_deltaSmoother.get();

  interpolate_waveforms(&waveform_interpolator_sine);
  interpolate_notes(chord_interpolator);
}

void update_trackers(){
  hal1_deltaSmoother.slow_tail_to_zero();
  hal2_deltaSmoother.slow_tail_to_zero();
  waveform_interpolator_sine.update();
}






///////////UTIL////////////



float mtof(uint8_t midival) {
  return 8.1757989156 * pow(2.0, midival/12.0);
}


float scale_HAL_values(HAL_SensorTracker* sensor, uint16_t reading, float range){
  return ((reading - sensor->get_low()) / ((float)sensor->get_high() - sensor->get_low())) * range;
}


float scale_STRETCH_values(uint16_t reading, float range){
  if (reading < 751){
    return (1. - ((reading - 650) / 120.)) * range;
  } else {
    return filter_freq_mult; 
  }
}


float debug_print_value(char* label, float minval, float maxval, float val, bool line_end){
  char strbuf[100];
  
  strcpy(strbuf, label);
  strcat(strbuf, "_min:");
  Serial.print(strbuf);
  Serial.print(minval);
  
  strcpy(strbuf, ",");
  strcat(strbuf, label);
  strcat(strbuf, "_max:");
  Serial.print(strbuf);
  Serial.print(maxval);
  
  Serial.print(",");
  Serial.print(label);
  Serial.print(":");
  if (line_end){
    Serial.println(val);
  } else {
    Serial.print(val);
    Serial.print(",");
  }
}
