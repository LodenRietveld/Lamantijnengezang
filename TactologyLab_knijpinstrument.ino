#include <Audio.h>
#include "classes.h"

#define HAL1_PIN                A0
#define HAL2_PIN                A3
#define HAL3_PIN                A4
#define STRETCH_PIN             A2
#define ON_OFF_PIN              2

#define NUM_HAL_SENSORS         2
#define MEAN_COUNT              50

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

HAL_SensorTracker* sensors[NUM_HAL_SENSORS] = {&hal1, &hal2};

N_SineInterpolator waveform_interpolator_sine(NUM_OSCILLATORS, 0.2, 0.7);
N_VolumeInterpolator volume_interpolator(1, 0.1, 0.2);

LPF hal1_deltaSmoother(0.9999);
LPF hal2_deltaSmoother(0.9999);

LPF volume_fade(0.99999);
LPF btn_debounce(0.95);

float parts[NUM_HAL_SENSORS][MEAN_COUNT] = {0};
uint8_t part_idx = 0;
float sensor_mean = 0.;
float sensor_variance[NUM_HAL_SENSORS] = {0};

float instr_volume = 0;
bool changing_volume = false;

elapsedMillis vibrato_timer = 0;
float vibrato_phase = 0;
float vibrato_speed = 0.0001;
float freq_mult = 1;
float filter_freq_mult = 0;
float chord_interpolator = 0;

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

AudioConnection          patchCord29(verbMixerL, 0, mixer6, 1);
AudioConnection          patchCord30(verbMixerR, 0, mixer6, 2);
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
    note_freqs[i] = mtof(notes[0][i]);
    filter_freqs[i] = note_freqs[i];
    filters[i]->setLowpass(0, filter_freqs[i] , 0.7);
    filters[i]->setLowpass(1, filter_freqs[i] , 0.7);
    filters[i]->setLowpass(2, 15000., 0.7);
    filters[i]->setLowpass(3, 15000., 0.7);
  }

  for (int i = 0; i < NUM_WAVE_OSCILLATORS; i++){
    wave_oscillators[i]->begin(0.2, note_freqs[i / NUM_OSCILLATORS], waveforms[i % NUM_WAVEFORMS]);
    osc_mixers[i / NUM_OSCILLATORS]->gain(i % NUM_WAVEFORMS, (i % NUM_WAVEFORMS) == 0 ? 1 : 0);
  }

  mixer5.gain(0, .5);
  mixer5.gain(1, .5);
  mixer5.gain(2, .5);
  mixer5.gain(3, .5);

  noise_mixer.gain(0, .8);
  noise_mixer.gain(1, .2);
  noiseAM.amplitude(1.);
  noise_offset.amplitude(1.);

  verbMixerL.gain(0, 0.5);
  verbMixerL.gain(1, 0.5);
  verbMixerR.gain(0, 0.5);
  verbMixerR.gain(1, 0.5);

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

  mixer6.gain(1, 0.7);
  mixer6.gain(2, 0.7);
  
  waveform_interpolator_sine.update();

  pinMode(ON_OFF_PIN, INPUT_PULLUP);
  btn_debounce.set_thresholds(0.3, 0.6);
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
  change_volume();
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
  float scaled_hal2_val = constrain(hal2.get_scaled_value() * 4, 0, 1);
  
  chorusMixer1.gain(0, delta_diff);
  chorusMixer1.gain(1, 1. - delta_diff);
  chorusMixer2.gain(0, delta_diff);
  chorusMixer2.gain(1, 1. - delta_diff);
  verbMixerL.gain(1, scaled_hal2_val);
  verbMixerL.gain(0, 1. - scaled_hal2_val);
  verbMixerR.gain(1, scaled_hal2_val);
  verbMixerR.gain(0, 1. - scaled_hal2_val);
  noise_mixer.gain(0, 1. - (hal1.get_scaled_value() / 5.));
  noise_mixer.gain(1, hal1.get_scaled_value() / 5.);
}

void read_sensors_update_data_structures(){    
    if (hal_delta_time > HAL_DELTA_PERIOD){      
      for (int i = 0; i < NUM_HAL_SENSORS; i++){
        sensors[i]->set_value(analogRead(sensors[i]->get_pin()));
        hal_delta_time = 0;

        parts[i][part_idx] = sensors[i]->get_scaled_value();
      }

      
//      debug_print_value("val1", 0., 1., sensors[0]->get_scaled_value(), false);
//      debug_print_value("val2", 0., 1., sensors[1]->get_scaled_value(), true);
      
      calculate_sensor_variance();
      
      part_idx++;
      if (part_idx >= MEAN_COUNT){
        part_idx = 0;
      }
  }

  hal1_deltaSmoother.set_if_greater(hal1.get_delta());
  hal2_deltaSmoother.set_if_greater(hal2.get_delta());

  btn_debounce.set_value(1 - digitalRead(ON_OFF_PIN));
  btn_debounce.update_gate_value();

  bool gate_c = btn_debounce.gate_changed();

//  if (millis()%16 == 0 || gate_c){
//    debug_print_value("btn_raw", 0, 1, 1 - digitalRead(ON_OFF_PIN), false, false);
//    debug_print_value("btn_smth", 0, 1, btn_debounce.get(), false, false);
//    debug_print_value("btn_gate", 0, 1, btn_debounce.get_gated_value(), false, false);
//    debug_print_value("volume", 0, 0, volume_fade.get() + 1.5, true, false);
//  }

  if (gate_c){
    instr_volume = (btn_debounce.get_gated_value() * 0.6) + 0.2;
    changing_volume = true;
  }
}


void calculate_sensor_variance(){
  float variance_mean;
  
  for (int sensor_idx = 0; sensor_idx < NUM_HAL_SENSORS; sensor_idx++){
    sensor_mean = 0;
    
    for (int i = 0; i < MEAN_COUNT; i++){
      sensor_mean += parts[sensor_idx][i];
    }

    sensor_mean /= MEAN_COUNT;

    variance_mean = 0;

    for (int i = 0; i < MEAN_COUNT; i++){
      variance_mean += sq(parts[sensor_idx][i] - sensor_mean) * 10;
    }

    variance_mean /= MEAN_COUNT;

    sensor_variance[sensor_idx] = variance_mean;
  }

//  debug_print_value("variance1", 0., 1., sensor_variance[0], false);
//  debug_print_value("variance2", 0., 1., sensor_variance[1], true);
  
}

void set_values_and_interpolators(){
  if (hal2.get_delta() < 0.01){
    chord_interpolator = constrain(hal2.get_scaled_value(), 0., 0.999999) * 5;
  }
  filter_freq_mult = hal1.get_delta() * 100 + hal1_deltaSmoother.get() * 5;

//  debug_print_value("delta", 0., 1., hal1.get_delta(), true);
  filter_freq_mult += hal1.get_scaled_value() * 3;

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


float debug_print_value(char* label, float minval, float maxval, float val, bool line_end, bool min_max){
  char strbuf[100];

  if (min_max){
    strcpy(strbuf, label);
    strcat(strbuf, "_min:");
    Serial.print(strbuf);
    Serial.print(minval);
    
    strcpy(strbuf, "\t");
    strcat(strbuf, label);
    strcat(strbuf, "_max:");
    Serial.print(strbuf);
    Serial.print(maxval);
  
    Serial.print("\t");
  }
  Serial.print(label);
  Serial.print(":");
  if (line_end){
    Serial.println(val);
  } else {
    Serial.print(val);
    Serial.print("\t");
  }
}

void change_volume(){
  if (changing_volume){
    volume_fade.set_value(instr_volume);

    if (volume_fade.get() == instr_volume){
      changing_volume = false;
    }
  }

  mixer6.gain(0, volume_fade.get());
  mixer6.gain(1, volume_fade.get());
}
