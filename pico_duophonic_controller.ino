/*
raspberry pi based duophonic mini-synth controller
MIDI to CV with 2 control voltages for dual oscillators
MIDI in is hardware serial
oscillator control voltages use MPC4822 dual 12-bit SPI DAC
White and Pink noise generator, based on Electric Druid's asm implementation, pwm output
LFO with sine and square waveforms, pwm output
Gate / trigger GPIO outputs to drive analogue envelope generators

MIT License

Copyright (c) 2024 petegaggs

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

#include <MIDI.h>
#include <SPI.h>
#include <PWMAudio.h>

// gpio pin definitions
#define MIDI_RX_PIN 13
#define MIDI_TX_PIN 12
#define SPI_CS_PIN 17
#define SPI_SCK_PIN 18
#define SPI_DATA_PIN 19
#define WHITE_NOISE_PIN 2
#define PINK_NOISE_PIN 3
#define LFO_SINE_PIN 4
#define LFO_SQR_PIN 5
#define GATE_1_PIN 6
#define GATE_2_PIN 7
#define GATE_MODE_SW_PIN 8
#define ADC_LFO_SPEED A0

#define TRIGGER_PULSE_MS 20
#define MIDI_BASE_NOTE 12 // C0
#define DAC_SCALE_PER_SEMITONE 42 // 0.504 Volts per octave

MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI);
SPISettings spisettings(100000, MSBFIRST, SPI_MODE0);

typedef bool key_array_t[128];

// sine wave look up table for LFO
const int16_t sine_table[] = {
0,804,1608,2410,3212,4011,4808,5602,6393,7179,7962,8739,9512,10278,11039,
11793,12539,13279,14010,14732,15446,16151,16846,17530,18204,18868,19519,20159,20787,21403,22005,22594,23170,23731,24279,24811,25329,25832,
26319,26790,27245,27683,28105,28510,28898,29268,29621,29956,30273,30571,30852,31113,31356,31580,31785,31971,32137,32285,32412,32521,32609,
32678,32728,32757,32767,32757,32728,32678,32609,32521,32412,32285,32137,31971,31785,31580,31356,31113,30852,30571,30273,29956,29621,29268,
28898,28510,28105,27683,27245,26790,26319,25832,25329,24811,24279,23731,23170,22594,22005,21403,20787,20159,19519,18868,18204,17530,16846,
16151,15446,14732,14010,13279,12539,11793,11039,10278,9512,8739,7962,7179,6393,5602,4808,4011,3212,2410,1608,804,0,-804,-1608,-2410,-3212,
-4011,-4808,-5602,-6393,-7179,-7962,-8739,-9512,-10278,-11039,-11793,-12539,-13279,-14010,-14732,-15446,-16151,-16846,-17530,-18204,-18868,
-19519,-20159,-20787,-21403,-22005,-22594,-23170,-23731,-24279,-24811,-25329,-25832,-26319,-26790,-27245,-27683,-28105,-28510,-28898,-29268,
-29621,-29956,-30273,-30571,-30852,-31113,-31356,-31580,-31785,-31971,-32137,-32285,-32412,-32521,-32609,-32678,-32728,-32757,-32767,-32757,
-32728,-32678,-32609,-32521,-32412,-32285,-32137,-31971,-31785,-31580,-31356,-31113,-30852,-30571,-30273,-29956,-29621,-29268,-28898,-28510,
-28105,-27683,-27245,-26790,-26319,-25832,-25329,-24811,-24279,-23731,-23170,-22594,-22005,-21403,-20787,-20159,-19519,-18868,-18204,-17530,
-16846,-16151,-15446,-14732,-14010,-13279,-12539,-11793,-11039,-10278,-9512,-8739,-7962,-7179,-6393,-5602,-4808,-4011,-3212,-2410,-1608,-804
};

// Store Voss-McCartney algorithm tree in lookup table, a la Electric Druid
const uint8_t tree_table[] = {
	0, 1, 0, 2, 0, 1, 0, 3,
	0, 1, 0, 2, 0, 1, 0, 4,
	0, 1, 0, 2, 0, 1, 0, 3,
	0, 1, 0, 2, 0, 1, 0, 5,
	0, 1, 0, 2, 0, 1, 0, 3,
	0, 1, 0, 2, 0, 1, 0, 4,
	0, 1, 0, 2, 0, 1, 0, 3,
	0, 1, 0, 2, 0, 1, 0, 6,

	0, 1, 0, 2, 0, 1, 0, 3,
	0, 1, 0, 2, 0, 1, 0, 4,
	0, 1, 0, 2, 0, 1, 0, 3,
	0, 1, 0, 2, 0, 1, 0, 5,
	0, 1, 0, 2, 0, 1, 0, 3,
	0, 1, 0, 2, 0, 1, 0, 4,
	0, 1, 0, 2, 0, 1, 0, 3,
	0, 1, 0, 2, 0, 1, 0, 7,

	0, 1, 0, 2, 0, 1, 0, 3,
	0, 1, 0, 2, 0, 1, 0, 4,
	0, 1, 0, 2, 0, 1, 0, 3,
	0, 1, 0, 2, 0, 1, 0, 5,
	0, 1, 0, 2, 0, 1, 0, 3,
	0, 1, 0, 2, 0, 1, 0, 4,
	0, 1, 0, 2, 0, 1, 0, 3,
	0, 1, 0, 2, 0, 1, 0, 6,

	0, 1, 0, 2, 0, 1, 0, 3,
	0, 1, 0, 2, 0, 1, 0, 4,
	0, 1, 0, 2, 0, 1, 0, 3,
	0, 1, 0, 2, 0, 1, 0, 5,
	0, 1, 0, 2, 0, 1, 0, 3,
	0, 1, 0, 2, 0, 1, 0, 4,
	0, 1, 0, 2, 0, 1, 0, 3,
	0, 1, 0, 2, 0, 1, 0, 8
};

// table of the number of bits set
const uint8_t bits_set_table[] = {
 0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, 4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8
};

// Create stereo PWM audio device for white and pink noise
PWMAudio pwm_noise(WHITE_NOISE_PIN, true);

// Create stereo PWM audio device for LFO
PWMAudio pwm_lfo(LFO_SINE_PIN, true);


void find_highest_and_lowest_key_pressed(key_array_t key_array, uint8_t* high_note, uint8_t* low_note) {
  // find the highest and lowest notes pressed. No change if none found
  bool found_lowest = false;
  for (int i = 0; i < 128; i++) {
    if (key_array[i]) {
      *high_note = i;
      if (found_lowest == false) {
        *low_note = i;
        found_lowest = true;
      }
    }
  }
}

uint16_t calc_dac_value (uint8_t midi_note) {
  // compute the 16 bit DAC value corresponding to a midi note number
  return (((uint16_t) midi_note) - MIDI_BASE_NOTE) * DAC_SCALE_PER_SEMITONE;
}

void mpc_4822_write(uint16_t value, bool ch_b) {
  // write one channel of MPC4822
  uint16_t wdata = value & 0xFFF;
  wdata |= 0x1000; // set !SHDN bit high
  wdata |= 0x2000; // set !GA bit high x1 gain. remove this
  if (ch_b) {
    wdata |= 0x8000;
  }
  SPI.beginTransaction(spisettings);
  SPI.transfer16(wdata);
}

void spi_dac_write(uint16_t value_a, uint16_t value_b) {
  // write 2 values to SPI DAC
  mpc_4822_write(value_a, false);
  mpc_4822_write(value_b, true);
}

void set_osc_pitch(uint8_t osc1_midi_note, uint8_t osc2_midi_note) {
  // set the oscillator control voltages
  uint16_t osc1_control, osc2_control;
  osc1_control = calc_dac_value(osc1_midi_note);
  osc2_control = calc_dac_value(osc2_midi_note);
  spi_dac_write(osc1_control, osc2_control);
}

bool get_trigger_mode() {
  // read the switch and return true if in trigger mode
  return (digitalRead(GATE_MODE_SW_PIN) == LOW);
}

int64_t trig_alarm_callback(alarm_id_t id, void *user_data) {
    digitalWrite(GATE_2_PIN, LOW);
    return 0;
}

void gate_on_off(bool gate_on) {
  if (gate_on) {
    digitalWrite(GATE_1_PIN, HIGH);
    digitalWrite(GATE_2_PIN, HIGH);
    if (get_trigger_mode()) {
      // set a timer to clear the trigger in a few ms time
      add_alarm_in_ms(TRIGGER_PULSE_MS, trig_alarm_callback, NULL, false);
    }
  } else {
    digitalWrite(GATE_1_PIN, LOW);
    digitalWrite(GATE_2_PIN, LOW);
  }
}

void note_control(uint8_t pitch, bool note_on) {
  // handles note on and off events
  static key_array_t key_pressed_array = {false};
  static uint8_t osc1_midi_note, osc2_midi_note;
  static uint8_t key_pressed_count = 0;
  if (note_on) {
    key_pressed_array[pitch] = true;
    key_pressed_count++;
    gate_on_off(true);
  } else {
    key_pressed_array[pitch] = false;
    key_pressed_count--;
  }
  if (key_pressed_count > 0) {
    find_highest_and_lowest_key_pressed(key_pressed_array, &osc1_midi_note, &osc2_midi_note);
  } else {
    gate_on_off(false);
  }
  set_osc_pitch(osc1_midi_note, osc2_midi_note);
}

void handleNoteOn(uint8_t channel, uint8_t pitch, uint8_t velocity) {
  // called automatically by MIDI library for note on events
  note_control(pitch, true);
}

void handleNoteOff(uint8_t channel, uint8_t pitch, uint8_t velocity) {
  // called automatically by MIDI library for note off events
  note_control(pitch, false);
}

void noise() {
  // generate white and pink noise and output over PWM
  static uint32_t lfsr_a = 1; // 32 bit LFSR
  static uint32_t lfsr_b = 1; // 31 bit LFSR
  static uint16_t pink_noise = 0;
  static uint16_t sample_cnt = 0;
  uint8_t octave;
  uint16_t bit_mask;
  uint32_t lsb_a, lsb_b;
  uint16_t out_sample_wn, out_sample_pn;
  if (pwm_noise.availableForWrite() > 0) {
    // advance lfsrs
    lsb_a = lfsr_a & 1;
    lfsr_a >>= 1;
    if (lsb_a) {
      lfsr_a ^= 0xA3000000u; // 32 bit
    }
    lsb_b = lfsr_b & 1;
    lfsr_b >>= 1;
    if (lsb_b) {
      lfsr_b ^= 0x78000000u; // 31 bit
    }
    // calculate pink noise using Voss-McCartney algorithm
    if ((sample_cnt & 0x00FF) == 0) {
      octave = tree_table[sample_cnt >> 8] + 8; // octaves 8 to 13
    } else {
      octave = tree_table[sample_cnt & 0xFF]; // octaves 0 to 7
    }
    bit_mask = 1 << octave; // select the appropriate bit
    pink_noise &= ~bit_mask; // clear the bit for this octave
    if (lsb_b) {
      pink_noise |= bit_mask; // set the bit
    }
    // form the pink noise amplitude by summing the 1s
    out_sample_pn = bits_set_table[pink_noise & 0x00FF] + bits_set_table[pink_noise >> 8]; // the value here will be between 0 and 14
    if (lsb_a) {
      out_sample_pn += 1; // add white noise, value is now 0 to 15
    }
    out_sample_pn <<= 11; // convert 4-bit to 16-bit amplitude
    out_sample_pn += 0x8000;
    out_sample_wn = lfsr_a & 0xFFFF; // white noise is just 16-bits of the lfsr 
    // output the noise samples to pwm
    pwm_noise.write(out_sample_wn); // ch0
    pwm_noise.write(out_sample_pn); // ch1
    sample_cnt += 1;
    sample_cnt &= 0x3FFF;
  }
}

uint32_t get_lfo_speed() {
  // read the ADC and calculate LFO tuning word
  uint32_t adc_val = analogRead(ADC_LFO_SPEED);
  return (adc_val << 10) + 1374; // gives around 0.01 to 8Hz range
}

void lfo() {
  // generate LFO waveforms
  static uint32_t lfo_phaccu = 0;     // dds phase accumulator
  static uint32_t lfo_tword_m;        // dds tuning word m
  uint8_t lfo_count;                  // top 8 bits of accumulator is index into table
  uint32_t lfo_count_frac;
  int32_t sample_a, sample_b, sample_diff, delta, interpolated_sample;
  int16_t lfo_sample, sqr_wave;
  if (pwm_lfo.availableForWrite() > 0) {
    lfo_count = lfo_phaccu >> 24;  // use upper 8 bits for phase accu as frequency information
    lfo_count_frac = (lfo_phaccu & 0xFFFFFF) >> 8; // use lower 24 bits as fractional count
    sample_a = sine_table[lfo_count];
    sample_b = lfo_count == 255 ? sine_table[0] : sine_table[lfo_count + 1];
    sample_diff = sample_b - sample_a;
    delta = (sample_diff * lfo_count_frac) >> 16;
    interpolated_sample = sample_a + delta;
    lfo_sample = interpolated_sample;
    // generate square wave. don't really need pwm for this but using pwm allows other waveforms in future
    sqr_wave = lfo_sample > 0 ? 32767 : -32768;
    pwm_lfo.write(lfo_sample); // sine wave ch0
    pwm_lfo.write(sqr_wave); // square wave ch1
    lfo_tword_m = get_lfo_speed();
    lfo_phaccu += lfo_tword_m; // increment phase accumulator  
  }
}

void setup() {
  pinMode(GATE_1_PIN, OUTPUT);
  digitalWrite(GATE_1_PIN, LOW);
  pinMode(GATE_2_PIN, OUTPUT);
  digitalWrite(GATE_2_PIN, LOW);
  pinMode(GATE_MODE_SW_PIN, INPUT_PULLUP);
  Serial1.setRX(MIDI_RX_PIN);
  Serial1.setTX(MIDI_TX_PIN);
  MIDI.begin(MIDI_CHANNEL_OMNI);
  MIDI.setHandleNoteOn(handleNoteOn);
  MIDI.setHandleNoteOff(handleNoteOff);
  SPI.setCS(SPI_CS_PIN);
  SPI.setSCK(SPI_SCK_PIN);
  SPI.setTX(SPI_DATA_PIN);
  SPI.begin(true);
  pwm_noise.setBuffers(4, 32);
  pwm_noise.begin(32000);
  pwm_lfo.setBuffers(4, 32);
  pwm_lfo.begin(32000);
}

void loop() {
  MIDI.read();
  noise();
  lfo();
}
