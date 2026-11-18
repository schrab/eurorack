// Copyright 2014 Emilie Gillet.
//
// Author: Emilie Gillet (emilie.o.gillet@gmail.com)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
// 
// See http://creativecommons.org/licenses/MIT/ for more information.
//
// -----------------------------------------------------------------------------
//
// Group of voices.

#include "elements/dsp/part.h"
#include "helpers/tbd_dist_fast.h"

#include <atomic>
#include "esp_log.h"
#include "elements/resources.h"
#include "stmlib/utils/dsp.h"

namespace elements {
std::atomic<uint32_t> g_panic_count{0};
std::atomic<uint32_t> g_max_resonator_level_x10{0};
}

extern "C" uint32_t elements_debug_get_panic_count() {
  return elements::g_panic_count.load(std::memory_order_relaxed);
}

extern "C" float elements_debug_get_max_resonator_level() {
  return elements::g_max_resonator_level_x10.load(std::memory_order_relaxed) * 0.1f;
}

// Retroactor's Vult-generated Befaco wavefolder (vult_formantor.cpp,
// compiled in the ctagSoundProcessor component; also used by Formantor,
// Subbotnik, Freakwaves, Bjorklund). Stateless, external linkage — the
// distortion stage borrows it instead of duplicating the circuit.
extern float Fold_do(float signal, float level);

// braids' per-unit sine table (braids/resources.cc, same component build).
// Referenced through an extern declaration so the elements TU never
// includes braids/resources.h — its LUT_* macros collide with the
// elements ones (the documented deferred collision class).
namespace braids { extern const int16_t wav_sine[]; }

namespace elements {

using namespace std;
using namespace stmlib;

void Part::Init(uint16_t* reverb_buffer) {
  patch_.exciter_envelope_shape = 1.0f;
  patch_.exciter_bow_level = 0.0f;
  patch_.exciter_bow_timbre = 0.5f;
  patch_.exciter_blow_level = 0.0f;
  patch_.exciter_blow_meta = 0.5f;
  patch_.exciter_blow_timbre = 0.5f;
  patch_.exciter_strike_level = 0.8f;
  patch_.exciter_strike_meta = 0.5f;
  patch_.exciter_strike_timbre = 0.5f;
  patch_.exciter_signature = 0.0f;
  patch_.resonator_geometry = 0.2f;
  patch_.resonator_brightness = 0.5f;
  patch_.resonator_damping = 0.25f;
  patch_.resonator_position = 0.3f;
  patch_.resonator_modulation_frequency = 0.5f / kSampleRate;
  patch_.resonator_modulation_offset = 0.1f;
  patch_.reverb_diffusion = 0.625f;
  patch_.reverb_lp = 0.7f;
  patch_.space = 0.5f;
  patch_.dist_type = 0.0f;
  patch_.dist_drive = 0.0f;
  patch_.dist_bias = 0.0f;
  patch_.dist_sustain = 0.0f;
  patch_.dist_tone = 1.0f;
  patch_.dist_mix = 1.0f;
  dist_lp_[0] = dist_lp_[1] = 0.0f;
  dist_env_[0] = dist_env_[1] = 0.0f;
  dist_chainsaw_.Init(32000.0f);
  // Type 3 transfer table: braids SignatureWaveshaper::Init verbatim
  // (int16/int32 semantics included — the uint16 bumplet_gain wrap at the
  // extremes is part of the curve), seeded 0xcafe = the exact curve TBD03
  // ships with.
  {
    uint32_t seed = 0xcafe;
    int32_t skew = seed & 15;
    seed >>= 4;
    int32_t sigmoid_strength = seed & 31;
    seed >>= 5;
    int32_t bumplets_frequency = seed & 3;
    seed >>= 2;
    bumplets_frequency += 3;
    int32_t bumplets_width = seed & 7;
    seed >>= 3;
    bumplets_width += 1;
    bumplets_width <<= 7;
    bumplets_width *= bumplets_width;
    for (int i = 0; i < 256; ++i) {
      int16_t x = (i - 128) << 8;
      int16_t x_skew = i * i - 32768;
      x = stmlib::Mix(x, x_skew, skew << 11);
      int16_t sigmoid = x * (8192 + (sigmoid_strength << 10)) /
          (8192 + (sigmoid_strength * abs(x) >> 5));
      int16_t bumplets = braids::wav_sine[(i * bumplets_frequency) & 255];
      uint16_t bumplet_gain = x * x / (bumplets_width) + 16;
      bumplet_gain = 32768 * 128 / (128 + bumplet_gain);
      dist_ws_[i] = stmlib::Mix(sigmoid, bumplets, bumplet_gain);
    }
    dist_ws_[256] = dist_ws_[255];
  }
  previous_gate_ = false;
  active_voice_ = 0;
  
  fill(&silence_[0], &silence_[kMaxBlockSize], 0.0f);
  fill(&note_[0], &note_[kNumVoices], 69.0f);
  
  for (size_t i = 0; i < kNumVoices; ++i) {
    voice_[i].Init();
    ominous_voice_[i].Init();
  }
  
  reverb_.Init(reverb_buffer);

  scaled_exciter_level_ = 0.0f;
  scaled_resonator_level_ = 0.0f;
  resonator_level_ = 0.0f;
  
  bypass_ = false;
  
  resonator_model_ = RESONATOR_MODEL_MODAL;
}

void Part::Seed(uint32_t* seed, size_t size) {
  // Scramble all bits from the serial number.
  uint32_t signature = 0xf0cacc1a;
  for (size_t i = 0; i < size; ++i) {
    signature ^= seed[i];
    signature = signature * 1664525L + 1013904223L;
  }
  float x;

  x = static_cast<float>(signature & 7) / 8.0f;
  signature >>= 3;
  patch_.resonator_modulation_frequency = (0.4f + 0.8f * x) / kSampleRate;
  
  x = static_cast<float>(signature & 7) / 8.0f;
  signature >>= 3;
  patch_.resonator_modulation_offset = 0.05f + 0.1f * x;

  x = static_cast<float>(signature & 7) / 8.0f;
  signature >>= 3;
  patch_.reverb_diffusion = 0.55f + 0.15f * x;

  x = static_cast<float>(signature & 7) / 8.0f;
  signature >>= 3;
  patch_.reverb_lp = 0.7f + 0.2f * x;

  x = static_cast<float>(signature & 7) / 8.0f;
  signature >>= 3;
  patch_.exciter_signature = x;
}

void Part::Process(
    const PerformanceState& performance_state,
    const float* blow_in,
    const float* strike_in,
    float* main,
    float* aux,
    size_t size) {

  // Copy inputs to outputs when bypass mode is enabled.
  if (bypass_ || panic_) {
    if (panic_) {
      // If the resonator is blowing up (this has been observed once before
      // corrective action was taken), reset the state of the filters to 0
      // to prevent the module to freeze with resonators' state blocked at NaN.
      ESP_LOGW("Elements", "PANIC RESET: voice resonators cleared to 0 (resonator_level was %.2f)", resonator_level_);
      for (size_t i = 0; i < kNumVoices; ++i) {
        voice_[i].Panic();
      }
      resonator_level_ = 0.0f;
      panic_ = false;
    }
    copy(&blow_in[0], &blow_in[size], &aux[0]);
    copy(&strike_in[0], &strike_in[size], &main[0]);
    return;
  }

  // When a new note is played, cycle to the next voice.
  if (performance_state.gate && !previous_gate_) {
    ++active_voice_;
    if (active_voice_ >= kNumVoices) {
      active_voice_ = 0;
    }
  }
  
  previous_gate_ = performance_state.gate;
  note_[active_voice_] = performance_state.note;
  fill(&main[0], &main[size], 0.0f);
  fill(&aux[0], &aux[size], 0.0f);
  
  // Compute the stereo spread and reverb parameters from the "space"
  // metaparameter. NOTE (TBD): upstream Elements below space 0.1 blends the
  // RAW EXCITER onto the aux output (raw_gain) — the real module's "patch
  // the exciter out of AUX" feature. This instrument's outputs go straight
  // to the codec/speakers, where those exciter pulses are heard as clicks
  // in the right channel, so the blend is removed and both outputs carry
  // resonator signal only. The exciter stays available internally — the
  // Warps stage taps raw_buffer_ directly.
  float space = patch_.space >= 1.0f ? 1.0f : patch_.space;
  space = space >= 0.1f ? space - 0.1f : 0.0f;
  float spread = space <= 0.7f ? space : 0.7f;
  float reverb_amount = space >= 0.5f ? 1.0f * (space - 0.5f) : 0.0f;
  float reverb_time = 0.35f + 1.2f * reverb_amount;
  
  // Render each voice.
  for (size_t i = 0; i < kNumVoices; ++i) {
    float midi_pitch = note_[i] + performance_state.modulation;
    if (easter_egg_) {
      ominous_voice_[i].Process(
          patch_,
          midi_pitch,
          performance_state.strength,
          i == active_voice_ && performance_state.gate,
          (i == active_voice_) ? blow_in : silence_,
          (i == active_voice_) ? strike_in : silence_,
          raw_buffer_,
          center_buffer_,
          sides_buffer_,
          size);
    } else {
      // Convert the MIDI pitch to a frequency.
      int32_t pitch = static_cast<int32_t>((midi_pitch + 48.0f) * 256.0f);
      if (pitch < 0) {
        pitch = 0;
      } else if (pitch >= 65535) {
        pitch = 65535;
      }
      voice_[i].set_resonator_model(resonator_model_);
      // Render the voice signal.
      voice_[i].Process(
          patch_,
          lut_midi_to_f_high[pitch >> 8] * lut_midi_to_f_low[pitch & 0xff],
          performance_state.strength,
          i == active_voice_ && performance_state.gate,
          (i == active_voice_) ? blow_in : silence_,
          (i == active_voice_) ? strike_in : silence_,
          raw_buffer_,
          center_buffer_,
          sides_buffer_,
          size);
    }
    
    // Mixdown: a center/side (M-S) stereo pair, resonator signal only.
    for (size_t j = 0; j < size; ++j) {
      float side = sides_buffer_[j] * spread;
      main[j] += center_buffer_[j] - side;
      aux[j] += center_buffer_[j] + side;
    }
  }
  
  // Pre-clipping
  if (!easter_egg_) {
    for (size_t i = 0; i < size; ++i) {
      main[i] = SoftLimit(main[i]);
      aux[i] = SoftLimit(aux[i]);
    }
  }
  
  // Metering.
  float exciter_level = voice_[active_voice_].exciter_level();
  float resonator_level = resonator_level_;
  for (size_t i = 0; i < size; ++i) {
    float error = main[i] * main[i] - resonator_level;
    resonator_level += error * (error > 0.0f ? 0.05f : 0.0005f);
  }
  resonator_level_ = resonator_level;
  uint32_t cur_x10 = static_cast<uint32_t>(resonator_level * 10.0f);
  uint32_t prev_max = elements::g_max_resonator_level_x10.load(std::memory_order_relaxed);
  while (cur_x10 > prev_max && !elements::g_max_resonator_level_x10.compare_exchange_weak(prev_max, cur_x10, std::memory_order_relaxed)) {}

  if (resonator_level >= 200.0f) {
    panic_ = true;
    elements::g_panic_count.fetch_add(1, std::memory_order_relaxed);
    ESP_LOGW("Elements", "PANIC TRIGGERED! resonator_level=%.2f >= 200.0 (count=%lu)",
             resonator_level, (unsigned long)elements::g_panic_count.load(std::memory_order_relaxed));
  } else {
    static uint32_t s_block_cnt = 0;
    if ((++s_block_cnt & 1023) == 0 && resonator_level > 20.0f) {
      ESP_LOGW("Elements", "HIGH resonator_level=%.2f (panic at 200)", resonator_level);
    }
  }
  
  if (easter_egg_) {
    float l = (patch_.exciter_blow_level + patch_.exciter_strike_level) * 0.5f;
    scaled_exciter_level_ = l * (2.0f - l);
  } else {
    exciter_level *= 16.0f;
    scaled_exciter_level_ = exciter_level > 0.1f ? 1.0f : exciter_level;
  }

  resonator_level *= 16.0f;
  scaled_resonator_level_ = resonator_level < 1.0f ? resonator_level : 1.0f;

  // Simple stereo distortion stage (TBD), four characters on one Type
  // selector — same curve on both channels, followed by a 1-pole tone LP
  // and a dry/wet blend:
  //   0 tube: SoftClip(k*x)/SoftClip(k) — the stmlib tanh-family rational
  //     (unity small-signal gain, +-1 asymptote, one division per BLOCK in
  //     the normalization): punch-preserving glue, odd harmonics.
  //   1 fold: Fold_do(x, d) — Retroactor's Vult/Befaco wavefolder (identity
  //     at level 0, output exactly in [-1, 1]): aggressive folds.
  //   2 fuzz: biased (SoftClip(k*x+b) - SoftClip(b)) / (1 + SoftClip(b)) —
  //     bounded on BOTH sides, the positive swing shrinks as the bias grows
  //     (the classic asymmetric fuzz); Bias only acts in this mode.
  //   3 shape: TBD03's seeded waveshaper (braids SignatureWaveshaper math
  //     verbatim, seed 0xcafe = TBD03's curve; skew + sigmoid + bumplets =
  //     the braids "analog impurity" transfer) with Drive as pre-gain into
  //     the curve. Table lookup, ~10 cycles per sample.
  // No oversampling, one division per block + one per sample in the tanh
  // modes — a few us per block, safe on the A1S helper budget where the
  // Warps stage is not. Sits after the metering/panic block (the guard
  // keeps measuring the pre-drive resonator level) and every wet path is
  // bounded, so the reverb can never be overdriven by this stage.
  // drive 0 = zero-cost bypass.
  const int distType0 = static_cast<int>(patch_.dist_type + 0.5f);
  if (distType0 > 0 && patch_.dist_drive > 0.0f && patch_.dist_mix > 0.0f) {
    const float d = patch_.dist_drive;
    const float mix = patch_.dist_mix;
    const int type = (distType0 - 1) % 5;   // wrapper: 1 Tube .. 5 Chainsaw
    if (type == 4) {
      tbd::ChainsawStage(main, aux, size, dist_chainsaw_, d, patch_.dist_bias,
                         patch_.dist_tone, mix, patch_.dist_sustain,
                         dist_lp_, dist_env_);
    } else {
    const float lp_k = 0.04f + 0.96f * patch_.dist_tone;
    float k = 0.0f, k_norm = 1.0f;
    float bias = 0.0f, bias_sub = 0.0f, bias_norm = 1.0f;
    float ws_gain = 1.0f;
    if (type == 0) {
      k = 1.0f + 9.0f * d * d;
      k_norm = tbd::distfast::FastRecip(tbd::distfast::FastSoftClip(k));
    } else if (type == 2) {
      k = 1.0f + 9.0f * d * d;
      bias = patch_.dist_bias;
      const float s_b = tbd::distfast::FastSoftClip(bias);
      bias_sub = s_b;
      bias_norm = 1.0f / (1.0f + s_b);
    } else if (type == 3) {
      // TBD03 waveshaper: Drive pre-gains into the seeded curve (deeper
      // sigmoid knee + bumplets), the table lookup itself is ~10 cycles.
      ws_gain = 1.0f + 7.0f * d;
    }
    // Sustainer AGC: as the note decays the envelope falls and the gain
    // rises (up to 1 + 15*sustain^2, ~16x at full) holding the output near
    // a fixed 0.5 target — the audible tail extends until the resonator
    // decays below target/maxGain. Fast attack (~2 samples) so a new strike
    // ducks the gain instead of pumping; ~250 ms release shapes how quickly
    // the boost fades in. Envelope state decays toward 0 while the stage is
    // off so re-enabling starts clean.
    const float sus_max = 1.0f + 15.0f * patch_.dist_sustain * patch_.dist_sustain;
    const bool sustain_on = sus_max > 1.001f;
    float* chans[2] = { main, aux };
    for (size_t c = 0; c < 2; ++c) {
      float lp = dist_lp_[c];
      float env = dist_env_[c];
      if (!sustain_on) {
        env *= 0.995f;
      }
      for (size_t i = 0; i < size; ++i) {
        const float dry = chans[c][i];
        float wet;
        if (type == 1) {
          wet = Fold_do(dry, d);
        } else if (type == 2) {
          wet = (tbd::distfast::FastSoftClip(k * dry + bias) - bias_sub) * bias_norm;
        } else if (type == 3) {
          float xg = dry * ws_gain;
          if (xg > 1.0f) xg = 1.0f; else if (xg < -1.0f) xg = -1.0f;
          const uint16_t idx = static_cast<uint16_t>(
              static_cast<int16_t>(xg * 32767.0f) + 32768);
          const int32_t a = dist_ws_[idx >> 8];
          const int32_t b = dist_ws_[(idx >> 8) + 1];
          wet = static_cast<float>(a + ((b - a) * (idx & 0xff) >> 8)) *
              (1.0f / 32768.0f);
        } else {
          wet = tbd::distfast::FastSoftClip(k * dry) * k_norm;
        }
        if (sustain_on) {
          const float x = fabsf(wet);
          env += (x > env ? 0.5f : 0.000125f) * (x - env);
          float g = 0.5f * tbd::distfast::FastRecip(env > 0.002f ? env : 0.002f);
          if (g > sus_max) g = sus_max;
          wet *= g;
        }
        lp += lp_k * (wet - lp);
        chans[c][i] = dry + (lp - dry) * mix;
      }
      dist_lp_[c] = lp;
      dist_env_[c] = env;
    }
    }
  } else {
    dist_env_[0] *= 0.995f;
    dist_env_[1] *= 0.995f;
  }

  // Apply reverb.
  reverb_.set_amount(reverb_amount);
  reverb_.set_diffusion(patch_.reverb_diffusion);
  bool freeze = patch_.space >= 1.75f;
  if (freeze) {
    reverb_.set_time(1.0f);
    reverb_.set_input_gain(0.0f);
    reverb_.set_lp(1.0f);
  } else {
    reverb_.set_time(reverb_time);
    reverb_.set_input_gain(0.2f);
    reverb_.set_lp(patch_.reverb_lp);
  }
  reverb_.Process(main, aux, size);
}

}  // namespace elements
