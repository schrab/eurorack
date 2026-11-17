// Copyright 2015 Emilie Gillet.
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

#include "rings/dsp/part.h"

#include "stmlib/dsp/dsp.h"
#include "stmlib/dsp/units.h"
#include "stmlib/utils/dsp.h"

#include "rings/resources.h"

// Retroactor's Vult-generated Befaco wavefolder (vult_formantor.cpp,
// compiled in the ctagSoundProcessor component) — the distortion stage's
// fold mode borrows it, same as the Elements stage.
extern float Fold_do(float signal, float level);

// braids' per-unit sine table (braids/resources.cc, same component build),
// via a namespace-scoped extern — including braids/resources.h would trip
// the documented LUT-macro collision class.
namespace braids { extern const int16_t wav_sine[]; }

namespace rings {

using namespace std;
using namespace stmlib;

// Type 3 "shape" transfer table (braids SignatureWaveshaper math verbatim,
// seed 0xcafe = TBD03's curve). FILE-STATIC, not a Part member: the TBDings
// wrapper fills the plugin arena exactly (object + blockMem), so the
// 1,028 B table must not grow the object. Deterministic seed -> one shared
// table for every Part instance.
int32_t s_dist_ws_[257];
bool s_dist_ws_built_ = false;

static void BuildDistShapeTable() {
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
    s_dist_ws_[i] = stmlib::Mix(sigmoid, bumplets, bumplet_gain);
  }
  s_dist_ws_[256] = s_dist_ws_[255];
  s_dist_ws_built_ = true;
}

void Part::Init(uint16_t* reverb_buffer) {
  if (!s_dist_ws_built_) {
    BuildDistShapeTable();
  }
  active_voice_ = 0;
  
  fill(&note_[0], &note_[kMaxPolyphony], 0.0f);
  
  bypass_ = false;
  polyphony_ = 1;
  model_ = RESONATOR_MODEL_MODAL;
  dirty_ = true;
  
  for (int32_t i = 0; i < kMaxPolyphony; ++i) {
    excitation_filter_[i].Init();
    plucker_[i].Init();
    dc_blocker_[i].Init(1.0f - 10.0f / kSampleRate);
  }
  
  reverb_.Init(reverb_buffer);
  limiter_.Init();
  dist_chainsaw_.Init(32000.0f);

  note_filter_.Init(
      kSampleRate / kMaxBlockSize,
      0.001f,  // Lag time with a sharp edge on the V/Oct input or trigger.
      0.010f,  // Lag time after the trigger has been received.
      0.050f,  // Time to transition from reactive to filtered.
      0.004f); // Prevent a sharp edge to partly leak on the previous voice.
}

void Part::ConfigureResonators() {
  if (!dirty_) {
    return;
  }
  
  switch (model_) {
    case RESONATOR_MODEL_MODAL:
      {
        int32_t resolution = 64 / polyphony_ - 4;
        for (int32_t i = 0; i < polyphony_; ++i) {
          resonator_[i].Init();
          resonator_[i].set_resolution(resolution);
        }
      }
      break;
    
    case RESONATOR_MODEL_SYMPATHETIC_STRING:
    case RESONATOR_MODEL_STRING:
    case RESONATOR_MODEL_SYMPATHETIC_STRING_QUANTIZED:
    case RESONATOR_MODEL_STRING_AND_REVERB:
      {
        float lfo_frequencies[kNumStrings] = {
          0.5f, 0.4f, 0.35f, 0.23f, 0.211f, 0.2f, 0.171f
        };
        for (int32_t i = 0; i < kNumStrings; ++i) {
          bool has_dispersion = model_ == RESONATOR_MODEL_STRING || \
              model_ == RESONATOR_MODEL_STRING_AND_REVERB;
          string_[i].Init(has_dispersion);

          float f_lfo = float(kMaxBlockSize) / float(kSampleRate);
          f_lfo *= lfo_frequencies[i];
          lfo_[i].Init<COSINE_OSCILLATOR_APPROXIMATE>(f_lfo);
        }
        for (int32_t i = 0; i < polyphony_; ++i) {
          plucker_[i].Init();
        }
      }
      break;
    
    case RESONATOR_MODEL_FM_VOICE:
      {
        for (int32_t i = 0; i < polyphony_; ++i) {
          fm_voice_[i].Init();
        }
      }
      break;
    
    default:
      break;
  }

  if (active_voice_ >= polyphony_) {
    active_voice_ = 0;
  }
  dirty_ = false;
}

#ifdef BRYAN_CHORDS

// Chord table by Bryan Noll:
const float chords[kMaxPolyphony][11][8] = {
  {
    { -12.0f, -0.01f, 0.0f,  0.01f, 0.02f, 11.98f, 11.99f, 12.0f }, // OCT
    { -12.0f, -5.0f,  0.0f,  6.99f, 7.0f,  11.99f, 12.0f,  19.0f }, // 5
    { -12.0f, -5.0f,  0.0f,  5.0f,  7.0f,  11.99f, 12.0f,  17.0f }, // sus4
    { -12.0f, -5.0f,  0.0f,  3.0f,  7.0f,   3.01f, 12.0f,  19.0f }, // m 
    { -12.0f, -5.0f,  0.0f,  3.0f,  7.0f,   3.01f, 10.0f,  19.0f }, // m7
    { -12.0f, -5.0f,  0.0f,  3.0f, 14.0f,   3.01f, 10.0f,  19.0f }, // m9
    { -12.0f, -5.0f,  0.0f,  3.0f,  7.0f,   3.01f, 10.0f,  17.0f }, // m11
    { -12.0f, -5.0f,  0.0f,  2.0f,  7.0f,   9.0f,  16.0f,  19.0f }, // 69
    { -12.0f, -5.0f,  0.0f,  4.0f,  7.0f,  11.0f,  14.0f,  19.0f }, // M9
    { -12.0f, -5.0f,  0.0f,  4.0f,  7.0f,  11.0f,  10.99f, 19.0f }, // M7
    { -12.0f, -5.0f,  0.0f,  4.0f,  7.0f,  11.99f, 12.0f,  19.0f } // M
  },
  { 
    { -12.0f, 0.0f,  0.01f, 12.0f }, // OCT
    { -12.0f, 6.99f, 7.0f,  12.0f }, // 5
    { -12.0f, 5.0f,  7.0f,  12.0f }, // sus4
    { -12.0f, 3.0f, 11.99f, 12.0f }, // m 
    { -12.0f, 3.0f, 10.0f,  12.0f }, // m7
    { -12.0f, 3.0f, 10.0f,  14.0f }, // m9
    { -12.0f, 3.0f, 10.0f,  17.0f }, // m11
    { -12.0f, 2.0f,  9.0f,  16.0f }, // 69
    { -12.0f, 4.0f, 11.0f,  14.0f }, // M9
    { -12.0f, 4.0f,  7.0f,  11.0f }, // M7
    { -12.0f, 4.0f,  7.0f,  12.0f }, // M
  },
  {
    { 0.0f, -12.0f },
    { 0.0f, 2.0f },
    { 0.0f, 3.0f },
    { 0.0f, 4.0f },
    { 0.0f, 5.0f },
    { 0.0f, 7.0f },
    { 0.0f, 9.0f },
    { 0.0f, 10.0f },
    { 0.0f, 11.0f },
    { 0.0f, 12.0f },
    { -12.0f, 12.0f }
  },
  {
    { 0.0f, -12.0f },
    { 0.0f, 2.0f },
    { 0.0f, 3.0f },
    { 0.0f, 4.0f },
    { 0.0f, 5.0f },
    { 0.0f, 7.0f },
    { 0.0f, 9.0f },
    { 0.0f, 10.0f },
    { 0.0f, 11.0f },
    { 0.0f, 12.0f },
    { -12.0f, 12.0f }
  }
};

#else

// Original chord table
const float chords[kMaxPolyphony][11][8] = {
  {
    { -12.0f, 0.0f, 0.01f, 0.02f, 0.03f, 11.98f, 11.99f, 12.0f },
    { -12.0f, 0.0f, 3.0f,  3.01f, 7.0f,  9.99f,  10.0f,  19.0f },
    { -12.0f, 0.0f, 3.0f,  3.01f, 7.0f,  11.99f, 12.0f,  19.0f },
    { -12.0f, 0.0f, 3.0f,  3.01f, 7.0f,  13.99f, 14.0f,  19.0f },
    { -12.0f, 0.0f, 3.0f,  3.01f, 7.0f,  16.99f, 17.0f,  19.0f },
    { -12.0f, 0.0f, 6.98f, 6.99f, 7.0f,  12.00f, 18.99f, 19.0f },
    { -12.0f, 0.0f, 3.99f, 4.0f,  7.0f,  16.99f, 17.0f,  19.0f },
    { -12.0f, 0.0f, 3.99f, 4.0f,  7.0f,  13.99f, 14.0f,  19.0f },
    { -12.0f, 0.0f, 3.99f, 4.0f,  7.0f,  11.99f, 12.0f,  19.0f },
    { -12.0f, 0.0f, 3.99f, 4.0f,  7.0f,  10.99f, 11.0f,  19.0f },
    { -12.0f, 0.0f, 4.99f, 5.0f,  7.0f,  11.99f, 12.0f,  17.0f }
  },
  { 
    { -12.0f, 0.0f, 0.01f, 12.0f },
    { -12.0f, 3.0f, 7.0f,  10.0f },
    { -12.0f, 3.0f, 7.0f,  12.0f },
    { -12.0f, 3.0f, 7.0f,  14.0f },
    { -12.0f, 3.0f, 7.0f,  17.0f },
    { -12.0f, 7.0f, 12.0f, 19.0f },
    { -12.0f, 4.0f, 7.0f,  17.0f },
    { -12.0f, 4.0f, 7.0f,  14.0f },
    { -12.0f, 4.0f, 7.0f,  12.0f },
    { -12.0f, 4.0f, 7.0f,  11.0f },
    { -12.0f, 5.0f, 7.0f,  12.0f },
  },
  {
    { 0.0f, -12.0f },
    { 0.0f, 0.01f },
    { 0.0f, 2.0f },
    { 0.0f, 3.0f },
    { 0.0f, 4.0f },
    { 0.0f, 5.0f },
    { 0.0f, 7.0f },
    { 0.0f, 10.0f },
    { 0.0f, 11.0f },
    { 0.0f, 12.0f },
    { -12.0f, 12.0f }
  },
  {
    { 0.0f, -12.0f },
    { 0.0f, 0.01f },
    { 0.0f, 2.0f },
    { 0.0f, 3.0f },
    { 0.0f, 4.0f },
    { 0.0f, 5.0f },
    { 0.0f, 7.0f },
    { 0.0f, 10.0f },
    { 0.0f, 11.0f },
    { 0.0f, 12.0f },
    { -12.0f, 12.0f }
  }
};

#endif  // BRYAN_CHORDS

void Part::ComputeSympatheticStringsNotes(
    float tonic,
    float note,
    float parameter,
    float* destination,
    size_t num_strings) {
  float notes[9] = {
      tonic,
      note - 12.0f,
      note - 7.01955f,
      note,
      note + 7.01955f,
      note + 12.0f,
      note + 19.01955f,
      note + 24.0f,
      note + 24.0f };
  const float detunings[4] = {
      0.013f,
      0.011f,
      0.007f,
      0.017f
  };
  
  if (parameter >= 2.0f) {
    // Quantized chords
    int32_t chord_index = parameter - 2.0f;
    const float* chord = chords[polyphony_ - 1][chord_index];
    for (size_t i = 0; i < num_strings; ++i) {
      destination[i] = chord[i] + note;
    }
    return;
  }

  size_t num_detuned_strings = (num_strings - 1) >> 1;
  size_t first_detuned_string = num_strings - num_detuned_strings;
  
  for (size_t i = 0; i < first_detuned_string; ++i) {
    float note = 3.0f;
    if (i != 0) {
      note = parameter * 7.0f;
      parameter += (1.0f - parameter) * 0.2f;
    }
    
    MAKE_INTEGRAL_FRACTIONAL(note);
    note_fractional = Squash(note_fractional);

    float a = notes[note_integral];
    float b = notes[note_integral + 1];
    
    note = a + (b - a) * note_fractional;
    destination[i] = note;
    if (i + first_detuned_string < num_strings) {
      destination[i + first_detuned_string] = destination[i] + detunings[i & 3];
    }
  }
}

void Part::RenderModalVoice(
    int32_t voice,
    const PerformanceState& performance_state,
    const Patch& patch,
    float frequency,
    float filter_cutoff,
    size_t size) {
  // Internal exciter is a pulse, pre-filter.
  if (performance_state.internal_exciter &&
      voice == active_voice_ &&
      performance_state.strum) {
    resonator_input_[0] += 0.25f * SemitonesToRatio(
        filter_cutoff * filter_cutoff * 24.0f) / filter_cutoff;
  }
  
  // Process through filter.
  excitation_filter_[voice].Process<FILTER_MODE_LOW_PASS>(
      resonator_input_, resonator_input_, size);

  Resonator& r = resonator_[voice];
  r.set_frequency(frequency);
  r.set_structure(patch.structure);
  r.set_brightness(patch.brightness * patch.brightness);
  r.set_position(patch.position);
  r.set_damping(patch.damping);
  r.Process(resonator_input_, out_buffer_, aux_buffer_, size);
}

void Part::RenderFMVoice(
    int32_t voice,
    const PerformanceState& performance_state,
    const Patch& patch,
    float frequency,
    float filter_cutoff,
    size_t size) {
  FMVoice& v = fm_voice_[voice];
  if (performance_state.internal_exciter &&
      voice == active_voice_ &&
      performance_state.strum) {
    v.TriggerInternalEnvelope();
  }

  v.set_frequency(frequency);
  v.set_ratio(patch.structure);
  v.set_brightness(patch.brightness);
  v.set_feedback_amount(patch.position);
  v.set_position(/*patch.position*/ 0.0f);
  v.set_damping(patch.damping);
  v.Process(resonator_input_, out_buffer_, aux_buffer_, size);
}

void Part::RenderStringVoice(
    int32_t voice,
    const PerformanceState& performance_state,
    const Patch& patch,
    float frequency,
    float filter_cutoff,
    size_t size) {
  // Compute number of strings and frequency.
  int32_t num_strings = 1;
  float frequencies[kNumStrings];

  if (model_ == RESONATOR_MODEL_SYMPATHETIC_STRING ||
      model_ == RESONATOR_MODEL_SYMPATHETIC_STRING_QUANTIZED) {
    num_strings = 2 * kMaxPolyphony / polyphony_;
    float parameter = model_ == RESONATOR_MODEL_SYMPATHETIC_STRING
        ? patch.structure
        : 2.0f + performance_state.chord;
    ComputeSympatheticStringsNotes(
        performance_state.tonic + performance_state.fm,
        performance_state.tonic + note_[voice] + performance_state.fm,
        parameter,
        frequencies,
        num_strings);
    for (int32_t i = 0; i < num_strings; ++i) {
      frequencies[i] = SemitonesToRatio(frequencies[i] - 69.0f) * a3;
    }
  } else {
    frequencies[0] = frequency;
  }

  if (voice == active_voice_) {
    const float gain = 1.0f / Sqrt(static_cast<float>(num_strings) * 2.0f);
    for (size_t i = 0; i < size; ++i) {
      resonator_input_[i] *= gain;
    }
  }

  // Process external input.
  excitation_filter_[voice].Process<FILTER_MODE_LOW_PASS>(
      resonator_input_, resonator_input_, size);

  // Add noise burst.
  if (performance_state.internal_exciter) {
    if (voice == active_voice_ && performance_state.strum) {
      plucker_[voice].Trigger(frequency, filter_cutoff * 8.0f, patch.position);
    }
    plucker_[voice].Process(noise_burst_buffer_, size);
    for (size_t i = 0; i < size; ++i) {
      resonator_input_[i] += noise_burst_buffer_[i];
    }
  }
  dc_blocker_[voice].Process(resonator_input_, size);
  
  fill(&out_buffer_[0], &out_buffer_[size], 0.0f);
  fill(&aux_buffer_[0], &aux_buffer_[size], 0.0f);
  
  float structure = patch.structure;
  float dispersion = structure < 0.24f
      ? (structure - 0.24f) * 4.166f
      : (structure > 0.26f ? (structure - 0.26f) * 1.35135f : 0.0f);
  
  for (int32_t string = 0; string < num_strings; ++string) {
    int32_t i = voice + string * polyphony_;
    String& s = string_[i];
    float lfo_value = lfo_[i].Next();
    
    float brightness = patch.brightness;
    float damping = patch.damping;
    float position = patch.position;
    float glide = 1.0f;
    float string_index = static_cast<float>(string) / static_cast<float>(num_strings);
    const float* input = resonator_input_;
    
    if (model_ == RESONATOR_MODEL_STRING_AND_REVERB) {
      damping *= (2.0f - damping);
    }
    
    // When the internal exciter is used, string 0 is the main
    // source, the other strings are vibrating by sympathetic resonance.
    // When the internal exciter is not used, all strings are vibrating
    // by sympathetic resonance.
    if (string > 0 && performance_state.internal_exciter) {
      brightness *= (2.0f - brightness);
      brightness *= (2.0f - brightness);
      damping = 0.7f + patch.damping * 0.27f;
      float amount = (0.5f - fabs(0.5f - patch.position)) * 0.9f;
      position = patch.position + lfo_value * amount;
      glide = SemitonesToRatio((brightness - 1.0f) * 36.0f);
      input = sympathetic_resonator_input_;
    }
    
    s.set_dispersion(dispersion);
    s.set_frequency(frequencies[string], glide);
    s.set_brightness(brightness);
    s.set_position(position);
    s.set_damping(damping + string_index * (0.95f - damping));
    s.Process(input, out_buffer_, aux_buffer_, size);
    
    if (string == 0) {
      // Was 0.1f, Ben Wilson -> 0.2f
      float gain = 0.2f / static_cast<float>(num_strings);
      for (size_t i = 0; i < size; ++i) {
        float sum = out_buffer_[i] - aux_buffer_[i];
        sympathetic_resonator_input_[i] = gain * sum;
      }
    }
  }
}

const int32_t kPingPattern[] = {
  1, 0, 2, 1, 0, 2, 1, 0
};

void Part::Process(
    const PerformanceState& performance_state,
    const Patch& patch,
    const float* in,
    float* out,
    float* aux,
    size_t size) {

  // Copy inputs to outputs when bypass mode is enabled.
  if (bypass_) {
    copy(&in[0], &in[size], &out[0]);
    copy(&in[0], &in[size], &aux[0]);
    return;
  }
  
  ConfigureResonators();
  
  note_filter_.Process(
      performance_state.note,
      performance_state.strum);

  if (performance_state.strum) {
    note_[active_voice_] = note_filter_.stable_note();
    if (polyphony_ > 1 && polyphony_ & 1) {
      active_voice_ = kPingPattern[step_counter_ % 8];
      step_counter_ = (step_counter_ + 1) % 8;
    } else {
      active_voice_ = (active_voice_ + 1) % polyphony_;
    }
  }
  
  note_[active_voice_] = note_filter_.note();
  
  fill(&out[0], &out[size], 0.0f);
  fill(&aux[0], &aux[size], 0.0f);
  for (int32_t voice = 0; voice < polyphony_; ++voice) {
    // Compute MIDI note value, frequency, and cutoff frequency for excitation
    // filter.
    float cutoff = patch.brightness * (2.0f - patch.brightness);
    float note = note_[voice] + performance_state.tonic + performance_state.fm;
    float frequency = SemitonesToRatio(note - 69.0f) * a3;
    float filter_cutoff_range = performance_state.internal_exciter
      ? frequency * SemitonesToRatio((cutoff - 0.5f) * 96.0f)
      : 0.4f * SemitonesToRatio((cutoff - 1.0f) * 108.0f);
    float filter_cutoff = min(voice == active_voice_
      ? filter_cutoff_range
      : (10.0f / kSampleRate), 0.499f);
    float filter_q = performance_state.internal_exciter ? 1.5f : 0.8f;

    // Process input with excitation filter. Inactive voices receive silence.
    excitation_filter_[voice].set_f_q<FREQUENCY_DIRTY>(filter_cutoff, filter_q);
    if (voice == active_voice_) {
      copy(&in[0], &in[size], &resonator_input_[0]);
    } else {
      fill(&resonator_input_[0], &resonator_input_[size], 0.0f);
    }
    
    if (model_ == RESONATOR_MODEL_MODAL) {
      RenderModalVoice(
          voice, performance_state, patch, frequency, filter_cutoff, size);
    } else if (model_ == RESONATOR_MODEL_FM_VOICE) {
      RenderFMVoice(
          voice, performance_state, patch, frequency, filter_cutoff, size);
    } else {
      RenderStringVoice(
          voice, performance_state, patch, frequency, filter_cutoff, size);
    }
    
    if (polyphony_ == 1) {
      // Send the two sets of harmonics / pickups to individual outputs.
      for (size_t i = 0; i < size; ++i) {
        out[i] += out_buffer_[i];
        aux[i] += aux_buffer_[i];
      }
    } else {
      // Dispatch odd/even voices to individual outputs.
      float* destination = voice & 1 ? aux : out;
      for (size_t i = 0; i < size; ++i) {
        destination[i] += out_buffer_[i] - aux_buffer_[i];
      }
    }
  }

  // Simple stereo distortion stage (TBD) — the same four characters as the
  // Elements stage (see elements/dsp/part.cc for the full curves), applied
  // to the resonator pair before the STRING_AND_REVERB model's reverb:
  //   0 tube: SoftClip(k*x)/SoftClip(k) — punch-preserving glue.
  //   1 fold: Fold_do(x, d) — Retroactor's Vult/Befaco wavefolder.
  //   2 fuzz: biased (SoftClip(k*x+b) - SoftClip(b)) / (1 + SoftClip(b)).
  //   3 shape: TBD03's seeded braids waveshaper (s_dist_ws_, Drive as
  //     pre-gain into the curve).
  // Then a 1-pole tone LP and a dry/wet blend, same curve on both
  // channels. drive 0 = zero-cost bypass. Renders on the audio core here
  // (TBDings has no helper) — a few us per block.
  if (patch.dist_drive > 0.0f) {
    const float d = patch.dist_drive;
    const float mix = patch.dist_mix;
    const int type = static_cast<int>(patch.dist_type + 0.5f) % 5;
    if (type == 4) {
      tbd::ChainsawStage(out, aux, size, dist_chainsaw_, d, patch.dist_bias,
                         patch.dist_tone, mix, patch.dist_sustain,
                         dist_lp_, dist_env_);
    } else {
    const float lp_k = 0.04f + 0.96f * patch.dist_tone;
    float k = 0.0f, k_norm = 1.0f;
    float bias = 0.0f, bias_sub = 0.0f, bias_norm = 1.0f;
    float ws_gain = 1.0f;
    if (type == 0) {
      k = 1.0f + 9.0f * d * d;
      k_norm = 1.0f / SoftClip(k);
    } else if (type == 2) {
      k = 1.0f + 9.0f * d * d;
      bias = patch.dist_bias;
      const float s_b = SoftClip(bias);
      bias_sub = s_b;
      bias_norm = 1.0f / (1.0f + s_b);
    } else if (type == 3) {
      ws_gain = 1.0f + 7.0f * d;
    }
    // Sustainer AGC — identical to the Elements stage: the gain rises as
    // the note decays (up to 1 + 15*sustain^2) holding the output near a
    // fixed 0.5 target; fast attack ducks new strikes, ~250 ms release.
    const float sus_max = 1.0f + 15.0f * patch.dist_sustain * patch.dist_sustain;
    const bool sustain_on = sus_max > 1.001f;
    float* chans[2] = { out, aux };
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
          wet = (SoftClip(k * dry + bias) - bias_sub) * bias_norm;
        } else if (type == 3) {
          float xg = dry * ws_gain;
          if (xg > 1.0f) xg = 1.0f; else if (xg < -1.0f) xg = -1.0f;
          const uint16_t idx = static_cast<uint16_t>(
              static_cast<int16_t>(xg * 32767.0f) + 32768);
          const int32_t a = s_dist_ws_[idx >> 8];
          const int32_t b = s_dist_ws_[(idx >> 8) + 1];
          wet = static_cast<float>(a + ((b - a) * (idx & 0xff) >> 8)) *
              (1.0f / 32768.0f);
        } else {
          wet = SoftClip(k * dry) * k_norm;
        }
        if (sustain_on) {
          const float x = fabsf(wet);
          env += (x > env ? 0.5f : 0.000125f) * (x - env);
          float g = 0.5f / (env > 0.002f ? env : 0.002f);
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

  if (model_ == RESONATOR_MODEL_STRING_AND_REVERB) {
    for (size_t i = 0; i < size; ++i) {
      float l = out[i];
      float r = aux[i];
      out[i] = l * patch.position + (1.0f - patch.position) * r;
      aux[i] = r * patch.position + (1.0f - patch.position) * l;
    }
    reverb_.set_amount(0.1f + patch.damping * 0.5f);
    reverb_.set_diffusion(0.625f);
    reverb_.set_time(0.35f + 0.63f * patch.damping);
    reverb_.set_input_gain(0.2f);
    reverb_.set_lp(0.3f + patch.brightness * 0.6f);
    reverb_.Process(out, aux, size);
    for (size_t i = 0; i < size; ++i) {
      aux[i] = -aux[i];
    }
  }
  
  // Apply limiter to string output.
  limiter_.Process(out, aux, size, model_gains_[model_]);
}

/* static */
const float Part::model_gains_[] = {
  1.4f,  // RESONATOR_MODEL_MODAL
  1.0f,  // RESONATOR_MODEL_SYMPATHETIC_STRING
  1.4f,  // RESONATOR_MODEL_STRING
  0.7f,  // RESONATOR_MODEL_FM_VOICE,
  1.0f,  // RESONATOR_MODEL_SYMPATHETIC_STRING_QUANTIZED
  1.4f,  // RESONATOR_MODEL_STRING_AND_REVERB
};

}  // namespace rings