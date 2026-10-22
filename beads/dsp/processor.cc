// Copyright 2017 Emilie Gillet.
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
// The whole FX chain.

#include "beads/dsp/processor.h"

#include <algorithm>

#include "beads/dsp/mu_law.h"

namespace beads {

const float kDelayModeTransitionTime = 1.5f;  // seconds
const float kDelayModeFadeTime = 0.75f;  // seconds

using namespace std;
using namespace stmlib;

void Processor::Init(
    uint8_t* buffers[],
    uint16_t* reverb_buffer,
    float host_sample_rate) {
  host_sample_rate_ = host_sample_rate;

  // Media-specific engine rates. All the half-rate media run the granular
  // engine at half the host sample rate and upsample/downsample around it.
  processor_digital_.Init(host_sample_rate, buffers);
  processor_vintage_digital_.Init(host_sample_rate * 0.5f, buffers);
  processor_clean_tape_.Init(host_sample_rate * 0.5f, buffers);
  processor_dirty_tape_.Init(host_sample_rate * 0.5f, buffers);

  helper_player_.Init(host_sample_rate);
  helper_active_grains_.store(0);
  helper_max_active_grains_.store(0);
  core_max_active_grains_ = 0;
  helper_split_counter_ = 0;
  helper_reset_pending_ = false;
  
  dry_ = 0.0f;
  wet_ = 0.0f;
  reverb_active_ = false;
  delay_mode_enabled_ = false;
  
  // Reverb and limiters always run at the host sample rate.
  reverb_.Init(reverb_buffer, host_sample_rate);

  // Wow and flutter is applied to the host-rate output of the dirty tape mode.
  wow_and_flutter_.Init(host_sample_rate);
  wow_and_flutter_.set_lp(0.95f);

  Configure(MEDIA_DIGITAL, false);
  
  // Used for the timing of the delay to granular transition.
  delay_mode_transition_ = -kDelayModeTransitionTime;
}

void Processor::Configure(Media media, bool mono) {
  media_ = media;
  mono_ = mono;
  
  int max_delay = 0;

  // Clearing the delay line is the expensive part of a media switch: 768KB of
  // PSRAM. Only the delay line of the active channel layout is cleared, and
  // both layouts share the same buffers, so a single pass is enough.
  switch (media_) {
    case MEDIA_DIGITAL:
      processor_digital_.Reset(mono);
      max_delay = processor_digital_.max_delay_times_ratio();
      break;
    case MEDIA_VINTAGE_DIGITAL:
      processor_vintage_digital_.Reset(mono);
      max_delay = processor_vintage_digital_.max_delay_times_ratio();
      break;
    case MEDIA_CLEAN_TAPE:
      processor_clean_tape_.Reset(mono);
      max_delay = processor_clean_tape_.max_delay_times_ratio();
      break;
    case MEDIA_DIRTY_TAPE:
      processor_dirty_tape_.Reset(mono);
      max_delay = processor_dirty_tape_.max_delay_times_ratio();
      break;
  }
  
  pre_reverb_limiter_.Init(1.0f / (host_sample_rate_ * 4.0f), 0.9f);
  output_limiter_.Init(1.0f / (host_sample_rate_ * 6.0f), 0.9f);

  // The trigger generator and the delay controller are stepped once per host
  // sample: Process() renders them over the full 32-sample block, and the
  // half-rate path in GranularProcessor downsamples their output (triggers are
  // preserved across the ratio group, tap delays are divided by ratio). They
  // must therefore be initialized with the host sample rate, not the engine
  // rate, or their clock constants and LFO increments run at the wrong speed.
  trigger_generator_.Init(host_sample_rate_);
  delay_controller_.Init(host_sample_rate_, max_delay * (mono ? 2 : 1));
  reverb_.Reset(host_sample_rate_);
  helper_reset_pending_ = true;
}

void Processor::SetMaxActiveGrains(int n) {
  if (n < 0) n = 0;
  if (n > 32) n = 32;
  core_max_active_grains_ = n;
  // Apply to every media engine so the limit survives a media switch.
  processor_digital_.SetMaxActiveGrains(n);
  processor_vintage_digital_.SetMaxActiveGrains(n);
  processor_clean_tape_.SetMaxActiveGrains(n);
  processor_dirty_tape_.SetMaxActiveGrains(n);
}

void Processor::SetHelperMaxActiveGrains(int n) {
  if (n < 0) n = 0;
  if (n > 32) n = 32;
  helper_max_active_grains_.store(n);
}

bool Processor::HelperOwnsTrigger(int counter, int total, int helper) {
  return total > 0 && helper > 0 && (counter % total) < helper;
}

int Processor::HelperActiveGrains() const {
  return helper_active_grains_.load();
}

void Processor::SetDelayModeEnabled(bool enabled) {
  delay_mode_enabled_ = enabled;
  if (!enabled) {
    // Keep the granular wet path fully faded in. The transition is driven by
    // Process() only while delay mode is enabled.
    delay_mode_transition_ = -kDelayModeTransitionTime;
  }
}

void Processor::Process(
    SeedButtonState seed_button_state,
    const stmlib::GateFlags* seed_input,
    const bool* freeze,
    const ModulatedParameter& density,
    const Parameters& parameters,
    const MixParameters& mix_parameters,
    const FloatFrame* in,
    FloatFrame* out,
    size_t size,
    GrainHelperBlock* helper_block,
    const FloatFrame* extra_wet) {
  // TBD always passes 32 samples; never overflow the fixed scratch buffers.
  if (size > kMaxBlockSize) {
    size = kMaxBlockSize;
  }

  const bool delay_mode = delay_mode_enabled_ &&
      (delay_mode_transition_ > 0.0f);
  if (delay_mode_enabled_) {
    const float threshold = delay_mode ? 0.98f : 0.99f;
    const float direction = parameters.size.base > threshold ? 1.0f : -1.0f;
    delay_mode_transition_ += direction * float(size) / host_sample_rate_;
    CONSTRAIN(delay_mode_transition_,
              -kDelayModeTransitionTime,
              +kDelayModeTransitionTime);
  } else {
    // v1 runs the granular engine across the entire Size range. Keep the wet
    // path fully faded in; no multitap delay is rendered.
    delay_mode_transition_ = -kDelayModeTransitionTime;
  }
  
  float expo_fm_amount = 20.0f * (fabsf(density.base - 0.5f) - 0.225f);
  CONSTRAIN(expo_fm_amount, 0.0f, 1.0f);

  TriggerGenerator::Output* triggers = &triggers_buffer_[0];
  DelayController::Output* taps = &taps_buffer_[0];
  
  // Extract grain triggers.
  if (delay_mode) {
    triggers = NULL;
    delay_controller_.Render(
        seed_button_state,
        seed_input,
        freeze,
        density.base,
        (1.0f - expo_fm_amount) * density.cv,
        expo_fm_amount * density.cv,
        parameters.time,
        parameters.shape,
        taps,
        size);
    for (size_t i = 0; i < size; ++i) {
      gate_out_[i] = taps[i].gate;
    }
  } else {
    taps = NULL;
    trigger_generator_.Render(
        seed_button_state,
        seed_input,
        density.base,
        (1.0f - expo_fm_amount) * density.cv,
        expo_fm_amount * density.cv,
        triggers,
        size);
    for (size_t i = 0; i < size; ++i) {
      gate_out_[i] = triggers[i].gate;
    }
  }
  
  // Compute the dry signal *before* the wet processor runs, so that Process()
  // is safe even if a caller passes in == out.
  switch (media_) {
    case MEDIA_DIGITAL:
    case MEDIA_CLEAN_TAPE:
      for (size_t i = 0; i < size; ++i) {
        dry_signal_[i] = in[i];
      }
      break;
    
    case MEDIA_VINTAGE_DIGITAL:
      for (size_t i = 0; i < size; ++i) {
        dry_signal_[i].l =
            static_cast<int32_t>(in[i].l * 4096.0f) / 4096.0f;
        dry_signal_[i].r =
            static_cast<int32_t>(in[i].r * 4096.0f) / 4096.0f;
      }
      break;
    
    case MEDIA_DIRTY_TAPE:
      for (size_t i = 0; i < size; ++i) {
        dry_signal_[i].l = MuLaw2Lin(
            Lin2MuLaw(
                stmlib::Clip16(static_cast<int32_t>(in[i].l * 32768.0f))));
        dry_signal_[i].r = MuLaw2Lin(
            Lin2MuLaw(
                stmlib::Clip16(static_cast<int32_t>(in[i].r * 32768.0f))));
      }
      break;
  }

  // Fill the helper block (if any) before the core granular processor writes
  // the delay line. The snapshot is the write pointer at block start; the
  // helper reads only already-written samples.
  if (helper_block != nullptr) {
    helper_block->valid = (triggers != nullptr) && (media_ == MEDIA_DIGITAL);
    helper_block->size = size;
    helper_block->mono = mono_;
    helper_block->parameters = parameters;
    helper_block->core_active_grains = ActiveGrains();
    helper_block->write_ptr = helper_block->valid
        ? *mutable_delay_write_ptr()
        : 0;
    helper_block->reset = false;
    for (size_t i = 0; i < size; ++i) {
      helper_block->triggers[i] = triggers ? triggers[i]
                                          : TriggerGenerator::Output{};
      if (helper_block->triggers[i].trigger) {
        helper_block->triggers[i].trigger = false;
      }
    }
    if (helper_block->valid) {
      const int total = core_max_active_grains_ + helper_max_active_grains_.load();
      for (size_t i = 0; i < size; ++i) {
        if (triggers[i].trigger &&
            HelperOwnsTrigger(helper_split_counter_++, total,
                              helper_max_active_grains_.load())) {
          helper_block->triggers[i].trigger = true;
          triggers[i].trigger = false;
        }
      }
      if (helper_reset_pending_) {
        helper_block->reset = true;
        helper_reset_pending_ = false;
      }
    }
  }

  // External active-grain count for amplitude/pan normalisation.
  switch (media_) {
    case MEDIA_DIGITAL:
      processor_digital_.SetExternalActiveGrains(helper_active_grains_.load());
      break;
    default:
      processor_vintage_digital_.SetExternalActiveGrains(0);
      processor_clean_tape_.SetExternalActiveGrains(0);
      processor_dirty_tape_.SetExternalActiveGrains(0);
      break;
  }

  // Granular processing.
  const float feedback = mix_parameters.feedback;
  switch (media_) {
    case MEDIA_DIGITAL:
      processor_digital_.Process(
          triggers, taps, freeze, parameters, feedback, mono_, in, out, size,
          &half_rate_scratch_, extra_wet);
      break;
    
    case MEDIA_VINTAGE_DIGITAL:
      processor_vintage_digital_.Process(
          triggers, taps, freeze, parameters, feedback, mono_, in, out, size,
          &half_rate_scratch_);
      break;
    
    case MEDIA_CLEAN_TAPE:
      processor_clean_tape_.Process(
          triggers, taps, freeze, parameters, feedback, mono_, in, out,
          size, &half_rate_scratch_);
      break;
    
    case MEDIA_DIRTY_TAPE:
      processor_dirty_tape_.Process(
          triggers, taps, freeze, parameters, feedback, mono_, in, out,
          size, &half_rate_scratch_);
      break;
  }
  
  // Fade-out / Fade-in the dry signal when switching between granular and
  // Delay mode.
  const float wet_fade = SmoothStep(
      std::min(1.0f, fabsf(delay_mode_transition_) / kDelayModeFadeTime));
  
  // Dry/wet balance.
  const float post_gain_dry = 0.75f;
  const float post_gain_wet = 1.5f;
  const float dry_wet = mix_parameters.dry_wet;
  const float dry = (1.0f - dry_wet) * (1.0f + dry_wet) * post_gain_dry;
  const float wet = dry_wet * (2.0f - dry_wet) * post_gain_wet * wet_fade;

  stmlib::ParameterInterpolator dry_modulation(&dry_, dry, size);
  stmlib::ParameterInterpolator wet_modulation(&wet_, wet, size);

  for (size_t i = 0; i < size; ++i) {
    out[i] = dry_modulation.Next() * dry_signal_[i] + \
        wet_modulation.Next() * out[i];
  }
  
  // Wow and flutter simulation.
  if (media_ == MEDIA_DIRTY_TAPE) {
    wow_and_flutter_.Process(out, size);
  }
  
  pre_reverb_limiter_.Process(out, size);
  
  // Reverb.
  const bool caveman = media_ != MEDIA_DIGITAL;
  const float reverb = mix_parameters.reverb_amount;
  const float reverb_mix = reverb * reverb * 0.9f;
  const float reverb_time = reverb * reverb * reverb;
  reverb_.set_mix(reverb_mix);
  reverb_.set_time(caveman
      ? (0.45f + 0.5f * reverb_time)
      : (0.3f + 0.6f * reverb_time));
  reverb_.set_lp(
      caveman ? 0.4f : (media_ == MEDIA_VINTAGE_DIGITAL ? 0.75f : 0.6f));
  if (reverb_mix > kReverbActiveThreshold) {
    if (!reverb_active_) {
      // The network was skipped while muted; drop the frozen tail so that
      // raising the reverb does not replay audio from before the mute.
      reverb_.Clear();
      reverb_active_ = true;
    }
    reverb_.Process(out, size);
  } else {
    reverb_active_ = false;
  }
  
  // Final clipping/limiting
  output_limiter_.Process(out, size);
  
  if (mix_parameters.output_gate) {
    for (size_t i = 0; i < size; ++i) {
      out[i].l = (out[i].l + out[i].r) * 0.5f;
      out[i].r = gate_out_[i] ? -0.8f : 0.0f;
    }
  }
}

void Processor::RenderHelperGrains(const GrainHelperBlock& block, FloatFrame* out) {
  if (!block.valid || block.size == 0) {
    for (size_t i = 0; i < block.size; ++i) {
      out[i].l = 0.0f;
      out[i].r = 0.0f;
    }
    helper_active_grains_.store(0);
    return;
  }

  if (block.reset) {
    helper_player_.Reset();
    helper_player_.SetExternalActiveGrains(0);
  }
  helper_player_.SetMaxActiveGrains(helper_max_active_grains_.load());
  helper_player_.SetExternalActiveGrains(block.core_active_grains);

  const bool mono = block.mono;
  const size_t size = block.size;
  helper_player_.BeginBlock();
  for (size_t i = 0; i < size; ++i) {
    out[i].l = 0.0f;
    out[i].r = 0.0f;
  }
  if (mono) {
    auto dl = processor_digital_.mono_delay_line();
    for (size_t i = 0; i < size; ++i) {
      helper_player_.Render(dl, block.triggers[i], block.parameters, &out[i],
                            block.write_ptr);
    }
  } else {
    auto dl = processor_digital_.stereo_delay_line();
    for (size_t i = 0; i < size; ++i) {
      helper_player_.Render(dl, block.triggers[i], block.parameters, &out[i],
                            block.write_ptr);
    }
  }

  helper_active_grains_.store(helper_player_.CountActiveGrains());
}

}  // namespace beads