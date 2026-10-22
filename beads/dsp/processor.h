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

#ifndef BEADS_DSP_PROCESSOR_H_
#define BEADS_DSP_PROCESSOR_H_

#include "stmlib/stmlib.h"

#include "beads/dsp/granular_processor.h"
#include "beads/dsp/modulated_parameter.h"
#include "beads/dsp/fx/reverb.h"
#include "beads/dsp/fx/wow_and_flutter.h"

#include <atomic>

// The codec runs at a fixed host sample rate (44.1kHz on TBD). The wet signal
// is downsampled by a 0.5 factor for the half-rate media (vintage digital and
// both tape modes) and low-pass filtering is applied to the output signal
// within the wow and flutter code. The reverb, the limiters and the wow and
// flutter simulation always run at the host sample rate.

namespace beads {

// (512 + 256) * 1000 bytes, 48kHz 16-bit stereo = 4 seconds
// (4.35 seconds at 44.1kHz; time and pitch are relative so this only makes the
// buffer slightly longer than on the original hardware.)
const size_t kDelayBufferSizeBits = 768 * 1000 * 8;

class Processor {
 public:
  Processor() { }
  ~Processor() { }
  
  enum Media {
    MEDIA_DIRTY_TAPE,
    MEDIA_CLEAN_TAPE,
    MEDIA_VINTAGE_DIGITAL,
    MEDIA_DIGITAL
  };
  
  struct MixParameters {
    float feedback;
    float dry_wet;
    float reverb_amount;
    bool output_gate;
  };

  struct GrainHelperBlock {
    TriggerGenerator::Output triggers[beads::kMaxBlockSize] = {};
    Parameters parameters = {};
    int write_ptr = 0;
    int core_active_grains = 0;
    size_t size = 0;
    bool mono = false;
    bool reset = false;
    bool valid = false;
  };

  // TBD always processes 32-sample blocks. All scratch buffers used by Process
  // are fixed-size members so that the audio task stack (4KB) is not used for
  // them. Must match beads::kMaxBlockSize (granular_processor.h).
  static constexpr size_t kMaxBlockSize = 32;
  // Reverb send mix (reverb^2 * 0.9) below this is inaudible; the network is
  // bypassed to reclaim its substantial CPU cost.
  static constexpr float kReverbActiveThreshold = 0.0001f;
  
  void Init(
      uint8_t* buffers[],
      uint16_t* reverb_buffer,
      float host_sample_rate);
  void Configure(Media media, bool mono);
  void SetMaxActiveGrains(int n);
  void SetHelperMaxActiveGrains(int n);
  void RenderHelperGrains(const GrainHelperBlock& block, FloatFrame* out);
  int HelperActiveGrains() const;
  static bool HelperOwnsTrigger(int counter, int total, int helper);
  // Number of grains currently rendering. Host-test and bring-up aid.
  inline int ActiveGrains() const {
    switch (media_) {
      case MEDIA_DIRTY_TAPE:
        return processor_dirty_tape_.ActiveGrains();
      case MEDIA_CLEAN_TAPE:
        return processor_clean_tape_.ActiveGrains();
      case MEDIA_VINTAGE_DIGITAL:
        return processor_vintage_digital_.ActiveGrains();
      case MEDIA_DIGITAL:
      default:
        return processor_digital_.ActiveGrains();
    }
  }
  // Delay mode (the 8-tap pitch-shifting multitap delay) is disabled in the
  // v1 plugin because it exceeds the TBD audio-task budget. The DSP core keeps
  // the implementation for tests and future optimization.
  void SetDelayModeEnabled(bool enabled);
  
  inline Media media() const { return media_; }
  inline bool mono() const { return mono_; }
  
  // Sample rate at which the granular/delay engine runs. Half of the host
  // sample rate for every media except MEDIA_DIGITAL.
  inline float sample_rate() const {
    return media_ == MEDIA_DIGITAL
        ? host_sample_rate_
        : host_sample_rate_ * 0.5f;
  }
  
  // Ratio between the host sample rate and the engine sample rate.
  inline int ratio() const {
    const int ratio[4] = { 2, 2, 2, 1 };
    return ratio[media_];
  }
  
  inline uint8_t* delay_buffer_raw_bytes(size_t offset) {
    // Independently of the quality settings, we're writing in the same
    // bytes anyway!
    return processor_vintage_digital_.delay_buffer_raw_bytes(mono_, offset);
  }
  
  inline int* mutable_delay_write_ptr() {
    switch (media_) {
      case MEDIA_DIRTY_TAPE:
        return processor_dirty_tape_.mutable_delay_write_ptr(mono_);
      case MEDIA_CLEAN_TAPE:
        return processor_clean_tape_.mutable_delay_write_ptr(mono_);
      case MEDIA_VINTAGE_DIGITAL:
        return processor_vintage_digital_.mutable_delay_write_ptr(mono_);
      case MEDIA_DIGITAL:
      default:
        return processor_digital_.mutable_delay_write_ptr(mono_);
    }
  }
  
  inline float overwrite_reserve() const {
    float reserve = float(remaining());
    // Sample rate is the engine rate, and remaining() is expressed in engine
    // samples, so the fade duration is 0.15s of engine time.
    float fade_duration = sample_rate() * 0.15f;
    return reserve >= fade_duration ? 1.0f : reserve / fade_duration;
  }
  
  inline int remaining() const {
    switch (media_) {
      case MEDIA_DIRTY_TAPE:
        return processor_dirty_tape_.remaining(mono_);
      case MEDIA_CLEAN_TAPE:
        return processor_clean_tape_.remaining(mono_);
      case MEDIA_VINTAGE_DIGITAL:
        return processor_vintage_digital_.remaining(mono_);
      case MEDIA_DIGITAL:
      default:
        return processor_digital_.remaining(mono_);
    }
  }

  inline void Trim() {
    switch (media_) {
      case MEDIA_DIRTY_TAPE:
        processor_dirty_tape_.Trim(mono_);
      case MEDIA_CLEAN_TAPE:
        processor_clean_tape_.Trim(mono_);
      case MEDIA_VINTAGE_DIGITAL:
        processor_vintage_digital_.Trim(mono_);
      case MEDIA_DIGITAL:
      default:
        processor_digital_.Trim(mono_);
    }
  }
  
  void WasteCPUCycles(size_t size) {
    // The one with the most convoluted memory access!
    processor_vintage_digital_.WasteCPUCycles(size);
  }

  void Process(
      SeedButtonState seed_button_state,
      const stmlib::GateFlags* seed_input,
      const bool* freeze,
      const ModulatedParameter& density,
      const Parameters& parameters,
      const MixParameters& mix_parameters,
      const FloatFrame* in,
      FloatFrame* out,
      size_t size,
      GrainHelperBlock* helper_block = nullptr,
      const FloatFrame* extra_wet = nullptr);
  
 private:
  GranularProcessor<
      RESOLUTION_16_BIT,
      kDelayBufferSizeBits / (16 * 2),
      1> processor_digital_;
  GranularProcessor<
      RESOLUTION_12_BIT,
      kDelayBufferSizeBits / (12 * 2),
      2> processor_vintage_digital_;
  GranularProcessor<
      RESOLUTION_12_BIT,
      kDelayBufferSizeBits / (12 * 2),
      2> processor_clean_tape_;
  GranularProcessor<
      RESOLUTION_8_BIT,
      kDelayBufferSizeBits / (8 * 2),
      2> processor_dirty_tape_;

  DelayGranularPlayer helper_player_;
  std::atomic<int> helper_active_grains_{0};
  std::atomic<int> helper_max_active_grains_{0};
  int core_max_active_grains_ = 0;
  int helper_split_counter_ = 0;
  bool helper_reset_pending_ = false;

  TriggerGenerator trigger_generator_;
  DelayController delay_controller_;
  
  Media media_;
  bool process_dry_;
  bool mono_;
  // True while the reverb network is being processed. When the mix is zero
  // the network is skipped; the flag makes the first enable clear stale tail
  // audio instead of resuming a frozen reverb from minutes ago.
  bool reverb_active_;
  bool delay_mode_enabled_;
  
  float host_sample_rate_;
  float delay_mode_transition_;

  float dry_;
  float wet_;
  
  WowAndFlutter wow_and_flutter_;
  Reverb reverb_;
  Limiter pre_reverb_limiter_;
  Limiter output_limiter_;
  
  // Fixed block scratch, previously stack-allocated VLAs.
  TriggerGenerator::Output triggers_buffer_[kMaxBlockSize];
  DelayController::Output taps_buffer_[kMaxBlockSize];
  bool gate_out_[kMaxBlockSize];
  FloatFrame dry_signal_[kMaxBlockSize];
  // Half-rate path scratch, shared by the Vintage/tape engines.
  HalfRateScratch half_rate_scratch_;
  
  DISALLOW_COPY_AND_ASSIGN(Processor);
};

// The class member sizes Processor's scratch buffers; the namespace constant
// sizes the half-rate scratch of GranularProcessor. They must not drift apart.
static_assert(
    Processor::kMaxBlockSize == kMaxBlockSize,
    "Processor::kMaxBlockSize and beads::kMaxBlockSize must match");

}  // namespace beads

#endif  // BEADS_DSP_PROCESSOR_H_
