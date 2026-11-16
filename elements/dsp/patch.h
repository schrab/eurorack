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
// Patch parameters.

#ifndef ELEMENTS_DSP_PATCH_H_
#define ELEMENTS_DSP_PATCH_H_

namespace elements {

struct Patch {
  float exciter_envelope_shape;
  float exciter_bow_level;
  float exciter_bow_timbre;
  float exciter_blow_level;
  float exciter_blow_meta;
  float exciter_blow_timbre;
  float exciter_strike_level;
  float exciter_strike_meta;
  float exciter_strike_timbre;
  float exciter_signature;
  float resonator_geometry;
  float resonator_brightness;
  float resonator_damping;
  float resonator_position;
  float resonator_modulation_frequency;
  float resonator_modulation_offset;
  float reverb_diffusion;
  float reverb_lp;
  float space;
  
  float modulation_frequency;

  // TBD: internal Warps cross-modulation stage (carrier = resonator mono
  // sum, modulator = raw exciter). Algorithm/timbre/mix are 0..1, the two
  // levels map onto the engine's channel_drive gain pots (0..1). All zero
  // cost while warps_algorithm == 0.
  float warps_algorithm;
  float warps_timbre;
  float warps_mod_level;
  float warps_carrier_level;
  float warps_mix;

  // TBD: simple stereo distortion stage, four characters selected by
  // dist_type (0 = tube: SoftClip knee glue, 1 = fold: Befaco/Vult
  // wavefolder, 2 = fuzz: biased asymmetric SoftClip, 3 = shape: TBD03's
  // seeded braids waveshaper). All 0..1; zero cost while dist_drive == 0.
  // Cheap enough for the A1S helper budget where the Warps stage is not.
  float dist_type;
  float dist_drive;
  float dist_bias;
  float dist_tone;
  float dist_mix;
};

}  // namespace elements

#endif  // ELEMENTS_DSP_PATCH_H_
