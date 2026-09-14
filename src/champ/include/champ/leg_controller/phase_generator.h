/*
Copyright (c) 2019-2020, Juan Miguel Jimeno
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef PHASE_GENERATOR_H
#define PHASE_GENERATOR_H

#include <macros/macros.h>
#include <quadruped_base/quadruped_base.h>

#include <cmath>

namespace champ
{
    // Trot phase clock. LF/RH (legs 0, 3) share one clock, RF/LH (legs 1, 2)
    // run half a stride behind. Each leg spends stance_duration in stance and
    // then swing_phase_period in swing; the two periods need not be equal
    // (unequal periods give a short four-leg support window per stride).
    class PhaseGenerator
    {
        public:
            typedef unsigned long int Time;
            static inline Time now() { return time_us(); }

        private:
            champ::QuadrupedBase *base_;

            Time last_touchdown_;

        public:
            PhaseGenerator(champ::QuadrupedBase &base, Time time = now()):
                base_(&base),
                last_touchdown_(time),
                has_started(false),
                stance_phase_signal{0.0f,0.0f,0.0f,0.0f},
                swing_phase_signal{0.0f,0.0f,0.0f,0.0f}
            {
            }        

            void run(float target_velocity, float /*step_length*/, Time time = now())
            {
                const float swing_phase_period = 0.25f * SECONDS_TO_MICROS;
                const float stance_phase_period =  base_->gait_config.stance_duration * SECONDS_TO_MICROS;
                const float stride_period = stance_phase_period + swing_phase_period;

                if(target_velocity == 0.0f)
                {
                    has_started = false;
                    last_touchdown_ = 0;
                    for(unsigned int i = 0; i < 4; i++)
                    {
                        stance_phase_signal[i] = 0.0f;
                        swing_phase_signal[i] = 0.0f;  
                    }
                    return;
                }

                if(!has_started)
                {
                    has_started = true;
                    // Begin the stride at the instant RF/LH leave stance, so every
                    // foot is on the ground and the first swing rises from z = 0
                    // (a clock started at 0 would drop RF/LH into the middle of a
                    // swing, i.e. a full swing_height step in one tick). The stance
                    // feet then sit a fraction of a step length from neutral, which
                    // is small whenever the caller ramps the velocity up from zero.
                    const Time start_offset = static_cast<Time>(
                        fmodf(stance_phase_period + 0.5f * stride_period, stride_period));
                    last_touchdown_ = time - start_offset;
                }

                // Advance by whole strides only, so the start_offset remainder
                // is kept. Assigning last_touchdown_ = time would skip that
                // remainder on the first wrap (~25 ms of phase, a foot pop).
                const Time stride_period_t = static_cast<Time>(stride_period);
                if(stride_period_t > 0)
                {
                    const Time elapsed = time - last_touchdown_;
                    if(elapsed >= stride_period_t)
                    {
                        last_touchdown_ += (elapsed / stride_period_t) * stride_period_t;
                    }
                }
                const float elapsed_time_ref = static_cast<float>(time - last_touchdown_);

                const float leg_offsets[4] = {0.0f, 0.5f, 0.5f, 0.0f};
                for(int i = 0; i < 4; i++)
                {
                    // Every leg clock is wrapped into [0, stride): stance first,
                    // then swing. Letting the half-stride legs run negative instead
                    // would cut their stance short whenever stance_duration differs
                    // from the swing period (the foot then jumps into the swing).
                    float leg_clock = elapsed_time_ref - leg_offsets[i] * stride_period;
                    if(leg_clock < 0.0f)
                        leg_clock += stride_period;

                    if(leg_clock >= 0.0f && leg_clock < stance_phase_period)
                    {
                        stance_phase_signal[i] = leg_clock / stance_phase_period;
                        swing_phase_signal[i] = 0.0f;
                    }
                    else if(leg_clock >= stance_phase_period && leg_clock < stride_period)
                    {
                        stance_phase_signal[i] = 0.0f;
                        swing_phase_signal[i] = (leg_clock - stance_phase_period) / swing_phase_period;
                    }
                    else
                    {
                        stance_phase_signal[i] = 0.0f;
                        swing_phase_signal[i] = 0.0f;
                    }
                }
            }

            bool has_started;

            float stance_phase_signal[4];
            float swing_phase_signal[4];
    };
}

#endif