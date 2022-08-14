/*
 * Copyright 1996-2022 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**********************************************************************************/
/* Description:  Webots C programming interface for the Thruster node             */
/**********************************************************************************/

#ifndef WB_THRUSTER_H
#define WB_THRUSTER_H

#define WB_USING_C_API
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Get max thrust in N, as specified in the 'maxThrust' field of the node
double wb_thruster_get_max_thrust(WbDeviceTag tag);
// the throttle is how effectively we can change the thrust, a number between [0 .. 1.0]
double wb_thruster_get_throttle(WbDeviceTag tag);
// Get the specific impulse (returns NAN if no fuel tanks are defined)
double wb_thruster_get_ISR(WbDeviceTag tag);


// Enable recalc of the rocket mass with a refresh period (like wb_*_enable() on sensors)
void wb_thruster_enable_mass_info(WbDeviceTag tag, int sampling_period);
// Get the mass of the enire merged Solid the thruster is connected to. 
double wb_thruster_get_rocket_mass(WbDeviceTag tag);

// How much fuel, in kg, is left in all fuel tanks connected to the thruster. Returns -1 is no tanks are connected (= infinite fuel)
double wb_thruster_get_fuel(WbDeviceTag tag);
// Change the amount of fuel to 'level' of the total capacity tank, where level is between [0 .. 1.0]
void   wb_thruster_refuel(WbDeviceTag tag, double level);     

// void wb_thruster_set_max_thrust(WbDeviceTag tag, double max_thrust);
// void wb_thruster_set_throttle(WbDeviceTag tag, double throttle);

// Fire the rocket! Duration need not be equal to the sim step, the thruster may "burn" through several sim steps, on the last step
// the applied force will be smaller.
void wb_thruster_start_burn(WbDeviceTag tag, float throttle, int duration);
// Returns the time remaining till the end of the current burn. X > 0 means it is still burning, X = 0 - finished, X < 0 - went out of fuel X seconds before the end of the burn
int wb_thruster_burn_state(WbDeviceTag tag);


#ifdef __cplusplus
}
#endif

#endif /* WB_THRUSTER_H */
