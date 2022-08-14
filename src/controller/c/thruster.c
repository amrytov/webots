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

// ***************************************************
//  this file contains the API code for the LED device
// ***************************************************

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/thruster.h>
#include <webots/nodes.h>
#include <webots/robot.h>
#include "device_private.h"
#include "messages.h"
#include "robot_private.h"

// Static functions

typedef struct {
    int     req_flags;
    float   maxThrust;
    float   maxThrottle;
    float   currThrottle;
    float   fuelMass;
    float   rocketMass;
    float   refuelRate;
    float   myISR;
    int     burnState;
    int     massSampling;
} 
    Thruster;

static Thruster *thruster_create() {
    Thruster *thr = malloc(sizeof(Thruster));
    thr->req_flags = 0;
    thr->maxThrust = NAN;
    thr->maxThrottle = 1.0;
    thr->currThrottle = 1.0;
    thr->fuelMass = -1;
    thr->rocketMass = NAN;
    thr->burnState = 0;
    thr->massSampling = 0;
    thr->myISR = NAN;
    thr->refuelRate = NAN;
    return thr;
}

static Thruster *thruster_get_struct(WbDeviceTag t, const char * from_func) {
    WbDevice *d = robot_get_device_with_node(t, WB_NODE_THRUSTER, true);
    if( d ) return d;
    fprintf(stderr, "Error: %s(): invalid device tag.\n", from_func);
    return NULL;
}

static void thruster_write_request(WbDevice *d, WbRequest *r) {
    Thruster *thruster = d->pdata;

    if (thruster->req_flags & C_THRUSTER_START_BURN) {
        request_write_uchar(r, C_THRUSTER_START_BURN);
        request_write_int32(r, thruster->burnState);
        request_write_double(r, thruster->currThrottle);
    }
    if (thruster->req_flags & C_THRUSTER_ENABLE_MASS_CALC) {
        request_write_uchar(r, C_THRUSTER_ENABLE_MASS_CALC);
        request_write_int32(r, thruster->massSampling);
    }
    if (thruster->req_flags & C_THRUSTER_SET_FUEL) {
        request_write_uchar(r, C_THRUSTER_SET_FUEL);
        request_write_double(r, thruster->refuelRate);
    }
    /*
    // Cheat functions, no need actually
    if (thruster->req_flags & C_THRUSTER_SET_MAX_THRUST) {
        request_write_uchar(r, C_THRUSTER_SET_MAX_THRUST);
        request_write_double(r, thruster->maxThrust);
    }
    if (thruster->req_flags & C_THRUSTER_SET_THROTTLE) {
        request_write_uchar(r, C_THRUSTER_SET_THROTTLE);
        request_write_double(r, thruster->maxThrottle);
    }
    */
    thruster->req_flags = 0;
}

static void thruster_read_answer(WbDevice *d, WbRequest *r) {
    Thruster *thruster = (Thruster *)d->pdata;
    switch (request_read_uchar(r)) {
        case C_THRUSTER_GET_MASS:
            thruster->fuelMass = request_read_double(r);
            thruster->rocketMass = request_read_double(r);
            break;
        case C_CONFIGURE:
            thruster->burnState = request_read_int32(r);
            thruster->currThrottle = request_read_double(r);
            thruster->maxThrottle = request_read_double(r);
            thruster->maxThrust = request_read_double(r);
            thruster->myISR     = request_read_double(r);
            break;
        default:
            ROBOT_ASSERT(0);  // should never be reached
            break;
    }
}

static void thruster_cleanup(WbDevice *d) {
    free(d->pdata);
}

static void thruster_toggle_remote(WbDevice *d, WbRequest *r) {
    Thruster *thruster = d->pdata;
    // ???  Figure out what should be here!!!
}

// Exported functions

void wb_thruster_init(WbDevice *d) {
  d->pdata = distance_sensor_create();
  d->write_request = thruster_write_request;
  d->read_answer = thruster_read_answer;
  d->cleanup = thruster_cleanup;
  d->toggle_remote = thruster_toggle_remote;
}

// Public functions (available from the user API)

// Get max thrust in N, as specified in the 'maxThrust' field of the node
double wb_thruster_get_max_thrust(WbDeviceTag tag)
{
    robot_mutex_lock();
    Thruster *t = thruster_get_struct(tag, __FUNCTION__);
    robot_mutex_unlock();
    return t ? t->maxThrust : NAN;
}

// the throttle is how effectively we can change the thrust, a number between [0 .. 1.0]
double wb_thruster_get_throttle(WbDeviceTag tag)
{
    robot_mutex_lock();
    Thruster *t = thruster_get_struct(tag, __FUNCTION__);
    robot_mutex_unlock();
    return t ? t->maxThrottle : NAN;
}

// Get the specific impulse (returns NAN if no fuel tanks are connected)
double wb_thruster_get_ISR(WbDeviceTag tag)
{
    robot_mutex_lock();
    Thruster *t = thruster_get_struct(tag, __FUNCTION__);
    robot_mutex_unlock();
    return t ? t->myISR : NAN;
}


// Enable recalc of the rocket mass with a refresh period (like wb_*_enable() on sensors)
void wb_thruster_enable_mass_info(WbDeviceTag tag, int sampling_period)
{
  if (sampling_period < 0) {
    fprintf(stderr, "Error: %s() called with negative sampling period.\n", __FUNCTION__);
    return;
  }

  robot_mutex_lock();
  Thruster *thruster = thruster_get_struct(tag, __FUNCTION__);
  if (thruster) {
    thruster->massSampling = sampling_period;
    thruster->req_flags |= C_THRUSTER_ENABLE_MASS_CALC;
  }
  robot_mutex_unlock();
}

// Get the mass of the enire merged Solid the thruster is connected to. 
double wb_thruster_get_rocket_mass(WbDeviceTag tag)
{
    robot_mutex_lock();
    Thruster *t = thruster_get_struct(tag, __FUNCTION__);
    robot_mutex_unlock();
    return t ? t->rocketMass : NAN;
}

// How much fuel, in kg, is left in all fuel tanks connected to the thruster. Returns -1 is no tanks are connected (= infinite fuel)
double wb_thruster_get_fuel(WbDeviceTag tag)
{
    robot_mutex_lock();
    Thruster *t = thruster_get_struct(tag, __FUNCTION__);
    robot_mutex_unlock();
    return t ? t->fuelMass : NAN;
}

// Change the amount of fuel to 'level' of the total capacity tank, where level is between [0 .. 1.0]
void   wb_thruster_refuel(WbDeviceTag tag, double level)
{
    if( level < 0 || level > 1.0 ){
        fprintf(stderr, "Error: %s() - level must be in range [0..1] (of total fuel capacity).\n", __FUNCTION__);
        return;
    }
    robot_mutex_lock();
    Thruster *thruster = thruster_get_struct(tag, __FUNCTION__);
    if( !thruster ) return;
    if( thruster->fuelMass == NAN || thruster->fuelMass < 0 ){
        fprintf(stderr, "Error: %s() - can't refuel, no fuel tanks.\n", __FUNCTION__);
        return;
    }
    thruster->refuelRate = level;
    thruster->req_flags |= C_THRUSTER_SET_FUEL;

    robot_mutex_unlock();
}   

// Fire the rocket! Duration need not be equal to the sim step, the thruster may "burn" through several sim steps, on the last step
// the applied force will be proportionally smaller.
void wb_thruster_start_burn(WbDeviceTag tag, float throttle, int duration)
{
    if( duration <= 0 ){
        fprintf(stderr, "Error: %s() - 'duration' must be positive, but it is: %d.\n", __FUNCTION__, duration);
        return;
    }
    if( throttle <= 0.0 || throttle > 1.0 ){
        fprintf(stderr, "Error: %s() - 'throttle' must be positive and not greater than 1:  %f.\n", __FUNCTION__, throttle);
        return;
    }

    robot_mutex_lock();
    Thruster *thruster = thruster_get_struct(tag, __FUNCTION__);
    if( thruster ){
        if( thruster->fuelMass == 0 ){  // only 0 is bad, if negative we are on infinite fuel
            fprintf(stderr, "Warning: %s() - your rocket is out of fuel.\n", __FUNCTION__);
        }else{
            if( throttle < thruster->maxThrottle ){
                thruster->currThrottle = thruster->maxThrottle;
                fprintf(stderr, "Warning: %s() - 'throttle' can't be less than %f for this thruster, but it's %f.\n", __FUNCTION__, thruster->maxThrottle, throttle);
            }else   
                thruster->currThrottle = throttle;
            thruster->burnState = duration;
            thruster->req_flags |= C_THRUSTER_START_BURN;           
        }
    }
    robot_mutex_unlock();    
}

// Returns the time remaining till the end of the current burn. 
// X > 0 means it is still burning, X = 0 - finished, X < 0 - went out of fuel X seconds before the end of the burn.
int wb_thruster_burn_state(WbDeviceTag tag)
{
    robot_mutex_lock();
    Thruster *t = thruster_get_struct(tag, __FUNCTION__);
    robot_mutex_unlock();
    return t ? t->burnState : NAN;
}




