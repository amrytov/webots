// Copyright 1996-2022 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "WbThruster.hpp"

#include "WbAppearance.hpp"
#include "WbField.hpp"
#include "WbGroup.hpp"
// #include "WbLight.hpp"
#include "WbMFNode.hpp"
#include "WbMaterial.hpp"
#include "WbPbrAppearance.hpp"
#include "WbShape.hpp"
#include "WbUrl.hpp"
#include "WbNetwork.hpp"
#include "WbDownloader.hpp"
#include "../sound/WbSoundEngine.hpp"
#include "../../controller/c/messages.h"  // contains the definitions for the macros C_SET_LED

#include <ode/ode.h>


#include <QtCore/QDataStream>
#include <cassert>

void WbThruster::init() 
{
    mBurnState = 0;
    mNeedToConfigure = false;
    mSoundClip = NULL;

    mMaxThrust  = findSFDouble("maxThrust");   // max thrust in Newtons
    mMaxThrottle   = findSFDouble("throttle");    // minimum possible thrust is throttle*maxThrust.  1.0 means it's either ON of OFF (no throttling)
    mISP        = findSFDouble("ISP");         // Specific impulse of the engine, only used when the
    mFuelDensity = findSFDouble("fuelDensity");  // Fuel density (default as in RP1)
    mFuelTanks  = findMFNode("fuelTanks");    // Refers to a fuel tank(s) node that feeds
    mFeedOrder  = findSFBool("seqFeed");      // If there are several tanks, empty them one-by-one, otherwise change masses proportionally
    mFlameShape = findSFNode("flameShape");    // Refers to a fuel tank(s) node that feeds
    mSoundUrl   = findSFString("sound");

    WbSFNode   *mFlameShape;  // It's a Shape kinematic Solid containing both helix's representation and boundingObject for high angular speeds
    WbSFString *mSoundUrl;        // wave file of the motor sound   
}

WbThruster::WbThruster(WbTokenizer *tokenizer) : WbSolidDevice("Thruster", tokenizer) {
  init();
}

WbThruster::WbThruster(const WbThruster &other) : WbSolidDevice(other) {
  init();
}

WbThruster::WbThruster(const WbNode &other) : WbSolidDevice(other) {
  init();
}

WbThruster::~WbThruster() 
{
  // ??
}

void WbThruster::preFinalize() 
{
    WbSolidDevice::preFinalize();

    updateSound();
    updateMaxThrust();
    updateThrottle();
    updateISP();
    updateFuelDensity();
    updateFuelTanks();
    updateFeedOrder();
    updateFlameShape();

}

void WbThruster::postFinalize() 
{
  WbSolidDevice::postFinalize();
  updateChildren();
  assert(robot());

  connect(mMaxThrust, &WbSFDouble::changed, this, &WbThruster::updateMaxThrust);
  connect(mISP, &WbSFDouble::changed, this, &WbThruster::updateISP);
  connect(mFuelDensity, &WbSFDouble::changed, this, &WbThruster::updateFuelDensity);
  connect(mMaxThrottle, &WbSFDouble::changed, this, &WbThruster::updateThrottle);
  connect(mFeedOrder, &WbSFBool::changed, this, &WbThruster::updateFeedOrder);
  connect(mFuelTanks, &WbMFNode::changed, this, &WbThruster::updateFuelTanks);
  connect(mFlameShape, &WbSFNode::changed, this, &WbThruster::updateFlameShape);
  connect(mSoundUrl, &WbSFString::changed, this, &WbThruster::updateSound);
}

/*
void WbMotor::preFinalize() {
  WbJointDevice::preFinalize();

  cMotors << this;

  // note: only parameter validity check has to occur in preFinalize, validity across couplings must be done in postFinalize
  updateMaxVelocity();
  updateMaxAcceleration();
  updateControlPID();
  updateMinAndMaxPosition();
  updateMultiplier();
  updateSound();

  mForceOrTorqueSensor = new WbSensor();

  const WbJoint *const j = joint();
  mTargetPosition = j ? position() : 0.0;
  mMotorForceOrTorque = mMaxForceOrTorque->value();
  updateMaxForceOrTorque();
}

void WbMotor::postFinalize() {
  WbJointDevice::postFinalize();
  assert(robot());
  if (!mMuscles->isEmpty() || robot()->maxEnergy() > 0)
    setupJointFeedback();

  inferMotorCouplings();  // it also checks consistency across couplings

  // must be done in the postFinalize step as the coupled motor consistency check could change the values
  mTargetVelocity = mMaxVelocity->value();
  mNeedToConfigure = true;  // notify libController about the changes done by the check

  WbMFIterator<WbMFNode, WbNode *> it(mMuscles);
  while (it.hasNext())
    dynamic_cast<WbMuscle *>(it.next())->postFinalize();

  connect(mMultiplier, &WbSFDouble::changed, this, &WbMotor::updateMultiplier);
  connect(mMaxVelocity, &WbSFDouble::changed, this, &WbMotor::updateMaxVelocity);
  connect(mAcceleration, &WbSFDouble::changed, this, &WbMotor::updateMaxAcceleration);
  connect(mControlPID, &WbSFVector3::changed, this, &WbMotor::updateControlPID);
  connect(mMinPosition, &WbSFDouble::changed, this, &WbMotor::updateMinAndMaxPosition);
  connect(mMaxPosition, &WbSFDouble::changed, this, &WbMotor::updateMinAndMaxPosition);
  connect(mMinPosition, &WbSFDouble::changed, this, &WbMotor::minPositionChanged);
  connect(mMaxPosition, &WbSFDouble::changed, this, &WbMotor::maxPositionChanged);
  connect(mSound, &WbSFString::changed, this, &WbMotor::updateSound);
  connect(mMuscles, &WbSFNode::changed, this, &WbMotor::updateMuscles);
  connect(mMaxForceOrTorque, &WbSFDouble::changed, this, &WbMotor::updateMaxForceOrTorque);
  connect(mDeviceName, &WbSFString::changed, this, &WbMotor::inferMotorCouplings);
}
*/

void WbThruster::reset(const QString &id) 
{
  WbSolidDevice::reset(id);
  // ???
}

void WbThruster::updateIfNeeded(WbField *field) {
  if (field->name() == "material" || field->name() == "appearance")
    updateChildren();
}

void WbThruster::updateChildren() {
  WbSolid::updateChildren();

  if (!isPostFinalizedCalled())
    return;

  // ???
}

void WbThruster::downloadAssets() 
{
    const QString &soundString = mSoundUrl->value();
    if (soundString.isEmpty())
        return;

    const QString &completeUrl = WbUrl::computePath(this, "sound", soundString);
    if (!WbUrl::isWeb(completeUrl) || WbNetwork::instance()->isCached(completeUrl))
        return;

    if (mDownloader != NULL)
        delete mDownloader;
    mDownloader = new WbDownloader(this);
    if (isPostFinalizedCalled())
        connect(mDownloader, &WbDownloader::complete, this, &WbThruster::updateSound);

    mDownloader->download(QUrl(completeUrl));
}

/*

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
*/

// Receiving a data package from the controller
void WbThruster::handleMessage(QDataStream &stream) {
  unsigned char byte;
  int v;
  double d;
  stream >> byte;

  switch (byte) {
    case C_THRUSTER_START_BURN:   // int burn_time, double thrust_rate
      stream >> v;
      stream >> d;
      start_burn(v, d);
      break;
    case C_THRUSTER_ENABLE_MASS_CALC:   // int sampling_rate
      stream >> massSamplingRate;
      break;
    case C_THRUSTER_SET_FUEL:
      double refuel;
      stream >> refuel;
      refuelTanks(refuel);
      break;
    default:
      assert(0);
  }
}

void WbThruster::start_burn(int burn_time, double thrust_rate)
{
    mBurnState = burn_time;
    mCurrThrust = mMaxThrust->value() * thrust_rate;
  //??
}

void WbThruster::refuelTanks(double fuel_level)
{
    if( mTSFC <= 0 ){
        warn(tr("Can't refuel thruster - already running on infinite fuel (no fuel tanks defined)."));
        return;
    }

    if( fuel_level < 0 || fuel_level > 1.0 ){
        warn(tr("Refueling level must be in [0..1], got %f instead").arg(fuel_level));
        return;
    }  

    // ???  refuelling doesn't work yet!
    // getFullTankVolume() * fuel_level * mFuelDensity
    // depending on mFeedOrder we either refill tanks one-by-one in reverse order, or distribute extra fuel to each tank proportionally
    // in the simplest case there is only one tank, so we just refill it.
}

// processing before / after ODE world step
void WbThruster::prePhysicsStep(double ms)
{
    WbSolid::prePhysicsStep(ms);

    if( mBurnState <= 0 || mCurrThrust <= 0 ) return;  //  Nothing to do

    // Must be running in the dynamic mode
    const WbSolid *const us = upperSolid();
    if( !us || !us->isSolidMerger() ){
      parsingWarn(tr("Add a Physics node to Solid ancestors to enable thrust."));
      mBurnState = 0;
      return;
    }

    double thrust = mCurrThrust;

    // mBurnState keeps the remaining burn duration.
    int ms_burn = (int)(mBurnState > ms ? ms : mBurnState);

    // Do we have enough fuel for the current refresh interval?
    if( mTSFC > 0 ){
      int ms_left = (int)((mFuelLeft / (mTSFC*thrust)) * 1000.0);
      if( ms_left < ms_burn ){
        ms_burn = ms_left;
        mBurnState -= ms_burn; 
        mBurnState = -mBurnState;   // Inverse to indicate we are out of fuel
        parsingWarn(tr("Thruster is out of fuel!"));

      }else
        mBurnState -= ms_burn; 
      mFuelLeft -= (ms_burn/1000.0) * mTSFC * thrust;
      redistributeFuel();
    }  

    // if the rocket should work for less than the full sim period, reduce the force proportionally
    if( ms_burn < ms )
      thrust *= ms_burn / ms;

    // Find the body to which the force should be applied
    const WbSolid *const us = upperSolid();
    const WbSolidMerger *const sm = us->solidMerger();

    dBodyID b = sm ? sm->body() : NULL;
    if (b == NULL) {
      parsingWarn(tr("Add a Physics node to Solid ancestors to enable thrust."));
      return;
    }

    // Computes thrust
    const WbVector3 tv(thrust, 0, 0);
    const WbVector3 &cot = matrix() * tv;

    // Applies thrust and torque
    const WbMatrix3 &m3 = rotationMatrix();
    const WbVector3 &axisVector = m3 * mNormalizedAxis;
    const WbVector3 &thrustVector = thrust * axisVector;

    if (sm && !sm->isBodyArtificiallyDisabled())
      dBodyEnable(b);
    dBodyAddForceAtPos(b, thrustVector.x(), thrustVector.y(), thrustVector.z(), cot.x(), cot.y(), cot.z());

}

/*
void WbPropeller::prePhysicsStep(double ms) {
  mCurrentThrust = 0.0;
  mCurrentTorque = 0.0;

  WbRotationalMotor *const m = motor();
  if (m == NULL)
    return;

  const bool run = m->runKinematicControl(ms, mPosition);
  if (!run)
    return;

  const double velocity = m->currentVelocity();
  const double absoluteVelocity = fabs(velocity);
  if (absoluteVelocity > 0.0) {
    const WbSolid *const us = upperSolid();
    const WbSolidMerger *const sm = us->solidMerger();
    dBodyID b = sm ? sm->body() : NULL;
    if (b == NULL) {
      parsingWarn(tr("Adds a Physics node to Solid ancestors to enable thrust and torque effect."));
      return;
    }

    // Computes thrust and torque
    const WbTransform *const ut = upperTransform();
    const WbVector3 &cot = ut->matrix() * mCenterOfThrust->value();
    double vp[4];
    dBodyGetPointVel(b, cot.x(), cot.y(), cot.z(), vp);
    const double V = dCalcVectorDot3(vp, mNormalizedAxis.ptr());

    const WbVector2 &tcs = mTorqueConstants->value();
    mCurrentTorque = tcs.x() * velocity * absoluteVelocity - tcs.y() * absoluteVelocity * V;
    const double mt = m->maxForceOrTorque();
    if (fabs(mCurrentTorque) > mt)
      mCurrentTorque = mCurrentTorque > 0.0 ? mt : -mt;

    const WbVector2 &fcs = mThrustConstants->value();
    mCurrentThrust = fcs.x() * velocity * absoluteVelocity - fcs.y() * absoluteVelocity * V;

    // Applies thrust and torque
    const WbMatrix3 &m3 = ut->rotationMatrix();
    const WbVector3 &axisVector = m3 * mNormalizedAxis;
    const WbVector3 &thrustVector = mCurrentThrust * axisVector;
    const WbVector3 &torqueVector = -mCurrentTorque * axisVector;
    if (sm && !sm->isBodyArtificiallyDisabled())
      dBodyEnable(b);
    dBodyAddForceAtPos(b, thrustVector.x(), thrustVector.y(), thrustVector.z(), cot.x(), cot.y(), cot.z());
    dBodyAddTorque(b, torqueVector.x(), torqueVector.y(), torqueVector.z());

    updateHelix(absoluteVelocity);
    if (mHelix == NULL)
      return;

    // Moves the slow helix
    const WbQuaternion q(mNormalizedAxis, mPosition);
    const WbQuaternion iq(mHelix->rotationFromFile(stateId()).toQuaternion());
    WbQuaternion qp(q * iq);
    if (qp.w() != 1.0)
      qp.normalize();
    WbRotation r(qp);
    if (r.angle() == 0.0)
      r = WbRotation(mNormalizedAxis.x(), mNormalizedAxis.y(), mNormalizedAxis.z(), 0.0);
    const WbVector3 &c = mCenterOfThrust->value();
    mHelix->setTranslationAndRotation(q * (mHelix->translationFromFile(stateId()) - c) + c, r);
  }
}

*/

void WbThruster::postPhysicsStep()
{
    WbSolid::postPhysicsStep();
  
}



void WbThruster::powerOn(bool e) {  // turn off
  WbDevice::powerOn(e);
  if (!e)
    setValue(0);
}

//  Field updaters

void WbThruster::updateSound() {
  const QString &soundString = mSoundUrl->value();
  if (soundString.isEmpty()) {
    mSoundClip = NULL;
  } else {
    const QString &completeUrl = WbUrl::computePath(this, "sound", mSoundUrl->value());
    if (WbUrl::isWeb(completeUrl)) {
      if (mDownloader && !mDownloader->error().isEmpty()) {
        warn(mDownloader->error());  // failure downloading or file does not exist (404)
        mSoundClip = NULL;
        // downloader needs to be deleted in case the URL is switched back to something valid
        delete mDownloader;
        mDownloader = NULL;
        return;
      }
      if (!WbNetwork::instance()->isCached(completeUrl)) {
        downloadAssets();
        return;
      }
    }

    // at this point the sound must be available (locally or in the cache).
    // determine extension from URL since for remotely defined assets the cached version does not retain this information
    const QString extension = completeUrl.mid(completeUrl.lastIndexOf('.') + 1).toLower();
    if (WbUrl::isWeb(completeUrl))
      mSoundClip = WbSoundEngine::sound(WbNetwork::instance()->get(completeUrl), extension);
    else
      mSoundClip = WbSoundEngine::sound(completeUrl, extension);
  }
  WbSoundEngine::clearAllMotorSoundSources();
}

void WbThruster::updateMaxThrust()
{

}

void WbThruster::updateThrottle()
{

}

void WbThruster::updateISP()
{

}

void WbThruster::updateFuelDensity()
{

}

void WbThruster::updateFuelTanks()
{

}

void WbThruster::updateFeedOrder()
{

}

void WbThruster::updateFlameShape()
{

}
