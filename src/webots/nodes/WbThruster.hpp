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

#ifndef WB_THRUSTER_HPP
#define WB_THRUSTER_HPP

#include "WbMFColor.hpp"
#include "WbSolidDevice.hpp"

class WbSFBool;
class WbGroup;

class WbDownloader;

class QDataStream;

class WbThruster : public WbSolidDevice {
    Q_OBJECT

public:
    // constructors and destructor
    explicit WbThruster(WbTokenizer *tokenizer = NULL);
    WbThruster(const WbThruster &other);
    explicit WbThruster(const WbNode &other);
    virtual ~WbThruster();

    // reimplemented public functions
    int nodeType() const override { return WB_NODE_THRUSTER; }
    void preFinalize() override;
    void postFinalize() override;
    void reset(const QString &id) override;

    // field accessors

    void powerOn(bool) override;
    void handleMessage(QDataStream &) override;
    void downloadAssets() override;

    // processing before / after ODE world step
    void prePhysicsStep(double ms) override;
    void postPhysicsStep() override;

protected slots:
    void updateChildren() override;
    virtual void updateIfNeeded(WbField *);

private slots:
    void updateSound();
    void updateMaxThrust();
    void updateThrottle();
    void updateISP();
    void updateFuelDensity();
    void updateFuelTanks();
    void updateFeedOrder();
    void updateFlameShape();

private:
    // user accessible fields

    // Thruster WRL fields
    WbSFDouble *mMaxThrust;   // max thrust in Newtons
    WbSFDouble *mMaxThrottle;    // minimum possible thrust is throttle*maxThrust.  1.0 means it's either ON of OFF (no throttling)
    WbSFDouble *mISP;         // Specific impulse of the engine, only used when the
    WbSFDouble *mFuelDensity;  // Fuel density (default as in RP1)
    WbMFNode   *mFuelTanks;    // Refers to a fuel tank(s) node that feeds
    WbSFBool   *mFeedOrder;    // If there are several tanks, empty them one-by-one, otherwise change masses proportionally

    WbSFNode   *mFlameShape;  // It's a Shape kinematic Solid containing both helix's representation and boundingObject for high angular speeds
    WbSFString *mSoundUrl;        // wave file of the motor sound

    // Ptr to care about
    WbSoundClip *mSoundClip;
    WbDownloader *mDownloader;

    // Calculated values
    float   rocketMass;
    float   myISR;
    int     massSamplingRate;

    // status
    int     mBurnState;
    bool    mNeedToConfigure;
    double  mCurrThrust;        // Thrust set for the current burn
    double  mTSFC;              // Thrust-specific fuel consumption
    double  mTankVolume;
    double  mFuelLeft;          //

    WbThruster &operator=(const WbThruster &);  // non copyable
    WbNode *clone() const override { return new WbThruster(*this); }
    void init();

    void start_burn(int burn_time, double thrust_rate);
    void refuelTanks(double fuel_level);

};

#endif
