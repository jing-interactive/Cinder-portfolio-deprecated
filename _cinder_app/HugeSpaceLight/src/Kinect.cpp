#include "LightApp.h"
#include "Config.h"
#include "States.h"
#include "cinder/osc/OscSender.h"
#include "cinder/osc/OscListener.h"
#include "cinder/Utilities.h"
#include "GestureDetector.h"
#include "Arduino.h"

const int kOscPort = 4444;
const int kPadPort = 5555;
const int kKinectPort = 7001;

osc::Listener   mPadListener;
osc::Listener   mKinectListener;
osc::Sender     mPadSender;

GestureDetector mGestures[24]; // TODO:

extern bool mIsAlive;

list<KinectBullet> mKinectBullets;

LightApp* self;

void onOscPadMessage(const osc::Message* msg)
{
    const string& addr = msg->getAddress();

    static bool sIsFirst = true;
    if (sIsFirst)
    {
        console() << "Remote IP: " << msg->getRemoteIp() << endl;
        sIsFirst = false;
        mPadSender.setup(msg->getRemoteIp(), kPadPort);
    }

    if (addr == "/ACK")
    {
        osc::Message msg;
        msg.setAddress("/msgBox");
        msg.addStringArg("Message received.");
        mPadSender.sendMessage(msg);
    }

    if (addr == "/schedule")
    {
        int hour = msg->getArgAsInt32(0);
        int prog = msg->getArgAsInt32(1);
        if (hour >=0 && hour < kHourCount 
            && prog >= -1 && prog < Config::kCount)
        {
            if (mCurrentHour == hour && mConfigIds[hour] != prog)
            {
                // invalidate current hour, refer to updateProgram()
                mHour = -1;
            }
            mConfigIds[hour] = prog;
        }
        writeProgramSettings();
        return;
    }

    if (addr == "/anim")
    {
        int idx = 0;
        const int cfg = msg->getArgAsInt32(idx++);
        Config& config = mConfigs[cfg];
        for (int i=0; i<AnimConfig::kCount; i++)
        {
            AnimConfig& animConfig = config.animConfigs[i];
            int loopCount = msg->getArgAsInt32(idx++);
            animConfig.loopCount = loopCount;
            animConfig.lightValue = msg->getArgAsFloat(idx++);
            animConfig.lightValue2 = msg->getArgAsFloat(idx++);
        }
        writeProgramSettings();
        return;
    }

    if (addr == "/WORLD_VISIBLE")
    {
        WORLD_VISIBLE = msg->getArgAsInt32(0);
    }
}

void updatePusher( int playerId, Vec3f * poses)
{
    float wavingSpeed = 0;
    bool isDetected = false;
    for (int i=0; i<2; i++)
    {
        mGestures[playerId*2+i].update(poses[0], poses[i + 1]);
        if (mGestures[playerId*2+i].isDetected(KINECT_DISTANCE, &wavingSpeed))
        {
            isDetected = true;
        }
    }

    if (!isDetected)
    {
        return;
    }

    mKinectBullets.push_back(KinectBullet(wavingSpeed));
    console() << "Hit " << " speed " << wavingSpeed << endl;
}


void onOscKinectMessage(const osc::Message* msg)
{
    if (!mIsAlive) return;

    if (mCurrentConfig == NULL) return;

    const AnimConfig& kinectCfg = mCurrentConfig->animConfigs[AnimConfig::kKinect];
    if (kinectCfg.loopCount == 0) return;

    const string& addr = msg->getAddress();
    if (addr != "/kinect") return;

    int playerId = msg->getArgAsInt32(1);

    const int SHOULDER_CENTER = 4;
    const int HAND_LEFT = 9;
    const int HAND_RIGHT = 13;
    const int ids[3] = {SHOULDER_CENTER, HAND_LEFT, HAND_RIGHT};
    Vec3f poses[3];

    for (int i=0; i<3; i++)
    {
        string str = msg->getArgAsString(ids[i]);
        vector<string> xyzw = ci::split(str, ",");
        poses[i].set(fromString<float>(xyzw[0]), fromString<float>(xyzw[1]), fromString<float>(xyzw[2]));

        if (poses[0].z > KINECT_FAR || poses[0].z < KINECT_NEAR)
        {
            return;
        }
    }

    mLastKinectMsgSeconds = getElapsedSeconds();

    if (self->mCurrentState == StateIdle::getSingleton())
    {
        sFadeOutNextState = StatePusher::getSingleton(); // StateInteractive::getRandomState();
        self->changeToState(StateFadeOut::getSingleton());
    }

    if (!mIsInteractive)
    {
        return;
    }

    updatePusher(playerId, poses);
}

void LightApp::setupOsc()
{
    self = this;

    for (int i=0; i<24; i++)
    {
        mGestures[i] = GestureDetector(-Vec3f::zAxis());
    }

    mPadListener.setup(kOscPort);
    mPadListener.registerMessageReceived(onOscPadMessage);

    mKinectListener.setup(kKinectPort);
    mKinectListener.registerMessageReceived(onOscKinectMessage);

    setupArduino();
}

void setFinish(KinectBullet* bullet)
{
    bullet->mIsFinished = true;
}

KinectBullet::KinectBullet(float wavingSpeed)
{
    mIsFinished = false;
    kinectSeq = &mAnims[kIdleAnimCount + rand() % kKinectAnimCount];

    //wavingSpeed = 24 * constrain(wavingSpeed * KINECT_MOVIE_SPEED, 1.0f, KINECT_MAX_SPEED);
    index = 0;
    float duration = (float)kinectSeq->seqs[0].size();
    length = duration - 1;
    timeline().apply(&index, length, duration / 24)
        .finishFn(bind(setFinish, this));
}

void KinectBullet::get(Channel* globe, Channel* wall)
{
    *globe = kinectSeq->seqs[0][static_cast<int>(index)];
    *wall = kinectSeq->seqs[1][static_cast<int>(index)];
}

bool KinectBullet::isFinished() const
{
    return mIsFinished;
}
