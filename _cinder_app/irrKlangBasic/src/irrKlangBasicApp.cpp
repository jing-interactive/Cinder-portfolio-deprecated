#include "cinder/app/AppBasic.h"
#include "cinder/Timeline.h"
#include "cinder/Rand.h"
#include "cinder/Thread.h"

#include "irrKlang.h"

#pragma comment( lib, "irrklang.lib" )

using namespace ci;
using namespace ci::app;

//! A free function designed to interact with irrklangDelete
template <typename T>
void irrklangDelete( T *p )
{
    if( irrklang::IRefCounted* ref = dynamic_cast<irrklang::IRefCounted*>( p ) ) {
        ref->drop();
    }
}

//! Creates a shared_ptr whose deleter will properly decrement the reference count of a irrklang object
template <typename T>
inline std::shared_ptr<T> makeIrrklangShared( T *p )
{ return std::shared_ptr<T>( p, &irrklangDelete<T>); }


#define CATCH_EXCEPTION( ExceptionType ) \
    catch ( ExceptionType& e ){\
    console() << #ExceptionType << e.what() << std::endl;}

namespace
{
    const float MIN_VOLUME = 0.0f;
    const float MAX_VOLUME = 1.0f;
    const float DURATION_VOLUME = 4.0f;

    const float MIN_PAN = -1.0f;
    const float MAX_PAN = +1.0f;
    const float DURATION_PAN = 4.0f;
}

class irrKlangBasicApp : public AppBasic {
public:

    void setup()
    {
        Rand::randomize();

        mSoundEngine = makeIrrklangShared(irrklang::createIrrKlangDevice());

        fs::path root = getAssetPath("./");
        fs::directory_iterator end_iter;
        for ( fs::directory_iterator dir_iter(root); dir_iter != end_iter; ++dir_iter)
        {
            if (fs::is_regular_file(*dir_iter) )
            {
                loadAudio(*dir_iter);
            }
        }

        setupNewSound();

        //timeline().apply( &mVolume, MIN_VOLUME, MAX_VOLUME, DURATION_VOLUME, easeInOutQuad ).pingPong().loop();
        timeline().apply( &mPan, MIN_PAN, MAX_PAN, DURATION_PAN, easeOutInQuart ).pingPong().loop();
    }

    void keyDown( KeyEvent event )
    {
        mCurrentSound->stop();
    }

    void update()
    {
#ifdef _DEBUG
        {
            std::lock_guard<std::mutex> lock(mMutexConsole);
            console() << "Played: " << mTimer.getSeconds() << " ";
            if ( mVolume <= 0.99f )
                console() << mVolume;
            console() << std::endl;
        }
#endif
        if ( mCurrentSound )
            mCurrentSound->setVolume( mVolume );
        //mCurrentSound->setPan( mPan );
    }

    void draw()
    {
        gl::clear();
        gl::drawSolidCircle( Vec2f( 
            lmap<float>( mPan, MIN_PAN, MAX_PAN, 0, getWindowWidth() ), 
            lmap<float>( mVolume, MIN_VOLUME, MAX_VOLUME, getWindowHeight(), 0 )), 
            10 );
    }

public:

    struct SoundStopCallback : public irrklang::ISoundStopEventReceiver
    {
        void OnSoundStopped(irrklang::ISound* sound, irrklang::E_STOP_EVENT_CAUSE reason, void* userData)
        {
            irrKlangBasicApp* app = reinterpret_cast<irrKlangBasicApp*>( userData );
            {
                std::lock_guard<std::mutex> lock( app->mMutexConsole );
                app::console() << "Sound stopped: " << sound->getSoundSource()->getName() << std::endl;

#define CASE(tag) case tag:  app::console() << #tag << std::endl; break;
                switch (reason)
                {
                    CASE(irrklang::ESEC_SOUND_FINISHED_PLAYING);
                    CASE(irrklang::ESEC_SOUND_STOPPED_BY_USER);
                    CASE(irrklang::ESEC_SOUND_STOPPED_BY_SOURCE_REMOVAL);
                }
#undef CASE
            }
            if (app)
                app->setupNewSound();
        }
    }mSoundStopCallback;

private:
    void loadAudio(const fs::path& pathName)
    {
        if (!pathName.empty())
        {
            bool preload = true;
            irrklang::ISoundSource* src = mSoundEngine->addSoundSourceFromFile( 
                pathName.string().c_str(), 
                irrklang::ESM_AUTO_DETECT,
                preload);
            if ( src )
            {
                mSoundSources.push_back( src );
                console() << "Sound file recognized: " << pathName.string() << std::endl;
            }
        }
    }

    void setupNewSound() 
    {
        irrklang::ISoundSource* source = getNextSoundSource();
        float length = source->getPlayLength() * 0.001f;
        app::console() << "Sound started: " << source->getName() 
            << " seconds: " << length
            << std::endl;
        bool loop = false;
        bool pause = false;
        bool track = true;
        mCurrentSound = mSoundEngine->play2D( source, loop, pause, track );
        if ( mCurrentSound)
        {
            mCurrentSound->setVolume( MIN_VOLUME );
            mCurrentSound->setSoundStopEventReceiver( &mSoundStopCallback, this );
            float duration = math<float>::min( length * 0.4f, DURATION_VOLUME );
            mVolume = MIN_VOLUME;
            timeline().apply( &mVolume, MAX_VOLUME, duration, easeInOutQuad );// fade in
            timeline().appendTo( &mVolume, MIN_VOLUME, duration, easeInOutQuad ).delay( length - duration * 2 );// delayed fade out
            mTimer.start();
        }
        else
        {

        }
    }

    irrklang::ISoundSource* getNextSoundSource() 
    {
        size_t idx = Rand::randInt( mSoundSources.size() );
        return mSoundSources[ idx ];
    }

private:
    std::vector<irrklang::ISoundSource* > mSoundSources;
    irrklang::ISound* mCurrentSound;
    Anim<float> mVolume;
    Anim<float> mPan;
    std::shared_ptr<irrklang::ISoundEngine> mSoundEngine;
    Timer   mTimer;

public:
    std::mutex mMutexConsole;
};

CINDER_APP_BASIC( irrKlangBasicApp, RendererGl )
