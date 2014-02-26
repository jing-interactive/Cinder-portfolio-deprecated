//
//  GlslHotProg.h
//  GlslHotProg
//
//  Created by Joel Pryde on 6/14/12.
//

#include "GlslHotProg.h"
#include "cinder/app/AppBasic.h"
#include "cinder/Utilities.h"

using namespace ci;
using namespace std;
using namespace ci::app;

bool GlslHotProg::load( const char* vertPath, const char* fragPath )
{    
    mVertPath       = vertPath;
    mFragPath       = fragPath;

    App::get()->getSignalUpdate().connect( std::bind( &GlslHotProg::update, this ) );

    return loadFile();
}

bool GlslHotProg::loadFile()
{   
    mLastVertTime   = fs::last_write_time( getAssetPath( mVertPath ) );
    mLastFragTime   = fs::last_write_time( getAssetPath( mFragPath ) );
    console() << "Refreshing shaders: " << mVertPath << ", " << mFragPath << endl;
    try
    {
        sleep(500);
        mProg = gl::GlslProg( loadAsset( mVertPath ), loadAsset( mFragPath ) );
    }
    catch (exception& e)
    {
        console() << e.what() << endl;
        return false;
    }
    
    return true;
}

bool GlslHotProg::hasChanged() const
{
    return fs::last_write_time( getAssetPath( mVertPath ) ) > mLastVertTime || 
        fs::last_write_time( getAssetPath( mFragPath ) ) > mLastFragTime;
}

bool GlslHotProg::update(){
    try 
    {
        if( hasChanged() )
        {
            return loadFile();
        }
    }
    catch( ... ){}
    return false;
}
