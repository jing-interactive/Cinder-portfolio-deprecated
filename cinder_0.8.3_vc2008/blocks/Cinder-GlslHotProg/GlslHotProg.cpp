//
//  GlslHotProg.h
//  GlslHotProg
//
//  Created by Joel Pryde on 6/14/12.
//

#include "GlslHotProg.h"
#include "Resources.h"
#include "cinder/app/AppBasic.h"

using namespace ci;
using namespace std;
using namespace ci::app;

GlslHotProg::GlslHotProg( const char* vertPath, const char* fragPath )
{    
    mVertPath       = vertPath;
    mFragPath       = fragPath;
    loadFile();
}

bool GlslHotProg::loadFile()
{   
    mLastVertTime   = ci::fs::last_write_time( getAssetPath( mVertPath ) );
    mLastFragTime   = ci::fs::last_write_time( getAssetPath( mFragPath ) );
    mProg = gl::GlslProg( loadAsset( mVertPath ), loadAsset( mFragPath ) );
    
    return true;
}

bool GlslHotProg::hasChanged()
{
    return ci::fs::last_write_time( getAssetPath( mVertPath ) ) > mLastVertTime || ci::fs::last_write_time( getAssetPath( mFragPath ) ) > mLastFragTime;
}

bool GlslHotProg::update(){
    try {
        if( hasChanged() ){
            return loadFile();
        }
    }
    catch( ... ){}
    return false;
}
