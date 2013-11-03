//
//  GlslHotProg.cpp
//  GlslHotProg
//
//  Created by Joel Pryde on 6/14/12.
//

#include "cinder/cinder.h"
#include "cinder/DataSource.h"
#include "cinder/Function.h"
#include "cinder/Filesystem.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/GlslProg.h"

class GlslHotProg 
{
    ci::fs::path mVertPath;
    ci::fs::path mFragPath;
    std::time_t mLastVertTime;
    std::time_t mLastFragTime;
    ci::gl::GlslProg mProg;
    
    bool loadFile();
    
public:
    GlslHotProg() {}
    /* !!! NOTE: only works with asset paths */
    GlslHotProg( const char* vertPath, const char* fragPath );
    
    //-----------------------------------------------------------------  
    bool update();
    bool hasChanged();
    bool reload();
    
    //-----------------------------------------------------------------  
    ci::gl::GlslProg& getProg() { return mProg; }
};


        