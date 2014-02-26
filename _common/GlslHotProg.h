//
//  GlslHotProg.cpp
//  GlslHotProg
//
//  Created by Joel Pryde on 6/14/12.
//  Refined by vinjn.z
//

#include "cinder/cinder.h"
#include "cinder/Filesystem.h"
#include "cinder/gl/GlslProg.h"

class GlslHotProg 
{
    ci::fs::path mVertPath;
    ci::fs::path mFragPath;
    std::time_t mLastVertTime;
    std::time_t mLastFragTime;
    ci::gl::GlslProg mProg;
    
    bool loadFile();
    //-----------------------------------------------------------------  
    bool update();
    bool hasChanged() const;
    bool reload();

public:
    GlslHotProg() {}

    // TODO: only works with asset paths
    bool load( const char* vertPath, const char* fragPath );
    
    //-----------------------------------------------------------------  
    ci::gl::GlslProg& getProg() { return mProg; }
};
