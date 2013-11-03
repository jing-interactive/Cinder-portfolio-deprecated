Cinder-GlslHotProg
==================

Wrapper for Cinder's GlslProg to facilitate hot loading of shaders.

This is a simple way to wrap Cinder's GLSL programs so that you can update them on the fly by just editing and then shading the program.  As soon as the program is saved, the new shader will auto reload.  Only works with shaders loaded from assets (and will probably crash with any others).

setup shader:
mSphereShaderHot= GlslHotProg( "sphere.vert", "sphere.frag" );

update (need to put this in your update loop):
mSphereShaderHot.update();

use shader:
mSphereShaderHot.getProg().bind();
mSphereShaderHot.getProg().uniform( "cubeMap", 0 );
mSphereShaderHot.getProg().uniform( "radius", 70.0f );
mSphereShaderHot.getProg().uniform( "time", (float)app::getElapsedSeconds() );
mSphereShaderHot.getProg().uniform( "mvpMatrix", mSpringCam.mMvpMatrix );
mSphereShaderHot.getProg().uniform( "eyePos", mSpringCam.getEye() );
mSphereShaderHot.getProg().uniform( "power", mRoom.getPower() );
mSphereShaderHot.getProg().uniform( "roomDim", mRoom.getDims() );
gl::drawSphere( Vec3f::zero(), 1.0f, 128 );
mSphereShaderHot.getProg().unbind();


