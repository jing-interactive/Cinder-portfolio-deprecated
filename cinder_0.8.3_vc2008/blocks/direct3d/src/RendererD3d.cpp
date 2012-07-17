/*
 Copyright (c) 2010, The Barbarian Group
 All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are permitted provided that
 the following conditions are met:

    * Redistributions of source code must retain the above copyright notice, this list of conditions and
	the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
	the following disclaimer in the documentation and/or other materials provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
*/

#include "cinder/app/Renderer.h"
/*#include "cinder/gl/gl.h"*/
#include "cinder/app/App.h"

#include "cinder/app/AppImplMsw.h"
#include "cinder/app/AppImplMswRendererD3d.h"

#include "cinder/ip/Flip.h"


namespace cinder { namespace app {

const int RendererD3d::sAntiAliasingSamples[] = { 0, 2, 4, 6, 8, 16, 32 };

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// RendererD3d
RendererD3d::RendererD3d()
	: Renderer(), mImpl( 0 )
{
	mAntiAliasing = AA_MSAA_16;
}

RendererD3d::RendererD3d( int aAntiAliasing )
	: Renderer(), mImpl( 0 ), mAntiAliasing( aAntiAliasing )
{}

void RendererD3d::setAntiAliasing( int aAntiAliasing )
{
	mAntiAliasing = aAntiAliasing;
}
  
RendererD3d::~RendererD3d()
{
	delete mImpl;
}

void RendererD3d::setup( App *aApp, HWND wnd, HDC dc )
{
	mWnd = wnd;
	mApp = aApp;
	if( ! mImpl )
		mImpl = new AppImplMswRendererD3d( mApp, this );
	mImpl->initialize( wnd, dc );
}

void RendererD3d::kill()
{
	mImpl->kill();
}

void RendererD3d::prepareToggleFullScreen()
{
	mImpl->prepareToggleFullScreen();
}

void RendererD3d::finishToggleFullScreen()
{
	mImpl->finishToggleFullScreen();
}

void RendererD3d::startDraw()
{
	mImpl->makeCurrentContext();
}

void RendererD3d::finishDraw()
{
	mImpl->swapBuffers();
}

void RendererD3d::defaultResize()
{
	mImpl->defaultResize();
}

Surface	RendererD3d::copyWindowSurface( const Area &area )
{
	Surface s( area.getWidth(), area.getHeight(), false );
	glFlush(); // there is some disagreement about whether this is necessary, but ideally performance-conscious users will use FBOs anyway
	GLint oldPackAlignment;
	glGetIntegerv( GL_PACK_ALIGNMENT, &oldPackAlignment ); 
	glPixelStorei( GL_PACK_ALIGNMENT, 1 );
	glReadPixels( area.x1, mApp->getWindowHeight() - area.y2, area.getWidth(), area.getHeight(), GL_RGB, GL_UNSIGNED_BYTE, s.getData() );
	glPixelStorei( GL_PACK_ALIGNMENT, oldPackAlignment );	
	ip::flipVertical( &s );
	return s;
}

} } // namespace cinder::app
