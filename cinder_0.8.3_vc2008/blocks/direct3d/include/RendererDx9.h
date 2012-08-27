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

#pragma once

#include "cinder/Cinder.h"
#include "cinder/Surface.h"
#include "cinder/Display.h"

#include <d3d9.h>

namespace cinder { namespace app {

class App;

class RendererDX9 : public Renderer {
 public:
	RendererDX9(int aAntiAliasing = 0);
	~RendererDX9();
  
	virtual void	setup( App *aApp, HWND wnd, HDC dc );
	virtual void	kill();
	virtual HWND	getHwnd() { return mWnd; }
	virtual void	prepareToggleFullScreen();
	virtual void	finishToggleFullScreen();

	enum	{ AA_NONE = 0, AA_MSAA_2, AA_MSAA_4, AA_MSAA_6, AA_MSAA_8, AA_MSAA_16, AA_MSAA_32 };
	static const int	sAntiAliasingSamples[];
	void				setAntiAliasing( int aAntiAliasing );
	int					getAntiAliasing() const { return mAntiAliasing; }

	virtual void	startDraw();
	virtual void	finishDraw();
	virtual void	defaultResize();
	virtual Surface	copyWindowSurface( const Area &area );
	
 protected:

    bool isDeviceLost();
    void onLostDevice();
    void onResetDevice();

	int			mAntiAliasing;
    IDirect3DDevice9* mDevice;
    IDirect3DDevice9* mPrevDevice;
	HWND        mWnd;
    D3DDEVTYPE mDevType;
    D3DPRESENT_PARAMETERS md3dPP;
};

} } // namespace cinder::app