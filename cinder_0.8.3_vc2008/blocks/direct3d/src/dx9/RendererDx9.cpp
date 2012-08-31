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
#include "cinder/app/App.h"
#include "cinder/Utilities.h"
#include "cinder/app/AppImplMsw.h"
#include "dx9/RendererDx9.h"
#include "dx9/dx9.h"

//#include "cinder/ip/Flip.h"

namespace{
    HRESULT hr = S_OK;
    IDirect3D9* md3dObject = NULL;
}

namespace cinder { namespace dx9 {
extern IDirect3DDevice9* g_device;
}}

namespace cinder { namespace app {

const int RendererDX9::sAntiAliasingSamples[] = { 0, 2, 4, 6, 8, 16, 32 };

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RendererDX9::RendererDX9( int aAntiAliasing )
	: Renderer(), mAntiAliasing( aAntiAliasing )
{
    mDevice = NULL;
}

void RendererDX9::setAntiAliasing( int aAntiAliasing )
{
	mAntiAliasing = aAntiAliasing;
}
  
RendererDX9::~RendererDX9()
{

}

void RendererDX9::setup( App *aApp, HWND wnd, HDC dc )
{
	mWnd = wnd;
	mApp = aApp;
    
    mDevType = D3DDEVTYPE_HAL;

	// Step 1: Create the IDirect3D9 object.

    if (!md3dObject)
    {
        md3dObject = Direct3DCreate9(D3D_SDK_VERSION);
	    if( !md3dObject )
	    {
		    MessageBox(0, L"Direct3DCreate9 FAILED", 0, 0);
		    PostQuitMessage(0);
	    }
    }

	// Step 2: Verify hardware support for specified formats in windowed and full screen modes.
	
	D3DDISPLAYMODE mode;
	md3dObject->GetAdapterDisplayMode(D3DADAPTER_DEFAULT, &mode);
	V(md3dObject->CheckDeviceType(D3DADAPTER_DEFAULT, mDevType, mode.Format, mode.Format, TRUE));
	V(md3dObject->CheckDeviceType(D3DADAPTER_DEFAULT, mDevType, D3DFMT_X8R8G8B8, D3DFMT_X8R8G8B8, FALSE));

	// Step 3: Check for requested vertex processing and pure device.

	D3DCAPS9 caps;
	V(md3dObject->GetDeviceCaps(D3DADAPTER_DEFAULT, mDevType, &caps));

	DWORD devBehaviorFlags = 0;
	if( caps.DevCaps & D3DDEVCAPS_HWTRANSFORMANDLIGHT )
		devBehaviorFlags |= D3DCREATE_HARDWARE_VERTEXPROCESSING;
	else
		devBehaviorFlags |= D3DCREATE_SOFTWARE_VERTEXPROCESSING;

	// If pure device and HW T&L supported
	if( caps.DevCaps & D3DDEVCAPS_PUREDEVICE &&
		devBehaviorFlags & D3DCREATE_HARDWARE_VERTEXPROCESSING)
			devBehaviorFlags |= D3DCREATE_PUREDEVICE;

	// Step 4: Fill out the D3DPRESENT_PARAMETERS structure.

	md3dPP.BackBufferWidth            = 0; 
	md3dPP.BackBufferHeight           = 0;
	md3dPP.BackBufferFormat           = D3DFMT_UNKNOWN ;
	md3dPP.BackBufferCount            = 1;
	md3dPP.MultiSampleType            = D3DMULTISAMPLE_NONE;
	md3dPP.MultiSampleQuality         = 0;
	md3dPP.SwapEffect                 = D3DSWAPEFFECT_DISCARD; 
	md3dPP.hDeviceWindow              = mWnd;
	md3dPP.Windowed                   = TRUE;
	md3dPP.EnableAutoDepthStencil     = TRUE; 
	md3dPP.AutoDepthStencilFormat     = D3DFMT_D24S8;
	md3dPP.Flags                      = 0;
    md3dPP.FullScreen_RefreshRateInHz = D3DPRESENT_RATE_DEFAULT;
	md3dPP.PresentationInterval       = D3DPRESENT_INTERVAL_IMMEDIATE;


	// Step 5: Create the device.
    SAFE_RELEASE(mDevice);

	V(md3dObject->CreateDevice(
		D3DADAPTER_DEFAULT, // primary adapter
		mDevType,           // device type
		mWnd,          // window associated with device
		devBehaviorFlags,   // vertex processing
	    &md3dPP,            // present parameters
        &mDevice));      // return created device

    dx9::g_device = mDevice;
}

void RendererDX9::kill()
{
	SAFE_RELEASE(md3dObject);
	SAFE_RELEASE(mDevice);
}

bool RendererDX9::isDeviceLost()
{
	// Get the state of the graphics device.
	HRESULT hr = mDevice->TestCooperativeLevel();

	// If the device is lost and cannot be reset yet then
	// sleep for a bit and we'll try again on the next 
	// message loop cycle.
	if( hr == D3DERR_DEVICELOST )
	{
		sleep(20);
		return true;
	}
	// Driver error, exit.
	else if( hr == D3DERR_DRIVERINTERNALERROR )
	{
		MessageBox(0, L"Internal Driver Error...Exiting", 0, 0);
		PostQuitMessage(0);
		return true;
	}
	// The device is lost but we can reset and restore it.
	else if( hr == D3DERR_DEVICENOTRESET )
	{
		onLostDevice();
		V(mDevice->Reset(&md3dPP));
		onResetDevice();
		return false;
	}
	else
		return false;
}

void RendererDX9::prepareToggleFullScreen()
{
	// Switch to fullscreen mode from windowed mode
	if( md3dPP.Windowed )
	{
		int width  = GetSystemMetrics(SM_CXSCREEN);
		int height = GetSystemMetrics(SM_CYSCREEN);

		md3dPP.BackBufferFormat = D3DFMT_X8R8G8B8;
		md3dPP.BackBufferWidth  = width;
		md3dPP.BackBufferHeight = height;
		md3dPP.Windowed         = false;

		// Change the window style to a more fullscreen friendly style.
		SetWindowLongPtr(mWnd, GWL_STYLE, WS_POPUP);

		// If we call SetWindowLongPtr, MSDN states that we need to call
		// SetWindowPos for the change to take effect.  In addition, we 
		// need to call this function anyway to update the window dimensions.
		SetWindowPos(mWnd, HWND_TOP, 0, 0, width, height, SWP_NOZORDER | SWP_SHOWWINDOW);	
	}
	// Switch to windowed mode.
	else
	{
		RECT R = {0, 0, 800, 600};
		AdjustWindowRect(&R, WS_OVERLAPPEDWINDOW, false);
		md3dPP.BackBufferFormat = D3DFMT_UNKNOWN;
		md3dPP.BackBufferWidth  = 800;//TODO
		md3dPP.BackBufferHeight = 600;
		md3dPP.Windowed         = true;
	
		// Change the window style to a more windowed friendly style.
		SetWindowLongPtr(mWnd, GWL_STYLE, WS_OVERLAPPEDWINDOW);

		// If we call SetWindowLongPtr, MSDN states that we need to call
		// SetWindowPos for the change to take effect.  In addition, we 
		// need to call this function anyway to update the window dimensions.
		SetWindowPos(mWnd, HWND_TOP, 100, 100, R.right, R.bottom, SWP_NOZORDER | SWP_SHOWWINDOW);
	}

	// Reset the device with the changes.
	onLostDevice();
}

void RendererDX9::onLostDevice()
{

}

void RendererDX9::onResetDevice()
{

}

void RendererDX9::finishToggleFullScreen()
{
	V(mDevice->Reset(&md3dPP));
	onResetDevice();
}

void RendererDX9::startDraw()
{
    isDeviceLost();
    V(mDevice->BeginScene());
}

void RendererDX9::finishDraw()
{
    V(mDevice->EndScene());
    V(mDevice->Present(0, 0, 0, 0));
}

void RendererDX9::defaultResize()
{

}

Surface	RendererDX9::copyWindowSurface( const Area &area )
{
	Surface s( area.getWidth(), area.getHeight(), false );
	//glFlush(); // there is some disagreement about whether this is necessary, but ideally performance-conscious users will use FBOs anyway
	//GLint oldPackAlignment;
	//glGetIntegerv( GL_PACK_ALIGNMENT, &oldPackAlignment ); 
	//glPixelStorei( GL_PACK_ALIGNMENT, 1 );
	//glReadPixels( area.x1, mApp->getWindowHeight() - area.y2, area.getWidth(), area.getHeight(), GL_RGB, GL_UNSIGNED_BYTE, s.getData() );
	//glPixelStorei( GL_PACK_ALIGNMENT, oldPackAlignment );	
	//ip::flipVertical( &s );
	return s;
}

} } // namespace cinder::app
