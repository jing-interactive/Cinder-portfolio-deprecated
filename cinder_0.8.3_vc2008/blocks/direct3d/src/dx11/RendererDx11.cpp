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
#include "dx11/RendererDx11.h"
#include "dx11/dx11.h"

//#include "cinder/ip/Flip.h"

namespace{
    HRESULT hr = S_OK;
}

namespace cinder { namespace dx11 {
extern ID3D11Device* g_device;
extern ID3D11DeviceContext* g_immediateContex;
extern ID3D11RenderTargetView* g_RenderTargetView;
extern ID3D11DepthStencilView* g_DepthStencilView;
}}

namespace cinder { namespace app {

const int RendererDX11::sAntiAliasingSamples[] = { 0, 2, 4, 6, 8, 16, 32 };

#define CASE_RETURN(item) case item: return #item;

const char* getDescription(D3D_FEATURE_LEVEL featureLevel)
{
    switch (featureLevel)
    {
    CASE_RETURN(D3D_FEATURE_LEVEL_9_1);
    CASE_RETURN(D3D_FEATURE_LEVEL_9_2);
    CASE_RETURN(D3D_FEATURE_LEVEL_9_3);
    CASE_RETURN(D3D_FEATURE_LEVEL_10_0);
    CASE_RETURN(D3D_FEATURE_LEVEL_10_1);
    CASE_RETURN(D3D_FEATURE_LEVEL_11_0);
    default:
        return "Wrong D3D_FEATURE_LEVEL!";
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RendererDX11::RendererDX11( int aAntiAliasing )
	: Renderer(), mAntiAliasing( aAntiAliasing )
{
	md3dDevice = NULL;
	md3dImmediateContext = NULL;
	mSwapChain = NULL;
	mDepthStencilBuffer = NULL;
	mRenderTargetView = NULL;
	mDepthStencilView = NULL;
}

void RendererDX11::setAntiAliasing( int aAntiAliasing )
{
	mAntiAliasing = aAntiAliasing;
}

void RendererDX11::setup( App *aApp, HWND wnd, HDC dc )
{
    kill();

	mWnd = wnd;
	mApp = aApp;
    
    mDevType = D3D_DRIVER_TYPE_HARDWARE;

	// Create the device and device context.

	UINT createDeviceFlags = 0;
#if defined(DEBUG) || defined(_DEBUG)  
    createDeviceFlags |= D3D11_CREATE_DEVICE_DEBUG;
#endif

	D3D_FEATURE_LEVEL featureLevel;
	HRESULT hr = D3D11CreateDevice(
			0,                 // default adapter
			mDevType,
			0,                 // no software device
			createDeviceFlags, 
			0, 0,              // default feature level array
			D3D11_SDK_VERSION,
			&md3dDevice,
			&featureLevel,
			&md3dImmediateContext);

    dx11::g_immediateContex = md3dImmediateContext;

	if( FAILED(hr) )
	{
		MessageBox(0, L"D3D11CreateDevice Failed.", 0, 0);
        mApp->quit();
	}

	if( featureLevel < D3D_FEATURE_LEVEL_10_0 )
	{
		MessageBox(0, L"Direct3D Feature Level 10.0 unsupported.", 0, 0);
		mApp->quit();
	}
    else
    {
        OutputDebugStringA(getDescription(featureLevel));
        OutputDebugStringA("\n");
    }

	// Check 4X MSAA quality support for our back buffer format.
	// All Direct3D 11 capable devices support 4X MSAA for all render 
	// target formats, so we only need to check quality support.

    UINT m4xMsaaQuality;
	V(md3dDevice->CheckMultisampleQualityLevels(
		DXGI_FORMAT_R8G8B8A8_UNORM, 4, &m4xMsaaQuality));
	assert( m4xMsaaQuality > 0 );

	// Fill out a DXGI_SWAP_CHAIN_DESC to describe our swap chain.

	DXGI_SWAP_CHAIN_DESC sd;
    sd.BufferDesc.Width  = mApp->getWindowWidth();
    sd.BufferDesc.Height = mApp->getWindowHeight();
	sd.BufferDesc.RefreshRate.Numerator = 60;
	sd.BufferDesc.RefreshRate.Denominator = 1;
	sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
	sd.BufferDesc.ScanlineOrdering = DXGI_MODE_SCANLINE_ORDER_UNSPECIFIED;
	sd.BufferDesc.Scaling = DXGI_MODE_SCALING_UNSPECIFIED;

	// Use 4X MSAA? 
	if( mAntiAliasing > 0 )
	{
		sd.SampleDesc.Count   = 4;
		sd.SampleDesc.Quality = 0;
	}
	// No MSAA
	else
	{
		sd.SampleDesc.Count   = 1;
		sd.SampleDesc.Quality = 0;
	}

	sd.BufferUsage  = DXGI_USAGE_RENDER_TARGET_OUTPUT;
	sd.BufferCount  = 1;
    sd.OutputWindow = mWnd;
	sd.Windowed     = true;
	sd.SwapEffect   = DXGI_SWAP_EFFECT_DISCARD;
	sd.Flags        = 0;

	// To correctly create the swap chain, we must use the IDXGIFactory that was
	// used to create the device.  If we tried to use a different IDXGIFactory instance
	// (by calling CreateDXGIFactory), we get an error: "IDXGIFactory::CreateSwapChain: 
	// This function is being called with a device from a different IDXGIFactory."

	IDXGIDevice* dxgiDevice = 0;
	V(md3dDevice->QueryInterface(__uuidof(IDXGIDevice), (void**)&dxgiDevice));
	      
	IDXGIAdapter* dxgiAdapter = 0;
	V(dxgiDevice->GetParent(__uuidof(IDXGIAdapter), (void**)&dxgiAdapter));

	IDXGIFactory* dxgiFactory = 0;
	V(dxgiAdapter->GetParent(__uuidof(IDXGIFactory), (void**)&dxgiFactory));

	V(dxgiFactory->CreateSwapChain(md3dDevice, &sd, &mSwapChain));
	
	SAFE_RELEASE(dxgiDevice);
	SAFE_RELEASE(dxgiAdapter);
	SAFE_RELEASE(dxgiFactory);

	// The remaining steps that need to be carried out for d3d creation
	// also need to be executed every time the window is resized.  So
	// just call the OnResize method here to avoid code duplication.
	
	//defaultResize();

    dx11::g_device = md3dDevice;
}

void RendererDX11::kill()
{
	SAFE_RELEASE(mRenderTargetView);
	SAFE_RELEASE(mDepthStencilView);
    SAFE_RELEASE(mDepthStencilBuffer);
	SAFE_RELEASE(mSwapChain);

    // Restore all default settings.
	if( md3dImmediateContext )
		md3dImmediateContext->ClearState();

	SAFE_RELEASE(md3dImmediateContext);
    SAFE_RELEASE(md3dDevice);
}

void RendererDX11::prepareToggleFullScreen()
{
	//// Switch to fullscreen mode from windowed mode
	//if( md3dPP.Windowed )
	//{
	//	int width  = GetSystemMetrics(SM_CXSCREEN);
	//	int height = GetSystemMetrics(SM_CYSCREEN);

	//	md3dPP.BackBufferFormat = D3DFMT_X8R8G8B8;
	//	md3dPP.BackBufferWidth  = width;
	//	md3dPP.BackBufferHeight = height;
	//	md3dPP.Windowed         = false;

	//	// Change the window style to a more fullscreen friendly style.
	//	SetWindowLongPtr(mWnd, GWL_STYLE, WS_POPUP);

	//	// If we call SetWindowLongPtr, MSDN states that we need to call
	//	// SetWindowPos for the change to take effect.  In addition, we 
	//	// need to call this function anyway to update the window dimensions.
	//	SetWindowPos(mWnd, HWND_TOP, 0, 0, width, height, SWP_NOZORDER | SWP_SHOWWINDOW);	
	//}
	//// Switch to windowed mode.
	//else
	//{
	//	RECT R = {0, 0, 800, 600};
	//	AdjustWindowRect(&R, WS_OVERLAPPEDWINDOW, false);
	//	md3dPP.BackBufferFormat = D3DFMT_UNKNOWN;
	//	md3dPP.BackBufferWidth  = mApp->getWindowWidth();//TODO
	//	md3dPP.BackBufferHeight = mApp->getWindowHeight();
	//	md3dPP.Windowed         = true;
	//
	//	// Change the window style to a more windowed friendly style.
	//	SetWindowLongPtr(mWnd, GWL_STYLE, WS_OVERLAPPEDWINDOW);

	//	// If we call SetWindowLongPtr, MSDN states that we need to call
	//	// SetWindowPos for the change to take effect.  In addition, we 
	//	// need to call this function anyway to update the window dimensions.
	//	SetWindowPos(mWnd, HWND_TOP, 100, 100, R.right, R.bottom, SWP_NOZORDER | SWP_SHOWWINDOW);
	//}
}

void RendererDX11::finishToggleFullScreen()
{
}

void RendererDX11::startDraw()
{
}

void RendererDX11::finishDraw()
{
    V(mSwapChain->Present(0, 0));
}

void RendererDX11::defaultResize()
{
	assert(md3dImmediateContext);
	assert(md3dDevice);
	assert(mSwapChain);

	// Release the old views, as they hold references to the buffers we
	// will be destroying.  Also release the old depth/stencil buffer.

	SAFE_RELEASE(mRenderTargetView);
	SAFE_RELEASE(mDepthStencilView);
	SAFE_RELEASE(mDepthStencilBuffer);

	// Resize the swap chain and recreate the render target view.

	V(mSwapChain->ResizeBuffers(1, mApp->getWindowWidth(), mApp->getWindowHeight(), DXGI_FORMAT_R8G8B8A8_UNORM, 0));
	ID3D11Texture2D* backBuffer;
	V(mSwapChain->GetBuffer(0, __uuidof(ID3D11Texture2D), reinterpret_cast<void**>(&backBuffer)));
	V(md3dDevice->CreateRenderTargetView(backBuffer, 0, &mRenderTargetView));
	SAFE_RELEASE(backBuffer);

	// Create the depth/stencil buffer and view.

	D3D11_TEXTURE2D_DESC depthStencilDesc;
	
    depthStencilDesc.Width     = mApp->getWindowWidth();
	depthStencilDesc.Height    = mApp->getWindowHeight();
	depthStencilDesc.MipLevels = 1;
	depthStencilDesc.ArraySize = 1;
	depthStencilDesc.Format    = DXGI_FORMAT_D24_UNORM_S8_UINT;

	// Use 4X MSAA? --must match swap chain MSAA values.
	if( mAntiAliasing > 0 )
	{
		depthStencilDesc.SampleDesc.Count   = 4;
		depthStencilDesc.SampleDesc.Quality = 0;
	}
	// No MSAA
	else
	{
		depthStencilDesc.SampleDesc.Count   = 1;
		depthStencilDesc.SampleDesc.Quality = 0;
	}

	depthStencilDesc.Usage          = D3D11_USAGE_DEFAULT;
	depthStencilDesc.BindFlags      = D3D11_BIND_DEPTH_STENCIL;
	depthStencilDesc.CPUAccessFlags = 0; 
	depthStencilDesc.MiscFlags      = 0;

	V(md3dDevice->CreateTexture2D(&depthStencilDesc, 0, &mDepthStencilBuffer));
	V(md3dDevice->CreateDepthStencilView(mDepthStencilBuffer, 0, &mDepthStencilView));


	// Bind the render target view and depth/stencil view to the pipeline.

	md3dImmediateContext->OMSetRenderTargets(1, &mRenderTargetView, mDepthStencilView);
	

	// Set the viewport transform.
    D3D11_VIEWPORT viewport;
	viewport.TopLeftX = 0;
	viewport.TopLeftY = 0;
	viewport.Width    = static_cast<float>(mApp->getWindowWidth());
	viewport.Height   = static_cast<float>(mApp->getWindowHeight());
	viewport.MinDepth = 0.0f;
	viewport.MaxDepth = 1.0f;

	md3dImmediateContext->RSSetViewports(1, &viewport);

    // assign to global ones
    dx11::g_RenderTargetView = mRenderTargetView;
    dx11::g_DepthStencilView = mDepthStencilView;
}

Surface	RendererDX11::copyWindowSurface( const Area &area )
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
