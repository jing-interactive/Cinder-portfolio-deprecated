#include "dx11/dx11.h"

#pragma comment( lib, "dxerr.lib" )
#pragma comment( lib, "dxguid.lib" )
#pragma comment( lib, "d3d11.lib" )
#ifdef _DEBUG
#pragma comment( lib, "d3dx11d.lib" )
#else
#pragma comment( lib, "d3dx11.lib" )
#endif

#include "cinder/CinderMath.h"
#include "cinder/Vector.h"
#include "cinder/Camera.h"
#include "cinder/TriMesh.h"
#include "cinder/Sphere.h"
//#include "dx9/Texture.h"
#include "cinder/Text.h"
#include "cinder/PolyLine.h"
#include "cinder/Path2d.h"
#include "cinder/Shape2d.h"
#include "cinder/Triangulate.h"

namespace{
HRESULT hr = S_OK;
}

namespace cinder { namespace dx11 {

ID3D11Device* g_device = NULL;
ID3D11DeviceContext* g_immediateContex = NULL;
ID3D11RenderTargetView* g_RenderTargetView = NULL;
ID3D11DepthStencilView* g_DepthStencilView = NULL;

ID3D11Device* getDevice()
{
    return g_device;
}

ID3D11DeviceContext* getImmediateContext()
{
    return g_immediateContex;
}

void clear( const ColorA &color, bool clearDepthBuffer, float clearZ)
{
	g_immediateContex->ClearRenderTargetView(g_RenderTargetView, reinterpret_cast<const float*>(&color));

    if (clearDepthBuffer)
        g_immediateContex->ClearDepthStencilView(g_DepthStencilView, D3D11_CLEAR_DEPTH|D3D11_CLEAR_STENCIL, clearZ, 0);
}

void setModelView( const Camera &cam )
{
    //g_immediateContex->SetTransform(D3DTS_VIEW, (D3DMATRIX*)cam.getModelViewMatrix().m);
}

void setProjection( const Camera &cam )
{
    //g_immediateContex->SetTransform(D3DTS_PROJECTION, (D3DMATRIX*)cam.getProjectionMatrix().m);
}

void setMatrices( const Camera &cam )
{
	setProjection( cam );
	setModelView( cam );
}

void enableWireframe()
{
    //g_immediateContex->SetRenderState(D3DRS_FILLMODE, D3DFILL_SOLID);
}

void disableWireframe()
{
    //g_immediateContex->SetRenderState(D3DRS_FILLMODE, D3DFILL_WIREFRAME);
}

void disableDepthRead()
{
	//g_immediateContex->SetRenderState(D3DRS_ZENABLE, D3DZB_TRUE);
}

void enableDepthRead( bool enable )
{
	//if( enable )
	//	//g_immediateContex->SetRenderState(D3DRS_ZENABLE, D3DZB_TRUE);
	//else
	//	//g_immediateContex->SetRenderState(D3DRS_ZENABLE, D3DZB_FALSE);
}

void enableDepthWrite( bool enable )
{
	//if( enable )
	//	//g_immediateContex->SetRenderState(D3DRS_ZWRITEENABLE, TRUE);
	//else
	//	//g_immediateContex->SetRenderState(D3DRS_ZWRITEENABLE, FALSE);
}

void disableDepthWrite()
{
	//g_immediateContex->SetRenderState(D3DRS_ZWRITEENABLE, FALSE);
}

void blendFunction(D3D11_BLEND  src, D3D11_BLEND dst)
{
    //g_immediateContex->OMSetBlendState();
    //g_immediateContex->SetRenderState(D3DRS_SRCBLEND, src);
    //g_immediateContex->SetRenderState(D3DRS_DESTBLEND, dst);
}

void enableAlphaBlending( bool premultiplied )
{
	//g_immediateContex->SetRenderState(D3DRS_ALPHABLENDENABLE, TRUE);
	if( ! premultiplied )
    {
        blendFunction(D3D11_BLEND_SRC_ALPHA, D3D11_BLEND_INV_SRC_ALPHA);
    }
	else
    {
		blendFunction(D3D11_BLEND_ONE , D3D11_BLEND_INV_SRC_ALPHA );
    }
}

void disableAlphaBlending()
{
	//g_immediateContex->SetRenderState(D3DRS_ALPHABLENDENABLE, FALSE);
}

void enableAdditiveBlending()
{
	//g_immediateContex->SetRenderState(D3DRS_ALPHABLENDENABLE, TRUE);
    blendFunction(D3D11_BLEND_SRC_ALPHA, D3D11_BLEND_ONE );
}

void enableAlphaTest( DWORD value, D3D11_COMPARISON_FUNC func )
{
	//g_immediateContex->SetRenderState(D3DRS_ALPHATESTENABLE, TRUE);
    //g_immediateContex->SetRenderState(D3DRS_ALPHAREF, value);
    //g_immediateContex->SetRenderState(D3DRS_ALPHAFUNC, func);
}

void disableAlphaTest()
{
	//g_immediateContex->SetRenderState(D3DRS_ALPHATESTENABLE, FALSE);
}

} } // namespace cinder::dx11