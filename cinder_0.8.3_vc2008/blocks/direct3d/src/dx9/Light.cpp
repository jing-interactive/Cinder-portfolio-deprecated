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
 NEdx9IGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
*/

#include "dx9/Light.h"

namespace cinder {
namespace dx9 {

void Light::setAttenuation( float aConstantAttenuation, float aLinearAttenuation, float aQuadraticAttenuation )
{
	mConstantAttenuation = aConstantAttenuation;
	mLinearAttenuation = aLinearAttenuation;
	mQuadraticAttenuation = aQuadraticAttenuation;
}

void Light::setAmbient( const Color &aAmbient )
{
	mAmbient = aAmbient;
    getDevice()->SetRenderState(D3DRS_AMBIENT, D3DCOLOR_COLORVALUE(aAmbient.r, aAmbient.g, aAmbient.b, 1.0f);
}

void Light::setDiffuse( const Color &aDiffuse )
{
	mDiffuse = aDiffuse;

    D3DLIGHT9 light;  
    getDevice()->GetLight( mID, &light );
    light.Diffuse.r = mDiffuse.r;
    light.Diffuse.g = mDiffuse.r;
    light.Diffuse.b = mDiffuse.b;
    light.Diffuse.a = 1.0f;
    getDevice()->SetLight( mID, &light );
}	

void Light::setSpecular( const Color &aSpecular )
{
	mSpecular = aSpecular;

    D3DLIGHT9 light;  
    getDevice()->GetLight( mID, &light );
    light.Specular.r = aSpecular.r;
    light.Specular.g = aSpecular.r;
    light.Specular.b = aSpecular.b;
    light.Specular.a = 1.0f;
    getDevice()->SetLight( mID, &light );
}

void Light::lookAt( const Vec3f &eye, const Vec3f &target )
{
	setPosition( eye );
	setDirection( ( target - eye ).normalized() );
}

void Light::setDirection( const Vec3f &aDirection )
{
	mDirection = aDirection;

	if( mType == DIRECTIONAL )
    {
        D3DLIGHT9 light;  
        getDevice()->GetLight( mID, &light );
        light.Direction.x = aDirection.x;
        light.Direction.y = aDirection.y;
        light.Direction.z = aDirection.z;
    }
	mShadowCam.lookAt( mPosition, mPosition + mDirection );
}

void Light::setPosition( const Vec3f &aPosition )
{
	mPosition = aPosition;
	if( mType != DIRECTIONAL )
    {
        D3DLIGHT9 light;  
        getDevice()->GetLight( mID, &light );
        light.Position.x = aPosition.x;
        light.Position.y = aPosition.y;
        light.Position.z = aPosition.z;
    }
	mShadowCam.lookAt( mPosition, mPosition + mDirection );
}

void Light::setConstantAttenuation( float aConstantAttenuation )
{
	mConstantAttenuation = aConstantAttenuation;
	dx9Lightf( dx9_LIGHT0 + mID, dx9_CONSTANT_ATTENUATION, mConstantAttenuation );
}

void Light::setLinearAttenuation( float aLinearAttenuation )
{
	mLinearAttenuation = aLinearAttenuation;
	dx9Lightf( dx9_LIGHT0 + mID, dx9_LINEAR_ATTENUATION, mLinearAttenuation );	
}

void Light::setQuadraticAttenuation( float aQuadraticAttenuation )
{
	mQuadraticAttenuation = aQuadraticAttenuation;
	dx9Lightf( dx9_LIGHT0 + mID, dx9_QUADRATIC_ATTENUATION, mQuadraticAttenuation );
}

void Light::setSpotExponent( float aSpotExponent )
{
	mSpotExponent = aSpotExponent;
	dx9Lightf( dx9_LIGHT0 + mID, dx9_SPOT_EXPONENT, mSpotExponent );	
}

void Light::setSpotCutoff( float aSpotCutoff )
{
	mSpotCutoff = aSpotCutoff;
	dx9Lightf( dx9_LIGHT0 + mID, dx9_SPOT_CUTOFF, mSpotCutoff );
}

void Light::enable()
{
	if( mEnabled )
		return;
	
	dx9Enable( dx9_LIGHT0 + mID );
	dx9Lightfv( dx9_LIGHT0 + mID, dx9_AMBIENT, mAmbient );
	dx9Lightfv( dx9_LIGHT0 + mID, dx9_DIFFUSE, mDiffuse );	
	dx9Lightfv( dx9_LIGHT0 + mID, dx9_SPECULAR, mSpecular );
	if( mType == DIRECTIONAL )
		dx9Lightfv( dx9_LIGHT0 + mID, dx9_POSITION, Vec4f( mDirection.x, mDirection.y, mDirection.z, 0.0f ).ptr() );
	else
		dx9Lightfv( dx9_LIGHT0 + mID, dx9_POSITION, Vec4f( mPosition.x, mPosition.y, mPosition.z, 1.0f ).ptr() );
	dx9Lightf( dx9_LIGHT0 + mID, dx9_CONSTANT_ATTENUATION, mConstantAttenuation );
	dx9Lightf( dx9_LIGHT0 + mID, dx9_LINEAR_ATTENUATION, mLinearAttenuation );
	dx9Lightf( dx9_LIGHT0 + mID, dx9_QUADRATIC_ATTENUATION, mQuadraticAttenuation );
	dx9Lightfv( dx9_LIGHT0 + mID, dx9_SPOT_DIRECTION, &mDirection.x );
	dx9Lightf( dx9_LIGHT0 + mID, dx9_SPOT_CUTOFF, mSpotCutoff );
	dx9Lightf( dx9_LIGHT0 + mID, dx9_SPOT_EXPONENT, mSpotExponent );
	
	mEnabled = true;
}

void Light::disable()
{
	dx9Disable( dx9_LIGHT0 + mID );
	mEnabled = false;
}

void Light::update( const Camera &relativeCamera ) const
{
	Vec3f relPos;
	
	if( mType == POINT ) {
		Vec3f relPos = relativeCamera.getInverseModelViewMatrix().transformPointAffine( mPosition );
		dx9Lightfv( dx9_LIGHT0 + mID, dx9_POSITION, Vec4f( relPos.x, relPos.y, relPos.z, 1.0f ).ptr() );
	}
	else if( mType == DIRECTIONAL ) {
		Vec3f relDir = relativeCamera.getInverseModelViewMatrix().transformVec( mDirection );
		dx9Lightfv( dx9_LIGHT0 + mID, dx9_POSITION, Vec4f( relDir.x, relDir.y, relDir.z, 0.0f ).ptr() );
	}
	else if( mType == SPOTLIGHT ) {
		Vec3f relPos = relativeCamera.getInverseModelViewMatrix().transformPointAffine( Vec3f( mPosition.x, mPosition.y, mPosition.z ) );
		dx9Lightfv( dx9_LIGHT0 + mID, dx9_POSITION, Vec4f( relPos.x, relPos.y, relPos.z, 1.0f ).ptr() );
		Vec3f relSpotDir = relativeCamera.getInverseModelViewMatrix().transformPointAffine( Vec3f( mDirection.x, mDirection.y, mDirection.z ) );
		dx9Lightfv( dx9_LIGHT0 + mID, dx9_SPOT_DIRECTION, &relSpotDir.x );
	}
}

void Light::setShadowParams( float aShadowFOV, float aShadowNear, float aShadowFar )
{
	mShadowFOV = aShadowFOV;
	mShadowNear = aShadowNear;
	mShadowFar = aShadowFar;
	mShadowCam.setPerspective( mShadowFOV, 1.0f, mShadowNear, mShadowFar );	
}

void Light::setShadowRenderMatrices() const
{
	setMatrices( mShadowCam );
}

Matrix44f Light::getShadowTransformationMatrix( const Camera &camera ) const
{
	//		Matrix44f shadowBias;
	//		shadowBias.scale( Vec3f( 0.5f, 0.5f, 0.5f ) );
	//		shadowBias.translate( Vec3f( 1.0f, 1.0f, 1.0f ) );
	//		dx9MultMatrixf( shadowBias );

	Matrix44f shadowTransMatrix = mShadowCam.getProjectionMatrix();
	shadowTransMatrix *= mShadowCam.getModelViewMatrix();
	shadowTransMatrix *= camera.getInverseModelViewMatrix();
	return shadowTransMatrix;
}

void Light::setDefaults()
{
	if( mType != DIRECTIONAL )
		setPosition( Vec3f::zero() );
	else
		setDirection( Vec3f( 0, 0, 1.0f ) );
	mAmbient = Color( 1.0f, 1.0f, 1.0f );
	mDiffuse = Color( 1.0f, 1.0f, 1.0f );
	mSpecular = Color( 1.0f, 1.0f, 1.0f );
	mSpotExponent = 1.0f;
	if( mType != SPOTLIGHT )
		mSpotCutoff = 180.0f;
	else
		mSpotCutoff = 1.0f;
	mConstantAttenuation = mLinearAttenuation = mQuadraticAttenuation = 1.0f;
}

} // namespace dx9
} // namespace cinder
