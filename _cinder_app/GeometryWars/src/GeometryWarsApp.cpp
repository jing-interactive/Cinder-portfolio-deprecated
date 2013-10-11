#include "GeometryWarsApp.h"
#include "../../../_common/MiniConfig.h"

#include "cinder/Rand.h"

#include <algorithm>

using namespace ci;
using namespace ci::app;
using namespace std;

void GeometryWarsApp::prepareSettings(Settings *settings)
{
    readConfig();

    settings->setWindowSize(WIN_WIDTH, WIN_HEIGHT);
}

void GeometryWarsApp::setup()
{
	mPreviousTime			= 0.0f;
	
	gl::Fbo::Format format;
	format.enableMipmapping();
	format.setMagFilter( GL_LINEAR_MIPMAP_LINEAR );
	mGridFbo				= gl::Fbo( getWindowWidth(), getWindowHeight(), format );
	mColorMaskFbo			= gl::Fbo( getWindowWidth(), getWindowHeight(), format );
	mColorMaskFbo1			= gl::Fbo( getWindowWidth(), getWindowHeight(), format );
	mColorMaskFbo2			= gl::Fbo( getWindowWidth(), getWindowHeight(), format );
	mDistortionMaskFbo		= gl::Fbo( getWindowWidth(), getWindowHeight(), format );
	mDistortionMaskFbo2		= gl::Fbo( getWindowWidth(), getWindowHeight(), format );
	mCharacterGlowFbo		= gl::Fbo( getWindowWidth(), getWindowHeight(), format );
	mCharacterGlowFbo1		= gl::Fbo( getWindowWidth(), getWindowHeight(), format );
	mCharacterGlowFbo2		= gl::Fbo( getWindowWidth(), getWindowHeight(), format );
	
	mGridSize				= Vec2i( 40, 30 );
	mGridUnitSize			= Vec2f( getWindowSize().x / (float) mGridSize.x, getWindowSize().y / (float) mGridSize.y );
	
	mGridShader				= gl::GlslProg( loadAsset( "passthru.vert" ), loadAsset( "colorgrid.frag" ) );
	mBlurShader				= gl::GlslProg( loadAsset( "passthru.vert" ), loadAsset( "blur.frag" ) );
	
	// Create some characters with random positions, values, colors
	for ( int i = 0; i < 10; i++ ) {
		Character* character = new Character();
		character->position = Vec3f( randFloat() * getWindowWidth(), randFloat() * getWindowHeight(), 0.0f );
		character->velocity = randVec3f() * ( 40.0f + randFloat() * 50.0f );
		character->velocity.z = 0.0f;
		character->color = ColorA( randFloat(),  randFloat(),  randFloat(), 1.0f );
		mCharacters.push_back( character );
	}
	
	mPlayer = new Character();
	mPlayer->color = ColorA( 1, 0, 0, 1.0f );
	mCharacters.push_back( mPlayer );

    mParams = params::InterfaceGl("params", Vec2i(300, getConfigUIHeight()));
    setupConfigUI(&mParams);
}

void GeometryWarsApp::mouseDown( MouseEvent event )
{
	// Call a quick, staggered series of ColorMaskAreas generated with special properties
	for ( int i = 0; i < 3; i++ )
    {
		timeline().add( bind( &GeometryWarsApp::showExplosion, this, event.getPos() ), timeline().getCurrentTime() + 0.15f * (float) i );
    }
}

void GeometryWarsApp::showExplosion( Vec2f position )
{
	// ColorMaskArea generated with special properties to be larger and last longer
	ColorMaskArea* maskArea = new ColorMaskArea();
	maskArea->position = position + randVec2f() * 70.0f;
	maskArea->growthRate = 500.0f;
	maskArea->maxLifetime = 2.0f;
	maskArea->color = ColorA(1,.5,0,1);
	mColorMaskAreas.push_back( maskArea );
}

void GeometryWarsApp::mouseMove( MouseEvent event )
{
	mMouseTarget = Vec3f( event.getPos(), 0.0f );
}

void GeometryWarsApp::update()
{
	// Get the timestep for each update
	float currentTime = getElapsedSeconds();
	float deltaTime = currentTime - mPreviousTime;
	mPreviousTime = currentTime;
	
	// Update color mask areas
	std::vector<ColorMaskArea*> toRemove;
	std::vector<ColorMaskArea*>::iterator iter;
	for ( iter = mColorMaskAreas.begin(); iter != mColorMaskAreas.end(); iter++ ) {
		(*iter)->update( deltaTime );
		// If the lifetime exceeds the max lifetime, we queue for removal
		if ( (*iter)->isExpired() ) toRemove.push_back( *iter );
	}
	// Iterate through the removal queue, deleting all elements and removing from mColorMaskAreas vector
	for ( iter = toRemove.begin(); iter != toRemove.end(); iter++ ) {
		std::vector<ColorMaskArea*>::iterator match = find( mColorMaskAreas.begin(), mColorMaskAreas.end(), *iter );
		if ( match != mColorMaskAreas.end() ) {
			delete *match;
			mColorMaskAreas.erase( match );
		}
	}
	
	// Update characters
	std::vector<Character*>::iterator c_iter;
	for ( c_iter = mCharacters.begin(); c_iter != mCharacters.end(); c_iter++ ) {
		(*c_iter)->update( deltaTime );
		
		// If character leaves window area, reset position randomly
		if ( !Rectf( getWindowBounds() ).contains( (*c_iter)->position.xy() ) )
			(*c_iter)->position = Vec3f( randFloat() * getWindowWidth(), randFloat() * getWindowHeight(), 0.0f );
		
		// Generate color mask areas, which are data objects used for rendering characteristic grid effect
		ColorMaskArea* maskArea = new ColorMaskArea();
		maskArea->position = (*c_iter)->position.xy();
		maskArea->growthRate = 100.0f;
		maskArea->maxLifetime = kMaxColorAreaLifetime;
		maskArea->color = (*c_iter)->color;
		mColorMaskAreas.push_back( maskArea );
	}
	
	// Player trakcs mouse movement
	if ( mPlayer->position.distance( mMouseTarget ) > 10.0f )
		mPlayer->velocity = (mMouseTarget - mPlayer->position).normalized() * 200.0f;
	else
		mPlayer->velocity = Vec3f::zero();
}

void GeometryWarsApp::applyBlur( gl::Texture& texture, gl::Fbo& intoFbo, float spread, float size, ci::Vec2f axis )
{
	intoFbo.bindFramebuffer();
	gl::clear( ColorA( 0, 0, 0, 0 ) );
	mBlurShader.bind();
	texture.bind( 0 );
	mBlurShader.uniform( "texture", 0 );
	mBlurShader.uniform( "spread", spread );
	mBlurShader.uniform( "size", size );
	mBlurShader.uniform( "axis", axis );
	gl::drawSolidRect( Rectf( getWindowBounds()));
	texture.unbind(0);
	mBlurShader.unbind();
	intoFbo.unbindFramebuffer();
}

void GeometryWarsApp::draw()
{
	// Clear, set the window
	gl::clear( Color( 0, 0, 0 ) );
	gl::setMatricesWindow( getWindowSize(), true );
	
	// Blending
	gl::enableAlphaBlending();
	glEnable( GL_BLEND );
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	
	// Draw the grid into a FBO
	Vec2i wSize = getWindowSize();
	mGridFbo.bindFramebuffer();
	gl::clear( ColorA( 0, 0, 0, 0 ) );
	for ( int i = 0; i < mGridSize.x; i++) {
		float alpha = i % 2 ? 0.5f : 1.0f; // make every other line half opacity
		Vec2f start = Vec2f( 0.0f, (float)i * mGridUnitSize.x );
		Vec2f end = Vec2f( wSize.x, (float)i * mGridUnitSize.x );
		gl::color( 1, 1, 1, alpha );
		gl::drawLine( start, end );
	}
	for ( int i = 0; i < mGridSize.x; i++) {
		float alpha = i % 2 ? 0.5f : 1.0f; // make every other line half opacity
		Vec2f start = Vec2f((float)i * mGridUnitSize.y, 0.0f );
		Vec2f end = Vec2f( (float)i * mGridUnitSize.y, wSize.y );
		gl::color( 1, 1, 1, alpha );
		gl::drawLine( start, end );
	}
	mGridFbo.unbindFramebuffer();
	
	// An iterator to use in future operations
	std::vector<ColorMaskArea*>::iterator iter;
	
	// Draw the color mask areas into the color mask buffer, which is used for revealing and tinting the grid behind them
	mColorMaskFbo.bindFramebuffer();
	gl::clear( ColorA( 0, 0, 0, 0 ) );
	for ( iter = mColorMaskAreas.begin(); iter != mColorMaskAreas.end(); iter++ ) {
		gl::color( (*iter)->color );
		gl::drawSolidCircle( (*iter)->position, (*iter)->size );
	}
	mColorMaskFbo.unbindFramebuffer();
	
	// The distortion mask is a single-color (Red) map similar to the color mask,
	// except that it will be blurerd and used for the pixel shader that does the distortion effect
	gl::setMatricesWindow( getWindowSize(), false );
	
	// Apply a blur using a pair of these method calls
	applyBlur( mColorMaskFbo.getTexture(),		mColorMaskFbo1,			0.073f,	0.008f,		Vec2f( 0.0f, 1.0f ) );
	applyBlur( mColorMaskFbo1.getTexture(),		mColorMaskFbo2,			0.073f,	0.008f,		Vec2f( 1.0f, 0.0f ) );
	
	// To see the color mask and its blur
	//gl::draw( mColorMaskFbo2.getTexture() ); drawCharacters(); return;
	
	// Apply a blur using a pair of these method calls
	applyBlur( mColorMaskFbo.getTexture(),		mDistortionMaskFbo,		0.06f,	0.008f,		Vec2f( 0.0f, 1.0f ) );
	applyBlur( mDistortionMaskFbo.getTexture(),	mDistortionMaskFbo2,	0.06f,	0.008f,		Vec2f( 1.0f, 0.0f ) );
	
	// If you want to see the distortion mask, do this
	//gl::draw( mDistortionMaskFbo2.getTexture() ); return;

	mGridShader.bind();
	mGridFbo.bindTexture( 0 );
	mColorMaskFbo2.bindTexture( 1 );
	mDistortionMaskFbo.bindTexture( 2 );
	mGridShader.uniform( "time",					mPreviousTime );
	mGridShader.uniform( "gridTexture",			0 );
	mGridShader.uniform( "colorMaskTexture",		1 );
	mGridShader.uniform( "distortionMaskTexture",	2 );
	mGridShader.uniform( "vanishingPoint",			Vec2f( 0.5, 0.5 ) );
	mGridShader.uniform( "focalLength",			10.0f );
	mGridShader.uniform( "depthMultiplier",		2.2f );
	gl::drawSolidRect( Rectf( getWindowBounds() ) );
	mGridFbo.unbindTexture();
	mColorMaskFbo2.unbindTexture();
	mDistortionMaskFbo.unbindTexture();
	mGridShader.unbind();
	
	// Draw the characters
	mCharacterGlowFbo.bindFramebuffer();
	gl::color( 1, 1, 1, 1 );
	gl::clear( ColorA( 0, 0, 0, 0 ) );
	drawCharacters();
	mCharacterGlowFbo.unbindFramebuffer();
	
	// Apply a blur using a pair of these method calls
	//applyBlur( mCharacterGlowFbo.getTexture(),	mCharacterGlowFbo1,		0.06f,	0.008f,		Vec2f( 0.0f, 1.0f ) );
	//applyBlur( mCharacterGlowFbo1.getTexture(),	mCharacterGlowFbo2,		0.06f,	0.008f,		Vec2f( 1.0f, 0.0f ) );
	
	//gl::setMatricesWindow( getWindowSize(), false );
	//gl::draw( mCharacterGlowFbo.getTexture() );
	//drawCharacters( ColorA( .3, .3, .3, 0 ) );
	
	drawCharacters();
	
	// Draw borders around the screen, as in the real game
	int m = 10;
	gl::color( 0, 1, 1, 1 );
	gl::drawStrokedRect( Rectf( m, m, getWindowWidth() - m, getWindowHeight() - m ) );
	m = 14;
	gl::drawStrokedRect( Rectf( m, m, getWindowWidth() - m, getWindowHeight() - m ) );

    mParams.draw();
}

void GeometryWarsApp::drawCharacters( ci::ColorA colorModifier )
{
	// Reset the matrices to draw characters as 3d shadpes
	gl::setMatricesWindowPersp( app::getWindowSize(), 20.0f, -100.0f, 300.0f, false );
	
	std::vector<Character*>::iterator c_iter;
	for ( c_iter = mCharacters.begin(); c_iter != mCharacters.end(); c_iter++ ) {
		gl::color( (*c_iter)->color + colorModifier );
		glPushMatrix();
		gl::translate( (*c_iter)->position );
		gl::rotate( (*c_iter)->rotation );
		gl::scale( (*c_iter)->scale );
		gl::drawStrokedCube( Vec3f::zero(), Vec3f( 10, 10, 10 ) );
		glPopMatrix();
	}
}

void GeometryWarsApp::keyDown( KeyEvent event )
{

}

void GeometryWarsApp::keyUp( KeyEvent event )
{
    if (event.getCode() == KeyEvent::KEY_ESCAPE)
        quit();
}



CINDER_APP_BASIC( GeometryWarsApp, RendererGl )
