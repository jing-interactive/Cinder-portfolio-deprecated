#pragma once

#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/GlslProg.h"
#include "cinder/gl/Fbo.h"
#include "cinder/Timeline.h"
#include "cinder/params/Params.h"

#include "DataObejcts.h"

#include <vector>

static const float kMaxColorAreaLifetime = 1.0f;

class GeometryWarsApp : public ci::app::AppBasic {
public:
	void setup();
	void mouseDown( ci::app::MouseEvent event );
	void mouseMove( ci::app::MouseEvent event );
	void prepareSettings(ci::app::AppBasic::Settings *settings);
	void update();
	void draw();
    void keyDown( ci::app::KeyEvent event);
    void keyUp( ci::app::KeyEvent event);
	
private:
	void applyBlur( ci::gl::Texture& texture, ci::gl::Fbo& intoFbo, float spread, float size, ci::Vec2f axis );
	void showExplosion( ci::Vec2f position );
	void drawCharacters( ci::ColorA colorModifier = ci::ColorA(0, 0, 0, 0 ) );
	
	float mPreviousTime;
	
	// Frame buffer objects
	ci::gl::Fbo mGridFbo;
	ci::gl::Fbo mColorMaskFbo;
	ci::gl::Fbo mColorMaskFbo1;
	ci::gl::Fbo mColorMaskFbo2;
	ci::gl::Fbo mDistortionMaskFbo;
	ci::gl::Fbo mDistortionMaskFbo2;
	ci::gl::Fbo mCharacterGlowFbo;
	ci::gl::Fbo mCharacterGlowFbo1;
	ci::gl::Fbo mCharacterGlowFbo2;
	
	// Shaders
	ci::gl::GlslProg mGridShader;
	ci::gl::GlslProg mBlurShader;
	
	// Grid
	ci::Vec2f mGridUnitSize;
	ci::Vec2i mGridSize;
	
	// Contains glowing areas that reveal grid beneath
	std::vector<ColorMaskArea*> mColorMaskAreas;
	
	// Entities that generate color mask areas
	std::vector<Character*> mCharacters;
	
	// Character that follows the mouse in this demo
	Character* mPlayer;
	ci::Vec3f mMouseTarget;

    ci::params::InterfaceGl mParams;
};
