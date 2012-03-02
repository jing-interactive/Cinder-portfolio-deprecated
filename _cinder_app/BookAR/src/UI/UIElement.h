#ifndef UIELEMENT_H
#define UIELEMENT_H

#include "cinder/Area.h"
#include "cinder/gl/Texture.h"
#include "cinder/app/AppBasic.h"
#include "cinder/Function.h"

using namespace ci;
using namespace ci::app;

class UIElement
{
public:
	enum State{
		NORMAL,
		OVER,
		CLICK
	};
	UIElement(int id, int x, int y, int width, int height, gl::Texture tex = gl::Texture());
	bool isPointIn( const Vec2f &pt );
	bool mouseMove( MouseEvent event );
	bool mouseDown( MouseEvent event );
	void draw();
	State getState(){return _state;}
	int getId(){return _id;}
// 	//! Registers a callback for button events. Returns a unique identifier.
// 	CallbackId		registerAction( std::function<bool (UIElement)> callback ) { return _mgr.registerCb( callback ); }
// 	//! Registers a callback for mouseDown events. Returns a unique identifier.
// 	template<typename T>
// 	CallbackId		registerAction( T *obj, bool (T::*callback)(UIElement) ) { return _mgr.registerCb( std::bind1st( std::mem_fun( callback ), obj ) ); }
private:
	int			_id;
	State		_state;
	Vec2f		_pos;
	gl::Texture _tex;
	Area		_area;
//	CallbackMgr<UIElement> _mgr;
};

#endif //UIELEMENT_H