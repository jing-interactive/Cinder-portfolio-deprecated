#ifdef USING_ARTK
#include <ARToolKitPlus/TrackerSingleMarker.h>
#endif
#include "BookAR.h"
#include "ui/UIElement.h"
#include "UI/UIDef.h"
#include <boost/foreach.hpp>

void BookAR::updateData(const ci::Surface32f& image, gl::VboMesh& mesh, float max_height)
{
	uint32_t book_w = image.getWidth();
	uint32_t book_h = image.getHeight();

	if (!mesh || mesh.getNumVertices() != book_w * book_h)
	{//needs refresh
		gl::VboMesh::Layout layout;
		layout.setDynamicColorsRGB();
		layout.setDynamicPositions();
		_mesh_book = gl::VboMesh( book_w * book_h, 0, layout, GL_POINTS );
	}
	Surface32f::ConstIter pixelIter = image.getIter();
	gl::VboMesh::VertexIter vertexIter( mesh );

	while( pixelIter.line() ) {
		while( pixelIter.pixel() ) {
			Color color( pixelIter.r(), pixelIter.g(), pixelIter.b() );
			float height = color.dot( Color( 0.3333f, 0.3333f, 0.3333f ) );

			// the x and the z coordinates correspond to the pixel's x & y
			float x = pixelIter.x() - book_h / 2.0f;
			float z = pixelIter.y() - book_h / 2.0f;

			vertexIter.setPosition( x, height * max_height, z );
			vertexIter.setColorRGB( color );
			++vertexIter;
		}
	}
}

namespace
{
	enum
	{//helper
		OBSERVE = 0,
		CREATE = 1,
		CAMERA = 2,
		SHARE = 3,
		FRIENDS = 4,
	};
}


void BookAR::update()
{
	StateMachine<BookAR>::update();

	BOOST_FOREACH(shared_ptr<UIElement> e, _buttons)
	{
		if (e->getState() == UIElement::CLICK)
		{
			int id = e->getId()-BUTTON_BASE;
			switch (id)
			{
			case OBSERVE:
				break;
			case CREATE:
				changeToState(_state_creating);
				break;
			case CAMERA:
				changeToState(_state_tracking);
				break;
			case SHARE:
				changeToState(_state_sharing);
				break;
			case FRIENDS:
				break;
			default:
				break;
			}
			break;
		}
	}
	BOOST_FOREACH(shared_ptr<UIElement> e, _thumbs)
	{
		if (e->getState() == UIElement::CLICK)
		{
			int id = e->getId()-THUMB_BASE;
			//TODO: set current model to create
			break;
		}
	}
}