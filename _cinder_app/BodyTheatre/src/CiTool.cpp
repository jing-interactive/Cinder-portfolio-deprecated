#include "CiTool.h"
#include "cinder/Area.h"
#include "cinder/gl/gl.h"

namespace cinder{

	Vec3f screenToWorld( const Vec2i &mouse, float z )
	{
		Vec3f p = Vec3f(mouse, z);

		/* adjust y (0,0 is lowerleft corner in OpenGL) */ 
		Area viewport = gl::getViewport();
		p.y = (viewport.getHeight() - p.y);

		/* near plane intersection */
		p.z = 0.0f;
		Vec3f p0 = unproject(p);

		/* far plane intersection */
		p.z = 1.0f;
		Vec3f p1 = unproject(p);

		/* find (x, y) coordinates */
		float t = (z - p0.z) / (p1.z - p0.z);
		p.x = (p0.x + t * (p1.x - p0.x));
		p.y = (p0.y + t * (p1.y - p0.y));
		p.z = z;

		return p;
	}

	Vec3f unproject( const Vec3f &pt )
	{
		/* find the inverse modelview-projection-matrix */
		Matrix44f a = gl::getProjection() * gl::getModelView();
		a.invert();

		/* transform to normalized coordinates in the range [-1, 1] */
		Area viewport = gl::getViewport();
		Vec4f in;
		in.x = (pt.x - viewport.getX1())/viewport.getWidth()*2.0f-1.0f;
		in.y = (pt.y - viewport.getY1())/viewport.getHeight()*2.0f-1.0f;
		in.z = 2.0f * pt.z - 1.0f;
		in.w = 1.0f;

		/* find the object's coordinates */
		Vec4f out = a * in;
		if(out.w != 0.0f) out.w = 1.0f / out.w;

		/* calculate output */
		Vec3f result;
		result.x = out.x * out.w;
		result.y = out.y * out.w;
		result.z = out.z * out.w;

		return result;
	}
}