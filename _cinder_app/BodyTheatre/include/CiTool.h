#include "cinder/Vector.h"

namespace cinder{

	Vec3f screenToWorld( const Vec2i &mouse, float z = 0 );

	Vec3f unproject( const Vec3f &pt );

}