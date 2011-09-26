#ifndef CI_TriangleData
#define CI_TriangleData

// Includes
#include "cinder/Cinder.h"
#include "cinder/Shape2d.h"
#include "cinder/Vector.h"
#include "lib/del_interface.hpp"

// Imports
using namespace ci;
using namespace tpp;
using namespace std;

// TriangleData data
typedef struct
{

	// Points
	ci::Vec2f a;
	ci::Vec2f b;
	ci::Vec2f c;
	
	// Properties
	int32_t id;
	float area;
	ci::Vec2f centroid;
	ci::Vec2f prevCentroid;


} TriangleData;

class ciTri
{

public:

	// Triangulate a shape
	static vector<TriangleData> triangulate(const vector<ci::Vec2f> & points, float resolution = 50.0f);

};

#endif
