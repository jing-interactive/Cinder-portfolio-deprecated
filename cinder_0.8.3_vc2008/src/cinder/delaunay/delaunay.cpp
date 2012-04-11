/*
* 
* Copyright (c) 2012, Ban the Rewind
* All rights reserved.
* 
* Redistribution and use in source and binary forms, with or 
* without modification, are permitted provided that the following 
* conditions are met:
* 
* Redistributions of source code must retain the above copyright 
* notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright 
* notice, this list of conditions and the following disclaimer in 
* the documentation and/or other materials provided with the 
* distribution.
* 
* Neither the name of the Ban the Rewind nor the names of its 
* contributors may be used to endorse or promote products 
* derived from this software without specific prior written 
* permission.
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
* ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
* 
*/

// Include header
#include "cinder/delaunay/delaunay.h"
#include "triangle/del_interface.hpp"

// Imports
using namespace ci;
using namespace ci::app;
using namespace tpp;
using namespace std;

namespace cinder{
// Calculate area of a triangle
float Triangle::calcArea( const Vec2f & a, const Vec2f & b, const Vec2f & c )
{

	// Get length of each side
	float xod( a.x - b.x );
	float yod( a.y - b.y );
	float xda( b.x - c.x );
	float yda( b.y - c.y );

	// Area is half times base times height
	return 0.5f * ( xod * yda - yod * xda );

}

// Calculate centroid from points
Vec2f Triangle::calcCentroid( const Vec2f & a, const Vec2f & b, const Vec2f & c ) {

	// The centroid is one third the sum of the points
	return ( a + b + c ) / 3.0f;

}

// Convert point list into Delaunay triangles
vector<Triangle> Triangle::triangulate( const vector<Vec2f> & aPoints, uint32_t aResolution )
{

	// Initialize output list
	vector<Triangle> triangles;

	// Set resolution to default or specified value
	float resolution = math<float>::max( 3.0f, (float)( aResolution > 0 ? aResolution : aPoints.size() / 2 ) );

	// Convert Cinder points to Delaunay points
    float size = (float)aPoints.size();
    float count = math<float>::min( resolution, size );
    Delaunay::Point position;
    vector<Delaunay::Point> positions;
    for ( float i = 0.0f; i < count; i++ ) {
		Vec2f point = aPoints[ (uint32_t)( i / count * size ) ];
		position[ 0 ] = point.x;
        position[ 1 ] = point.y;
        positions.push_back( position );
    }

	// Triangulate points
	Delaunay delaunay( positions );
	delaunay.Triangulate();

	// Triangle IDs
	int32_t id = 0;

	// Iterate through triangles
    for ( Delaunay::fIterator triIt = delaunay.fbegin(); triIt != delaunay.fend(); ++triIt ) {

		// Get point indexes
        float a = (float)delaunay.Org( triIt );
        float b = (float)delaunay.Dest( triIt );
        float c = (float)delaunay.Apex( triIt );
        uint32_t aId = (int32_t)( ( a / resolution ) * size );
        uint32_t bId = (int32_t)( ( b / resolution ) * size );
        uint32_t cId = (int32_t)( ( c / resolution ) * size );

		// Set positions in triangles
        Vec2f triangle[ 3 ];
        triangle[ 0 ] = aPoints[ aId ];
        triangle[ 1 ] = aPoints[ bId ];
        triangle[ 2 ] = aPoints[ cId ];

		// Find center of triangle
		Vec2f centroid = Vec2f(
			( triangle[ 0 ].x + triangle[ 1 ].x + triangle[ 2 ].x ) / 3.0f, 
			( triangle[ 0 ].y + triangle[ 1 ].y + triangle[ 2 ].y ) / 3.0f
		);

		// Initialize properties to test triangle position
		Vec2f point = aPoints[ 0 ];

		// Iterate through points
		int32_t counter = 0;
		for ( vector<Vec2f>::const_iterator pointIt = aPoints.begin(); pointIt != aPoints.end(); ++pointIt ) {

			// Compare centroid of this triangle to the previous one
			if ( centroid.y >  math<float>::min( point.y, pointIt->y ) && 
				 centroid.y <= math<float>::max( point.y, pointIt->y ) && 
				 centroid.x <= math<float>::max( point.x, pointIt->x ) && 
				 point.y != pointIt->y && 
				 ( point.x == pointIt->x || centroid.x <= ( centroid.y - point.y ) * ( pointIt->x - point.x ) / ( pointIt->y - point.y ) + point.x ) ) {
				counter++;
			}

			// Assign this point to last
			point = * pointIt;

		}

		// Only include triangles which are inside shape
        if ( counter % 2 != 0 ) {

			// Add triangle to list
			Triangle triData( triangle[ 0 ], triangle[ 1 ], triangle[ 2 ], id, (float)delaunay.area( triIt ), centroid );
			triangles.push_back( triData );

			// Increase default ID
			id++;

        }

    }

	// Return triangles
	return triangles;

}

// Constructor
Triangle::Triangle( const Vec2f & origin, const Vec2f & destination, const Vec2f & apex, 
					int32_t id, float area, const Vec2f & centroid ) 
	: mApex( apex ), mArea( area ), mCentroid( centroid ), mDestination( destination ), mId( id ), 
	  mOrigin( origin ), mPrevCentroid( centroid )
{
}

// Hit test triangle
bool Triangle::contains( const Vec2f & position )
{
	
	// Get area values using input position and two points from triangle
	float area0 = calcArea( mDestination, mApex, position );
	float area1 = calcArea( mApex, mOrigin, position );
	float area2 = calcArea( mOrigin, mDestination, position );

	// Sum the absolute values of each area
	float areaSum = math<float>::abs( area0 ) + math<float>::abs( area1 ) + math<float>::abs( area2 );

	// If sum of the three areas is the same as the triangle's 
	// area, the point is inside
	return areaSum == mArea;

}

// Hit test triangle
bool Triangle::contains( const Vec2f & position ) const
{

	// Get area values using input position and two points from triangle
	float area0 = calcArea( mDestination, mApex, position );
	float area1 = calcArea( mApex, mOrigin, position );
	float area2 = calcArea( mOrigin, mDestination, position );

	// Sum the absolute values of each area
	float areaSum = math<float>::abs( area0 ) + math<float>::abs( area1 ) + math<float>::abs( area2 );

	// If sum of the three areas is the same as the triangle's 
	// area, the point is inside
	return areaSum == mArea;

}

// Point getter shortcuts
const Vec2f Triangle::a() 
{ 
	return mOrigin; 
}
const Vec2f Triangle::a() const 
{ 
	return mOrigin; 
}
const Vec2f Triangle::b() 
{ 
	return mDestination; 
}
const Vec2f Triangle::b() const 
{ 
	return mDestination; 
}
const Vec2f Triangle::c() 
{ 
	return mApex; 
}
const Vec2f Triangle::c() const 
{ 
	return mApex; 
}

// Update area
void Triangle::calcArea()
{
	mArea = calcArea( mOrigin, mDestination, mApex );
}

// Getters
const Vec2f Triangle::getApex() 
{ 
	return mApex; 
}
const Vec2f Triangle::getApex() const 
{ 
	return mApex; 
}
const float	Triangle::getArea() 
{ 
	return mArea; 
}
const float Triangle::getArea() const 
{ 
	return mArea; 
}
const Vec2f Triangle::getCentroid() 
{ 
	return mCentroid; 
}
const Vec2f Triangle::getCentroid() const 
{ 
	return mCentroid; 
}
const Vec2f Triangle::getDestination() 
{ 
	return mDestination; 
}
const Vec2f Triangle::getDestination() const 
{ 
	return mDestination; 
}
const int32_t Triangle::getId() 
{ 
	return mId; 
}
const int32_t Triangle::getId() const 
{ 
	return mId; 
}
const Vec2f Triangle::getOrigin() 
{ 
	return mOrigin; 
}
const Vec2f Triangle::getOrigin() const 
{ 
	return mOrigin; 
}
Vec2f Triangle::getVelocity() 
{ 
	return mCentroid - mPrevCentroid; 
}
Vec2f Triangle::getVelocity() const 
{ 
	return mCentroid - mPrevCentroid; 
}

// Move triangle
void Triangle::move( const Vec2f & offset )
{

	// Move all points
	mApex += offset;
	mCentroid += offset;
	mDestination += offset;

	// Move centroid to update velocity
	setCentroid( mCentroid + offset );

}

// Point setter shortcuts
void Triangle::a( const Vec2f & origin )
{
	mOrigin = origin;
}
void Triangle::b( const Vec2f & destination )
{
	mDestination = destination;
}
void Triangle::c( const Vec2f & apex )
{
	mApex = apex;
}

// Setters
void Triangle::setApex( const Vec2f & apex )
{
	mApex = apex;
}
void Triangle::setArea( float area )
{
	mArea = area;
}
void Triangle::setCentroid( const Vec2f & centroid ) 
{ 
	mPrevCentroid = mCentroid;
	mCentroid = centroid;
}
void Triangle::setDestination( const Vec2f & destination )
{
	mDestination = destination;
}
void Triangle::setId( int32_t id )
{
	mId = id;
}
void Triangle::setOrigin( const Vec2f & origin )
{
	mOrigin = origin;
}
void Triangle::setPosition( const ci::Vec2f & position )
{
	
	// Move each point
	mApex = mApex - mCentroid + position;
	mDestination = mDestination - mCentroid + position;
	mOrigin = mOrigin - mCentroid + position;

	// Update centroid
	setCentroid( position );

}
}