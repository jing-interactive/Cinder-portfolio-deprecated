// Include header
#include "../include/ciTri.h"

// Convert point list into delaunay triangles
vector<TriangleData> ciTri::triangulate(const vector<ci::Vec2f> & points, float resolution)
{

	// Initialize output list
    vector<TriangleData> mTriangleDatas;

	// Convert list of points
	vector<ci::Vec2f> mPoints = points;
    float mSize = mPoints.size();
    float mCount = math<float>::min(resolution, mSize);
    Delaunay::Point mPoint;
    vector<Delaunay::Point> mVertices;
    for (int32_t i = 0; i< mCount; i++)
    {
        int32_t mId = (int32_t)((float)i / mCount * mSize);
		mPoint[0] = mPoints[mId].x;
        mPoint[1] = mPoints[mId].y;
        mVertices.push_back(mPoint);
    }

	// Triangulate points
	Delaunay mDelaunay(mVertices);
	mDelaunay.Triangulate();

	// Iterate through triangles
    for (Delaunay::fIterator mTriIt = mDelaunay.fbegin(); mTriIt != mDelaunay.fend(); ++mTriIt)
    {

		// Get point indexes
        int32_t mA = mDelaunay.Org(mTriIt);
        int32_t mB = mDelaunay.Dest(mTriIt);
        int32_t mC = mDelaunay.Apex(mTriIt);
        int32_t mAId = (int32_t)(((float)mA / resolution) * mSize);
        int32_t mBId = (int32_t)(((float)mB / resolution) * mSize);
        int32_t mCId = (int32_t)(((float)mC / resolution) * mSize);

		// Set positions in triangles
        ci::Vec2f mTriangleData[3];
        mTriangleData[0] = ci::Vec2f(mPoints[mAId].x, mPoints[mAId].y);
        mTriangleData[1] = ci::Vec2f(mPoints[mBId].x, mPoints[mBId].y);
        mTriangleData[2] = ci::Vec2f(mPoints[mCId].x, mPoints[mCId].y);

		// Find center of triangle
		ci::Vec2f mCentroid = ci::Vec2f(
			(mTriangleData[0].x + mTriangleData[1].x + mTriangleData[2].x) / 3.0f, 
			(mTriangleData[0].y + mTriangleData[1].y + mTriangleData[2].y) / 3.0f
		);

		// Initialize properties to test triangle position
		int32_t mCounter = 0;
		ci::Vec2f mPointA = mPoints[0];
		ci::Vec2f mPointB;

		// Iterate through points
		for (int32_t i = 1; i < (int32_t)mSize; i++)
		{

			// Get test point
			mPointB = mPoints[i];

			// Compare centroid of this triangle to the previous one
			if (mCentroid.y > math<float>::min(mPointA.y, mPointB.y) && 
				mCentroid.y <= math<float>::max(mPointA.y, mPointB.y) && 
				mCentroid.x <= math<float>::max(mPointA.x, mPointB.x) && 
				mPointA.y != mPointB.y && 
				(mPointA.x == mPointB.x || mCentroid.x <= (mCentroid.y - mPointA.y) * (mPointB.x - mPointA.x) / (mPointB.y - mPointA.y) + mPointA.x))
				mCounter++;

			// Assign this point to last
			mPointA = mPointB;

		}

		// Only include triangles which are inside shape
        if (mCounter % 2 != 0)
        {

			// Set triangle data
            TriangleData mTriData;
            mTriData.a = ci::Vec2f(mTriangleData[0].x, mTriangleData[0].y);
            mTriData.b = ci::Vec2f(mTriangleData[1].x, mTriangleData[1].y);
            mTriData.c = ci::Vec2f(mTriangleData[2].x, mTriangleData[2].y);
            mTriData.area = mDelaunay.area(mTriIt);
			mTriData.centroid = mCentroid;

			// Add triangle to list
            mTriangleDatas.push_back(mTriData);

        }

    }

	// Return triangles
	return mTriangleDatas;

}
