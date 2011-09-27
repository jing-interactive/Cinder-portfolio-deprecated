/*
* vTrackedBlob.h
* openFrameworks
*
* This class represents a blob with inter-frame information.
* This includes a persistent id to assume a persistent identity over
* time.
*
*/

#pragma once

#include "OpenCV.h"
#include "point2d.h"

using cv::Rect;
using cv::Point;
using cv::RotatedRect;
using cv::Point2f;

struct vBlob
{
	vBlob()
	{
		area = 0;
		angle  = 0;
		length = 0;
		isHole = false;
	}

	vBlob(const vBlob& b):box(b.box),center(b.center),pts(b.pts),rotBox(b.rotBox)
	{
	//	box = b.box;
	//	center = b.center;
		area = b.area;
		angle = b.angle;
		isHole = b.isHole;
		length = b.length;
	//	pts = b.pts;
	}

	vBlob(Rect rc, Point ct, float _area = 0, float _angle = 0, bool hole = false)
	{
		box = rc;
		center = ct;
		area = _area;
		angle = _angle;
		isHole = hole;
		length = 0;
	}
	
	Rect box;
	RotatedRect rotBox;
	float angle;

	Point2f center;

	bool operator<(const vBlob& other) const 
	{//sorted by Y-coord first then X-coord
		return (center.y < other.center.y) || 
			( (center.y == other.center.y) && (center.x < other.center.x)); 
	}

	bool similar(const vBlob& other, int k) const 
	{
		return (abs(box.width - other.box.width) < k) && (abs(box.height - other.box.height)<k);
	}

	void boxMerge(const vBlob& other)
	{
		int _x = cv::min(other.box.x, box.x);
		int _y = cv::min(other.box.y, box.y);
		box.width = cv::max(other.box.x + other.box.width, box.x + box.width)- _x;
		box.height = cv::max(other.box.y + other.box.height, box.y + box.height) - _y;

		box.x = _x;
		box.y = _y;

		center.x = box.x + box.width/2;
		center.y = box.y + box.height/2;		
	}

	float area;
	float length;
	vector<Point> pts;
	bool isHole;
};

enum E_status
{
	statusStill,
	statusEnter,
	statusLeave,
	statusMove,	
};

struct vTrackedBlob : public vBlob 
{
	enum
	{
		BLOB_UN_NAMED = -3,
		BLOB_TO_DELETE = -2,		
	};

	E_status status;
	int id;
	Point2f velocity;

    // Used only by BlobTracker
    //
    bool markedForDeletion;
    int framesLeft;

    vector<float> distance;
    vector<int> neighbors;  // ids of the closest points, sorted

	vTrackedBlob():vBlob() {
        id = BLOB_UN_NAMED;
		status = statusStill;
        markedForDeletion = false;
        framesLeft = 0;
    }

	vTrackedBlob( const vBlob& b ):vBlob(b) {
        area = b.area;
   //     box = b.box;
     //   center = b.center;
        isHole = b.isHole;
       // pts = b.pts;

        id = BLOB_UN_NAMED;
		status = statusStill;
        markedForDeletion = false;
        framesLeft = 0;
    }

	std::string getStatusString()
	{
		if (status == statusStill)
			return "still";
		else 	if (status == statusEnter)
			return "enter";
		else if (status == statusLeave)
			return "leave";
		else if (status == statusMove)
			return "move";
		else return "";
	}

    int getMinDistance() {
        int best=-1;
        float best_v=99999.0f;
        for( unsigned int i=0; i<distance.size(); i++ ) {
            if( distance[i] < best_v ) {
                best = i;
                best_v = distance[i];
            }
        }
        return best;
    }
};