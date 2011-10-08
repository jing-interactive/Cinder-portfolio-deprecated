/*
 *  ARTKpMultiTracker.cpp
 *
 *
 *  Copyright (C) 2011 Evan Raskob / pixelpusher <info@pixelist.info>
 *  http://pixelist.info
 *
 * Some code included in this distribution has different copyrights and
 * different licenses.
 *
 * PLEASE NOTE THE LICENSES ON EACH FILE IN THIS PROJECT CAN BE DIFFERENT!
 *
 *  This file is:
 *
 *  *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Affero General Public License as
 *  published by the Free Software Foundation, either version 3 of the
 *  License, or (at your option) any later version.
 
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Affero General Public License for more details.
 
 *  You should have received a copy of the GNU Affero General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
#include "ARTKpMultiTracker.h"
#include "ARToolKitPlus/TrackerSingleMarker.h"
#include "cinder/app/App.h"

ARTKpMultiTracker::ARTKpMultiTracker()
{
	tracker = NULL;
	patt_width = 80; // in millimeters
	patt_center[0] = patt_center[1] = 0;
	markerMode = ARToolKitPlus::MARKER_TEMPLATE;
	screenWidth = cameraWidth = 640.0f;
	screenHeight = cameraHeight = 480.0f; 
}

ARTKpMultiTracker::~ARTKpMultiTracker()
{
	marker_info = NULL;
	if (tracker != NULL)
	{
		delete tracker;
	}
}

void ARTKpMultiTracker::setFullImageProcessing(bool f)
{
	if (tracker)
	{
		if (f)
			tracker->setImageProcessingMode(ARToolKitPlus::IMAGE_FULL_RES);
		else 
			tracker->setImageProcessingMode(ARToolKitPlus::IMAGE_HALF_RES);
	}
}


bool ARTKpMultiTracker::init(const char * cameraParamFilename, const char * markerFilename, int _width, int _height)
{
	float clipNear = 0.1f;
	float clipFar = 5000.0f;
	
	if (tracker != NULL)
	{
		delete tracker;
	}
	
	cameraWidth = _width;
	cameraHeight = _height;
	
	
	// create a tracker that does:
    //  - 16x16 sized marker images
    //  - samples at a maximum of 16x16
    //  - works with luminance (gray) images
    //  - can load a maximum of 4 patterns
    //  - can detect a maximum of 4 patterns in one image
	//	int __PATTERN_SIZE_X, int __PATTERN_SIZE_Y, int __PATTERN_SAMPLE_NUM, int __MAX_LOAD_PATTERNS, int __MAX_IMAGE_PATTERNS>
	
	if (markerMode == ARToolKitPlus::MARKER_TEMPLATE)
		tracker = new ARToolKitPlus::TrackerSingleMarker(cameraWidth, cameraHeight, 16, 16, 16, 8, 16);
	else
		tracker = new ARToolKitPlus::TrackerSingleMarker(cameraWidth, cameraHeight, 12, 12, 12, 8, 32);	
		
	// luminance (grayscale) images
	tracker->setPixelFormat(ARToolKitPlus::PIXEL_FORMAT_LUM);
	//	setFullImageProcessing(true);
	
	if (!tracker->init(cameraParamFilename, clipNear, clipFar))
	{
		ci::app::App::get()->console()<<"ERROR: init() failed\n"<<std::endl;
		delete tracker;
		return false;
	}
	
	if (markerFilename)
	{
		int patternID = addPattern(markerFilename);
		if (patternID < 0) 
		{
			ci::app::App::get()->console()<<"ERROR: failed loading pattern file:"<< markerFilename << std::endl;
			delete tracker;
			return false;
		} else {
			ci::app::App::get()->console()<<"Loaded marker[" << patternID << "] from file: " << markerFilename << std::endl;
		}
	}
	
	//tracker->changeCameraSize(_width,_height);
	
	// set default border size - thin = 0.125 & large = 0.250
	tracker->setBorderWidth(0.250);
	tracker->setUndistortionMode(ARToolKitPlus::UNDIST_LUT);
	
	// this is true by default:
	//tracker->setUseDetectLite(true);
	
	// RPP is more robust than ARToolKit's standard pose estimator
    tracker->setPoseEstimator(ARToolKitPlus::POSE_ESTIMATOR_RPP);
	
    // use the tool in tools/IdPatGen to generate markers
	//    tracker->setMarkerMode(ARToolKitPlus::MARKER_ID_SIMPLE);
	//    tracker->setMarkerMode(ARToolKitPlus::MARKER_TEMPLATE);
	//    tracker->setMarkerMode(ARToolKitPlus::MARKER_ID_BCH);	
	tracker->setMarkerMode(markerMode);
	
	// on by default
	tracker->activateAutoThreshold(true);
	
	return true;
}



// Must be luminance! Returns number of detected markers. After this, call updateDetectedMarkers()
// to update the internal markers data. 

int ARTKpMultiTracker::detect(const unsigned char *img)
{
	tracker->calc(img, &marker_info, &numDetected);
	return numDetected;
}


// Update marker information (id, 3d matrix transform, screen position, vertices)
// Should be run after calc()

void ARTKpMultiTracker::updateDetectedMarkers()
{
	markers.clear();
	
	float newVerts[4][2];
	
	for (int i=0; i<numDetected; ++i)
	{
		this->loadModelViewMatrix(i);
		
		for (int n=0; n<4; ++n)
		{
			newVerts[n][0] = cameraXToScreenX(marker_info[i].vertex[n][0]);
			newVerts[n][1] = cameraYToScreenY(marker_info[i].vertex[n][1]);
		}
		
		ARTKpMarker* marker = new ARTKpMarker(marker_info[i].id, 
											  cameraXToScreenX((float)marker_info[i].pos[0]),
											  cameraYToScreenY((float)marker_info[i].pos[1]),
											  (float*) newVerts);
		
		// confidence - not implemented yet
		//		marker.cf = marker_info[i].cf;
		memcpy(marker->gl_modelview,gl_modelview,16*sizeof(float));
		markers.push_back( ARTKpMarkerRef (marker) );
		
		//DEBUG:
		//std::cout<< marker_info[i].id<<std::endl;
	}
}


const ARFloat *ARTKpMultiTracker::getProjectionMatrix()
{
	return tracker->getProjectionMatrix();
}


// only used internally
void ARTKpMultiTracker::loadModelViewMatrix(int mi)
{
	if ((mi >= 0) && (mi < numDetected))
	{
		tracker->executeSingleMarkerPoseEstimator(&marker_info[mi], patt_center, patt_width, patt_trans);
		
		for(int j = 0; j < 3; j++)
		{
			for(int i = 0; i < 4; i++)
			{
				gl_modelview[i * 4 + j] = patt_trans[j][i];
			}
		}
		
		gl_modelview[0 * 4 + 3] = gl_modelview[1 * 4 + 3] = gl_modelview[2 * 4 + 3] = 0.0;
		gl_modelview[3 * 4 + 3] = 1.0;
	}
}