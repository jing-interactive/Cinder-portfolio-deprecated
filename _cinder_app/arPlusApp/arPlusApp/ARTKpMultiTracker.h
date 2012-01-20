/*
 *  ARTKpMultiTracker.h
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


#ifndef ARTKpMultiTracker_H
#define ARTKpMultiTracker_H

#include <iostream>
#include <string>
#include <vector>

#include "ARToolKitPlus/TrackerSingleMarker.h"
#include "ARTKpMarker.h"
#include "cinder/Matrix.h"


 

// our markers
typedef boost::shared_ptr<class ARTKpMarker> ARTKpMarkerRef;


class ARTKpMultiTracker
{
public:
	
	ARToolKitPlus::MARKER_MODE markerMode; // pattern mode - ARToolKitPlus::MARKER_TEMPLATE by default
	float screenWidth, screenHeight;	// for converting to screen coordinates from camera coords
	float cameraWidth, cameraHeight;	// for converting to screen coordinates from camera coords
	
	
	ARTKpMultiTracker();
	~ARTKpMultiTracker();
	
	int getNumDetectedMarkers() const { return numDetected; }
	
	// initialize using camera params file, marker template file, near and far clip
	bool init(const char * cameraParamFilename, const char * markerConfigFilename, int _width, int _height);
	
	// returns -1 if unsuccessful, otherwise the index of the new stored pattern to match on detect()
	int addPattern (const char* markerFilename) { return tracker->addPattern(markerFilename); }
	
	void setThreshold(int t)
	{
		tracker->setThreshold(t);
	}
	
	bool isValid() { return valid; }
	
	int getThreshold()
	{ 
		return tracker->getThreshold(); 
	}
	
	void activateAutoThreshold(bool e)
	{
		tracker->activateAutoThreshold(e);
	}
	
	void setFullImageProcessing(bool f);
	
	void activateVignettingCompensation(bool e)
	{
		tracker->activateVignettingCompensation(e);
	}
	
	int detect(const unsigned char *img);
	
	void updateDetectedMarkers();
	const ARFloat* getProjectionMatrix();
	
	inline float cameraXToScreenX(float cx)
	{
		return cx*screenWidth/cameraWidth;
	}
	
	inline float cameraYToScreenY(float cy)
	{
		return cy*screenHeight/cameraHeight;
	}
	
	inline std::vector<ARTKpMarkerRef>::iterator getMarkersBegin()
	{
		return markers.begin();
	}
	
	inline std::vector<ARTKpMarkerRef>::iterator getMarkersEnd()
	{
		return markers.end();
	}
	
	
private:
	
	ARToolKitPlus::TrackerSingleMarker *tracker;
	std::vector<ARTKpMarkerRef> markers; 
	int numDetected;
	bool valid;
	ARToolKitPlus::ARMarkerInfo *marker_info;
	
	ARFloat patt_width;
	ARFloat patt_center[2];
	ARFloat patt_trans[3][4];
	void loadModelViewMatrix(int i);
	float gl_modelview[16];
};

#endif
