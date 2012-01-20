/*
 *  ARTKpMarker.cpp
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


#include "ARTKpMarker.h"

ARTKpMarker::ARTKpMarker()
{
	id = area = 0;
	screenX = screenY = 0.0f;
	//memset(verts, 0, 8*sizeof(float));
	
}


ARTKpMarker::ARTKpMarker(const int _id, const float cx, const float cy, const float* _verts)
{
	id = _id;
	screenX = cx;
	screenY = cy;
	memcpy(verts, _verts, 8*sizeof(float));
}

ARTKpMarker::~ARTKpMarker()
{
	
}