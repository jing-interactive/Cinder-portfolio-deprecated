#include <opencv/highgui.h>
#include "cvButtons.h"


void cvButtonsOnMouse( int e, int x, int y, int f, void* param ){
	CvButtons* btn = (CvButtons*)param;
	btn->setMouseState(e,x,y,f);
}

#define C1 cvScalar(255,255,255)
#define C2 cvScalar(0,0,0)

void CvButtons::paintButtons(IplImage *img){
	vector<PushButton>::iterator it = buttonList.begin();
	while (it != buttonList.end()) {
		
		// Grab button variables:
		int x = it->x_pos;
		int y = it->y_pos;
		int w = it->width;
		int h = it->height;
		int x2 = x+w;
		int y2 = y+h;
		
		// Highlight mouseover position:
		if( mx >= x && mx <= x2 && my >= y && my <= y2 ){
			cvRectangle( img, cvPoint(x-4,y-4), cvPoint(x2+4,y2+4), C2, 1,CV_AA );
			
			// Check for mouse pressed event:
			if( me == CV_EVENT_LBUTTONDOWN/* || mf & CV_EVENT_FLAG_LBUTTON */){
				
				// Check if toggle button has to change state:
				if( it->toggle == 0 || it->toggle == 1 ) it->toggle = !it->toggle;
				
				// Call callback function:
				it->cb(it->toggle);
				
				// Draw confirmation rectangle:
				cvRectangle( img, cvPoint(x,y), cvPoint(x2,y2), C2,CV_FILLED,CV_AA );
				
				// Reset event (avoid flickering buttons):
				me = CV_EVENT_MOUSEMOVE;
			}
		}
		
		// Draw toggle state:
		if( it->toggle == 1 )
			cvRectangle( img, cvPoint(x,y), cvPoint(x2,y2), C2,CV_FILLED,CV_AA );
			
		// Draw button with text:
		cvRectangle( img, cvPoint(x,y), cvPoint(x2,y2), C2,1,CV_AA );
		if( it->toggle == 1 )
			cvPutText( img, it->text, cvPoint(x+5,y+15), &font, C1 );
		else
			cvPutText( img, it->text, cvPoint(x+5,y+15), &font, C2 );
        
        // Step to next button:
        it++;
	}
}

