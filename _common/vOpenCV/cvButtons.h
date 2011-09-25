#ifndef CVBUTTONS_H_
#define CVBUTTONS_H_

#include <opencv/cv.h>
#include <vector>

using std::vector;

/**
	Global OpenCV mouse button callback.
    Must be set with cvSetMouseCallback(..) to make buttons work.
*/
void cvButtonsOnMouse( int event, int x, int y, int flags, void* param );

/**
	\class   PushButton
	\brief   <b>Class PushButton</b>\n
	\brief   Implements a single push button object.\n
	\author  Andreas Geiger
	\author  Karlsruhe Institute of Technology
	\version 1.0
	\date    16.07.2007
*/
class PushButton {

public:

    /**
        Constructor takes parameters such as:
        - \b x , \b y : x/y position of a push button
        - \b w , \b h : width/height of a push button
        - \b t: -1 if normal button or 0/1 as state of a toggle button
        - \b text: button description
        - \b cb: button callback function. Takes a function pointer.
        The argument will be the button toggle state when pressed.
	*/
    PushButton( int x, int y, int w, int h, int t, char *text, void (*cb)(int) ):
                x_pos(x), y_pos(y), width(w), height(h), toggle(t), text(text), cb(cb) {}

    /**
        x/y position of a push button
    */
	int x_pos, y_pos;

    /**
        width/height of a push button
    */
	int width, height;

    /**
        -1 if normal button or 0/1 as state of a toggle button
    */
	int toggle;

    /**
        button description
    */
	char *text;

    /**
        button callback function. Takes a function pointer.
    */
	void (*cb)(int);
};

/**
	\class   CvButtons
	\brief   <b>Class CvButtons</b>\n
	\brief   Implements functions to enhance the OpenCV GUI elements\n
	\brief   by simple, platform-independet push buttons and toggle elements.\n
	\author  Andreas Geiger
	\author  Karlsruhe Institute of Technology
	\version 1.0
	\date    16.07.2007
*/
class CvButtons {

public:

    /**
        Constructor creates button font
    */
	CvButtons(){ cvInitFont( &font, CV_FONT_HERSHEY_PLAIN, 1, 1, 0, 1, CV_AA ); }

	void release(){ buttonList.clear(); }
    /**
        Deconstructor clears the button list
    */
	~CvButtons(){ release(); }

    /**
        Called by cvButtonsOnMouse() when button was pressed
    */
	void setMouseState(int e, int x, int y, int f) {me=e; mx=x; my=y; mf=f;}

    /**
        Paint all buttons to an image
    */
	void paintButtons(IplImage *img);

    /**
        Add button to list
    */
	void addButton(PushButton pb){ buttonList.push_back(pb); }

    /**
        Delete button from list
    */
	void delButton(int pos){ buttonList.erase( buttonList.begin()+pos ); }

private:

    /**
        The list of all buttons in this class
    */
	vector<PushButton> buttonList;

    /**
        The last event (mouse state)
    */
	int me, mx, my, mf;

    /**
        The font to draw the buttons
    */
	CvFont font;
};

#endif /*CVBUTTONS_H_*/
