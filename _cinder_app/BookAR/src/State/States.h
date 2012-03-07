#include "State.h"
#include <cinder/app/MouseEvent.h>

class BookAR;

struct BookARState: public State<BookAR>
{
	BookARState(BookAR& app):State<BookAR>(app){}
	virtual void mouseDown(cinder::app::MouseEvent event){}
	virtual void mouseMove(cinder::app::MouseEvent event){}
	virtual void mouseUp(cinder::app::MouseEvent event){}
};

struct StateTracking: public BookARState
{
	StateTracking(BookAR& app):BookARState(app){}
	void enter();
	void update();
	void draw();
	void exit();
	void mouseUp(cinder::app::MouseEvent event);
};

struct StateCreating: public BookARState
{
	StateCreating(BookAR& app):BookARState(app){}
	void enter();
	void update();
	void draw();
	void exit();
	void mouseDown(cinder::app::MouseEvent event);
};

struct StateSharing: public BookARState
{
	StateSharing(BookAR& app):BookARState(app){}
	void enter();
	void update();
	void draw();
	void exit();
	void mouseDown(cinder::app::MouseEvent event);
};

struct StateGlobal: public BookARState
{
	StateGlobal(BookAR& app):BookARState(app){}
	void enter();
	void update();
	void draw();
	void exit();
};