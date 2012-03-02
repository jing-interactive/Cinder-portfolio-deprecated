#include "State.h"

class BookAR;

struct StateTracking: public State<BookAR>
{
	StateTracking(BookAR& app):State<BookAR>(app){}
	void enter();
	void update();
	void draw();
	void exit();
};

struct StateCreating: public State<BookAR>
{
	StateCreating(BookAR& app):State<BookAR>(app){}
	void enter();
	void update();
	void draw();
	void exit();
};

struct StateSharing: public State<BookAR>
{
	StateSharing(BookAR& app):State<BookAR>(app){}
	void enter();
	void update();
	void draw();
	void exit();
};

struct StateGlobal: public State<BookAR>
{
	StateGlobal(BookAR& app):State<BookAR>(app){}
	void enter();
	void update();
	void draw();
	void exit();
};