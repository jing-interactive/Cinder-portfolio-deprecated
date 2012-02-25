#include "State.h"

struct StateIdle: public State
{
	StateIdle(PuzzleApp& app):State(app){}
	void enter();
	void update();
	void draw();
	void exit();
};

struct StateInit: public State
{
	StateInit(PuzzleApp& app):State(app){}
	void enter();
	void update();
	void draw();
	void exit();
};

struct StateCountdown: public State
{
	StateCountdown(PuzzleApp& app):State(app){}
	void enter();
	void update();
	void draw();
	void exit();
};

struct StateShuffle: public State
{
	StateShuffle(PuzzleApp& app):State(app){}
	void enter();
	void update();
	void draw();
	void exit();
};

struct StateGame: public State
{
	StateGame(PuzzleApp& app):State(app){}
	void enter();
	void update();
	void draw();
	void exit();
};

struct StateGameover: public State
{
	StateGameover(PuzzleApp& app):State(app){}
	void enter();
	void update();
	void draw();
	void exit();
};

struct StateTakephoto: public State
{
	StateTakephoto(PuzzleApp& app):State(app){}
	void enter();
	void update();
	void draw();
	void exit();
};

struct StateSharepic: public State
{
	StateSharepic(PuzzleApp& app):State(app){}
	void enter();
	void update();
	void draw();
	void exit();
};