#include "State.h"

class PuzzleApp;

struct PuzzleState : public State<PuzzleApp>
{
	PuzzleState(PuzzleApp& app):State<PuzzleApp>(app){}
protected:
	double _time;
	double getElapsedSeconds();
	void resetTimer();
};

struct StateIdle: public PuzzleState
{
	StateIdle(PuzzleApp& app):PuzzleState(app){}
	void enter();
	void update();
	void draw();
	void exit();
};

struct StateInit: public PuzzleState
{
	StateInit(PuzzleApp& app):PuzzleState(app){}
	void enter();
	void update();
	void draw();
	void exit();
};

struct StateCountdown: public PuzzleState
{
	StateCountdown(PuzzleApp& app):PuzzleState(app){}
	void enter();
	void update();
	void draw();
	void exit();
};

struct StateShuffle: public PuzzleState
{
	StateShuffle(PuzzleApp& app):PuzzleState(app){}
	void enter();
	void update();
	void draw();
	void exit();
};

struct StateGame: public PuzzleState
{
	StateGame(PuzzleApp& app):PuzzleState(app){}
	void enter();
	void update();
	void draw();
	void exit();
};

struct StateGameover: public PuzzleState
{
	StateGameover(PuzzleApp& app):PuzzleState(app){}
	void enter();
	void update();
	void draw();
	void exit();
};

struct StateTakephoto: public PuzzleState
{
	StateTakephoto(PuzzleApp& app):PuzzleState(app){}
	void enter();
	void update();
	void draw();
	void exit();
};

struct StateSharepic: public PuzzleState
{
	StateSharepic(PuzzleApp& app):PuzzleState(app){}
	void enter();
	void update();
	void draw();
	void exit();
};