#pragma once

class PuzzleApp;

struct State
{
	State(PuzzleApp& app);
	virtual void enter() = 0;
	virtual void update() = 0;
	virtual void draw() = 0;
	virtual void exit() = 0;
	PuzzleApp& _app;
protected:
	double _time;
	double getElapsedSeconds();
	void resetTimer();
};
