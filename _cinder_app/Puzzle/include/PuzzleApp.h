#pragma once

#include "cinder/app/AppBasic.h"
#include "cinder/gl/Texture.h"

namespace cinder { namespace osc {
	class Listener;
	class Message;
}}

using namespace ci;
using namespace ci::app;
using namespace std;

struct Hand
{ 
	Hand()
	{
		push = false;
	}
	Vec2i pos;
	bool push;
};

struct Sprite;
struct State;

class PuzzleApp : public AppBasic 
{
  public:
	void prepareSettings(Settings *settings);
	void setup();

	void mouseWheel( MouseEvent event );
	void mouseMove( MouseEvent event );
	void mouseDown( MouseEvent event );
	void keyDown(KeyEvent event);
	void update();
	void draw();

	void onOscMessage(const osc::Message* msg);

private:
	shared_ptr<osc::Listener> listener;
	enum{
		LEFT = 0,
		RIGHT,
		N_HANDS,
	};
	Hand _hands[N_HANDS];
private: 
	float _scale;
	float _rotate;
private://image
	vector<Surface8u> _img_list;
	vector<shared_ptr<Sprite>> _sprites;

	void shuffleImage(const Surface8u& img);

private://game over
	gl::Texture _tex_player;

private://state machine
	shared_ptr<State> _state_idle;
	shared_ptr<State> _state_init;
	shared_ptr<State> _state_countdown;
	shared_ptr<State> _state_shuffle;
	shared_ptr<State> _state_game;
	shared_ptr<State> _state_gameover;
	shared_ptr<State> _state_takephoto;
	shared_ptr<State> _state_sharepic;

	shared_ptr<State> _current_state;

	void setupStates();
	void updateStates();
	void changeToState(const shared_ptr<State>& new_state);
}; 
