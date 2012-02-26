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

enum{
	LEFT = 0,
	RIGHT,
	N_HANDS,
};

struct Hand
{
	Hand()
	{
		push = false;
	}
	void draw();
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
	void mouseUp( MouseEvent event );
	void mouseDrag( MouseEvent event );

	void keyDown(KeyEvent event);
	void update();
	void draw();

	void onOscMessage(const osc::Message* msg);

private:
	shared_ptr<osc::Listener> listener;
public:
	Hand _hands[N_HANDS];
	float _scale;
	float _rotate;
private:
	vector<Surface8u> _img_list;
	//img -> size*size Sprite
public://image
	int _next_z; 
	void selectRandomImage();
	void shuffleSelectedImage(int size);
	vector<shared_ptr<Sprite>> _sprites;
	gl::Texture _tex_selected;
	Surface8u _img_selected;

	gl::Texture _tex_player;

public://state machine
	shared_ptr<State> _state_idle;
	shared_ptr<State> _state_init;
	shared_ptr<State> _state_countdown;
	shared_ptr<State> _state_shuffle;
	shared_ptr<State> _state_game;
	shared_ptr<State> _state_gameover;
	shared_ptr<State> _state_takephoto;
	shared_ptr<State> _state_sharepic;

	shared_ptr<State> _current_state;
	shared_ptr<Sprite> _sprite_selected;

	void setupStates();
	void updateStates();
	void changeToState(const shared_ptr<State>& new_state);
}; 
