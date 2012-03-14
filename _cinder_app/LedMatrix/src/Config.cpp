#include "Config.h"
#include <cinder/xml.h>
#include <cinder/app/App.h>
#include <cinder/Utilities.h>
#include <string>

using namespace ci;
using namespace ci::app;
using namespace std;

#define HUGE_STUFF() do {\
DO_ITEM(OSC_PORT);\
DO_ITEM(Z_NEAR);\
DO_ITEM(Z_FAR);\
DO_ITEM(LED_MIN_ALPHA);\
DO_ITEM(LED_MAX_ALPHA);\
DO_ITEM(LINE_MIN_LIFE);\
DO_ITEM(LINE_MAX_LIFE);\
DO_ITEM(LINE_MIN_SNAKE);\
DO_ITEM(LINE_MAX_SNAKE);\
DO_ITEM(LINE_RADIUS_X);\
DO_ITEM(LINE_RADIUS_Y);\
DO_ITEM(LINE_MIN_ALPHA_DECAY);\
DO_ITEM(LINE_MAX_ALPHA_DECAY);\
DO_ITEM(CUBE_SIZE);\
DO_ITEM(KX);\
DO_ITEM(KY);\
DO_ITEM(KZ);\
DO_ITEM(SCR_W);\
DO_ITEM(SCR_H);\
DO_ITEM(SP);\
DO_ITEM(DARK_CLR_R);\
DO_ITEM(DARK_CLR_G);\
DO_ITEM(DARK_CLR_B);\
DO_ITEM(LIGHT_CLR_R);\
DO_ITEM(LIGHT_CLR_G);\
DO_ITEM(LIGHT_CLR_B);\
DO_ITEM(SCR_LED_SPEED);\
DO_ITEM(SPARK_MIN_LIFE);\
DO_ITEM(SPARK_MAX_LIFE);\
DO_ITEM(SPARK_MIN_SPEED);\
DO_ITEM(SPARK_MAX_SPEED);\
DO_ITEM(SPARK_RADIUS_X);\
DO_ITEM(SPARK_RADIUS_Y);\
DO_ITEM(SPARK_RADIUS_Z);\
DO_ITEM(ANIMAL_MIN_GROW_SPEED);\
DO_ITEM(ANIMAL_MAX_GROW_SPEED);\
DO_ITEM(ANIMAL_POS_SCALE_X);\
DO_ITEM(ANIMAL_POS_SCALE_Y);\
DO_ITEM(ANIMAL_POS_SCALE_Z);\
DO_ITEM(ANIMAL_NEG_SCALE_X);\
DO_ITEM(ANIMAL_NEG_SCALE_Y);\
DO_ITEM(ANIMAL_NEG_SCALE_Z);\
DO_ITEM(ANIMAL_MIN_BODY_RADIUS);\
DO_ITEM(ANIMAL_MAX_BODY_RADIUS);\
DO_ITEM(BREATHE_MIN_ALPHA_SPEED);\
DO_ITEM(BREATHE_MAX_ALPHA_SPEED);\
DO_ITEM(BREATHE_MIN_DECAY);\
DO_ITEM(BREATHE_MAX_DECAY);\
DO_ITEM(BRETHE_MIN_THRESH_ALPHA);\
DO_ITEM(BRETHE_MAX_THRESH_ALPHA);\
DO_ITEM(BREATH_MIN_ROOT_ALPHA_LOW);\
DO_ITEM(BREATH_MIN_ROOT_ALPHA_HIGH);\
DO_ITEM(BREATH_MAX_ROOT_ALPHA_LOW);\
DO_ITEM(BREATH_MAX_ROOT_ALPHA_HIGH);\
DO_ITEM(FOLLOWING_N_ITEMS);\
DO_ITEM(FOLLOWING_MAX_SPEED);\
DO_ITEM(LOTS_SEC_FADEOUT);\
DO_ITEM(LOTS_SEC_FADEIN);\
DO_ITEM(LOTS_K_ITEMS);\
DO_ITEM(LOTS_COME_SPEED);\
DO_ITEM(LOTS_GO_SPEED);\
DO_ITEM(SPARK_N_ITEMS);\
DO_ITEM(SPARKINT_N_ITEMS);\
DO_ITEM(SEC_TURN_IDLE);\
DO_ITEM(SEC_FADE_OUT);\
DO_ITEM(DEFAULT_COUNTDOWN);\
DO_ITEM(ANIMAL_COUNTDOWN);\
DO_ITEM(BREATHE_COUNTDOWN);\
DO_ITEM(FOLLOWING_COUNTDOWN);\
DO_ITEM(LOTS_COUNTDOWN);\
DO_ITEM(SPARK_COUNTDOWN);\
DO_ITEM(SPARKINT_COUNTDOWN);\
}while(0)

bool loadConfig(const char* config)
{
	try
	{
		XmlTree tree(loadFile(config));
		XmlTree items = tree.getChild("LedMatrix");

#define DO_ITEM(item) item = items.getChild(#item).getValue<float>();
		HUGE_STUFF();
#undef DO_ITEM

	}
	catch( ... ) {
		console() << "[ERROR] Failed to load from " << config<<std::endl;
		return false;
	}
	return true;
}

bool saveConfig(const char* config)
{	
	try
	{
		XmlTree tree("LedMatrix","");

#define DO_ITEM(item) tree.push_back(XmlTree(#item, toString(item)))
		HUGE_STUFF();
#undef DO_ITEM
		tree.write( writeFile(config));
	}
	catch( ... ) {
		console() << "[ERROR] Failed to save to " << config<<std::endl;
		return false;
	}
	return true;
}

//device param
int OSC_PORT = 7777;
float Z_NEAR = 800;//kinect sensor value
float Z_FAR = 4000;
float LED_MIN_ALPHA = 0.0f;//very important
float LED_MAX_ALPHA = 1.0f;//very important

//Line
int LINE_MIN_LIFE = 1;
int LINE_MAX_LIFE = 3;
int LINE_MIN_SNAKE = 5;
int LINE_MAX_SNAKE = 30;
int LINE_RADIUS_X = 2;
int LINE_RADIUS_Y = 3;
float LINE_MIN_ALPHA_DECAY = 0.8f;
float LINE_MAX_ALPHA_DECAY = 0.95f;

//cube rendering
float CUBE_SIZE = 1.0f;
float KX = 5;//spacing X
float KY = 5;//spacing Y
float KZ = 5;//spacing Z
int SCR_W = 29;
int SCR_H = 28;

//led manager
int SP = 5;//texture spacing
int DARK_CLR_R = 229;
int DARK_CLR_G = 229;
int DARK_CLR_B = 229;
int LIGHT_CLR_R = 50;
int LIGHT_CLR_G = 179;
int LIGHT_CLR_B = 225;
float SCR_LED_SPEED = 0.2f;

//spark item
int SPARK_MIN_LIFE = 2;
int SPARK_MAX_LIFE = 4;
float SPARK_MIN_SPEED = 0.01f;
float SPARK_MAX_SPEED = 0.04f;
int SPARK_RADIUS_X = 2;
int SPARK_RADIUS_Y = 2;
int SPARK_RADIUS_Z = 4;

//state animal
float ANIMAL_MIN_GROW_SPEED = 0.003f;
float ANIMAL_MAX_GROW_SPEED = 0.012f;
float ANIMAL_POS_SCALE_X = 1.5f;
float ANIMAL_POS_SCALE_Y = 1.0f;
float ANIMAL_POS_SCALE_Z = 0.3f;
float ANIMAL_NEG_SCALE_X = 1.0f;
float ANIMAL_NEG_SCALE_Y = 1.5f;
float ANIMAL_NEG_SCALE_Z = 0.2f;
float ANIMAL_MIN_BODY_RADIUS = 2.0f;
float ANIMAL_MAX_BODY_RADIUS = 3.5f;

//state breathe
float BREATHE_MIN_ALPHA_SPEED = 0.003f;
float BREATHE_MAX_ALPHA_SPEED = 0.01f;
float BREATHE_MIN_DECAY = 5.4f;
float BREATHE_MAX_DECAY = 8.0f;
float BRETHE_MIN_THRESH_ALPHA = 1.0f;
float BRETHE_MAX_THRESH_ALPHA = 10.0f;
float BREATH_MIN_ROOT_ALPHA_LOW = 0;
float BREATH_MIN_ROOT_ALPHA_HIGH = 10;
float BREATH_MAX_ROOT_ALPHA_LOW = 170;
float BREATH_MAX_ROOT_ALPHA_HIGH = 255;

//state following
int FOLLOWING_N_ITEMS = 40;
float FOLLOWING_MAX_SPEED = 0.4f;

//state lots
int LOTS_SEC_FADEOUT = 2;
int LOTS_SEC_FADEIN = 2;
float LOTS_K_ITEMS = 0.75f;
float LOTS_COME_SPEED = 0.6f;
float LOTS_GO_SPEED = 2.4f;

//state spark
int SPARK_N_ITEMS = 100;//duplicates might exist

//state spark int
int SPARKINT_N_ITEMS = 30;

//timing
int SEC_TURN_IDLE = 5;//change to idle states if no one moving
int SEC_FADE_OUT = 5;

int DEFAULT_COUNTDOWN = 60;
int ANIMAL_COUNTDOWN = 60;
int BREATHE_COUNTDOWN = 60;
int FOLLOWING_COUNTDOWN = 60;
int LOTS_COUNTDOWN = 60;
int SPARK_COUNTDOWN = 60;
int SPARKINT_COUNTDOWN = 60;





