#pragma once

bool loadConfig(const char* config);
bool saveConfig(const char* config);

//device param
extern int OSC_PORT;
extern float Z_NEAR;//kinect sensor value
extern float Z_FAR;
extern float LED_MIN_ALPHA;//very important
extern float LED_MAX_ALPHA;//very important

//Line
extern int LINE_MIN_LIFE;
extern int LINE_MAX_LIFE;
extern int LINE_MIN_SNAKE;
extern int LINE_MAX_SNAKE;
extern int LINE_RADIUS_X;
extern int LINE_RADIUS_Y;
extern float LINE_MIN_ALPHA_DECAY;
extern float LINE_MAX_ALPHA_DECAY;

//cube rendering
extern float CUBE_SIZE;
extern float KX;//spacing X
extern float KY;//spacing Y
extern float KZ;//spacing Z
extern int SCR_W;
extern int SCR_H;

//led manager
extern int SP;//texture spacing 
extern int DARK_CLR_R;
extern int DARK_CLR_G;
extern int DARK_CLR_B;
extern int LIGHT_CLR_R;
extern int LIGHT_CLR_G;
extern int LIGHT_CLR_B;
extern float SCR_LED_SPEED;

//spark item
extern int SPARK_MIN_LIFE;
extern int SPARK_MAX_LIFE;
extern float SPARK_MIN_SPEED;
extern float SPARK_MAX_SPEED;
extern int SPARK_RADIUS_X;
extern int SPARK_RADIUS_Y;
extern int SPARK_RADIUS_Z;

//state animal
extern float ANIMAL_MIN_GROW_SPEED;
extern float ANIMAL_MAX_GROW_SPEED;
extern float ANIMAL_POS_SCALE_X;
extern float ANIMAL_POS_SCALE_Y;
extern float ANIMAL_POS_SCALE_Z;
extern float ANIMAL_NEG_SCALE_X;
extern float ANIMAL_NEG_SCALE_Y;
extern float ANIMAL_NEG_SCALE_Z;
extern float ANIMAL_MIN_BODY_RADIUS;
extern float ANIMAL_MAX_BODY_RADIUS;

//state breathe
extern float BREATHE_MIN_ALPHA_SPEED;
extern float BREATHE_MAX_ALPHA_SPEED;
extern float BREATHE_MIN_DECAY;
extern float BREATHE_MAX_DECAY;
extern float BRETHE_MIN_THRESH_ALPHA;
extern float BRETHE_MAX_THRESH_ALPHA;
extern float BREATH_MIN_ROOT_ALPHA_LOW;
extern float BREATH_MIN_ROOT_ALPHA_HIGH;
extern float BREATH_MAX_ROOT_ALPHA_LOW;
extern float BREATH_MAX_ROOT_ALPHA_HIGH;

//state following
extern int FOLLOWING_N_ITEMS;
extern float FOLLOWING_MAX_SPEED;

//state lots
extern int LOTS_SEC_FADEOUT;
extern int LOTS_SEC_FADEIN;
extern float LOTS_K_ITEMS;
extern float LOTS_COME_SPEED;
extern float LOTS_GO_SPEED;

//state spark
extern int SPARK_N_ITEMS;//duplicates might exist

//state spark int
extern int SPARKINT_N_ITEMS;

//timing
extern int SEC_TURN_IDLE;//change to idle states if no one moving
extern int SEC_FADE_OUT;

extern int DEFAULT_COUNTDOWN;
extern int ANIMAL_COUNTDOWN;
extern int BREATHE_COUNTDOWN;
extern int FOLLOWING_COUNTDOWN;
extern int LOTS_COUNTDOWN;
extern int SPARK_COUNTDOWN;
extern int SPARKINT_COUNTDOWN;



