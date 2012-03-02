#ifndef ANIMATION_H
#define ANIMATION_H

#include <vector>
#include <string>
#include <cinder/Cinder.h>

namespace ARContent{

class Animation
{
public:
	enum Type
	{
		T_TRANSLATION_X,
		T_TRANSLATION_Y,
		T_TRANSLATION_Z,
		T_ROTATE_X,
		T_ROTATE_Y,
		T_ROTATE_Z,
		T_SCALE_X,
		T_SCALE_Y,
		T_SCALE_Z,
	};
	enum Direction
	{
		T_POSITIVE,
		T_NEGATIVE,
	};
	enum FunctionType
	{
		T_LINEAR,
		T_SIN,
	};
	std::string name;
	Type type;
	float magnitude;
	float speed;
	float start;
	float start_random_max;
	FunctionType function;
	float function_multiplier1;
	float function_multiplier1_random_max;
	float function_multiplier2;
	float function_multiplier2_random_max;
	float end;
	Direction direction;
	float wait_before_start;
	float wait_before_start_random_max;
};
}
#endif //ANIMATION_H