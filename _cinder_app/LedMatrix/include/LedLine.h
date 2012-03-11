#pragma once

class Line
{
	float z=0;//[0, Z)
	float speed = 0.7;
	float decay = 0.8;
	boolean visible = true;
	boolean come = true;//come and back
	void setup()
	{
		z = 0;
		come = true;
		speed = random(0.7, 1);
		decay = random(0.65, 0.8);
		visible = random(10) > 0 ? true : false;
	}
	void update()
	{
		if (come)
		{
			z += speed/2;
			if (z > target_z)//TODO:
				come = false;
		}
		else
		{
			z -= speed;
			if (z <= 0)//TODO:
			{
				setup();
			}
		}
	}
	void draw()
	{

	}
}