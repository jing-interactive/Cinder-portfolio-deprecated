#include "PuzzleApp.h"
#include "Sprite.h"
#include "cinder/Rand.h"

void PuzzleApp::selectRandomImage()
{
	int n = _img_list.size();
	_img_selected = _img_list[randInt(n)];
	_tex_selected = _img_selected;
}

void PuzzleApp::shuffleSelectedImage(int size)
{
	_next_z = 0;

	if (!_img_selected)
		selectRandomImage();

	_sprites.clear();
	int img_w = _img_selected.getWidth();
	int img_h = _img_selected.getHeight();
	int tile_w = img_w/size;
	int tile_h = img_h/size;
	for (int y=0;y<size;y++)
	{
		for (int x=0;x<size;x++)
		{
			shared_ptr<Sprite> spr = shared_ptr<Sprite>(Sprite::createTile(_img_selected, x, y, tile_w, tile_h));
			spr->_z = _next_z++;
			_sprites.push_back(spr);
		}		
	}	
}