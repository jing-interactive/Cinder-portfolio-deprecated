#include <cinder/app/AppBasic.h>
#include <cinder/gl/gl.h>
#include <cinder/params/Params.h>
#include <OscSender.h>

using namespace ci;
using namespace ci::app;
using namespace std;

struct Blob
{
	Blob():visible(false),radius(10){}
	bool visible;
	Vec2f pos;
	float z;
	int id;
	const float radius;

	void draw()
	{
		if (!visible)
			return;
		gl::color(Color8u(100*id+100,0,200-100*id));
		gl::drawSolidCircle(pos, radius);
	}

	void addToBundle( osc::Bundle& bundle, int dev_id ) 
	{
		if (!visible)
			return;
		osc::Message m;			
		m.setAddress("/contour");
		m.addIntArg(id);                        //0
		m.addStringArg("/move");				//1
		m.addFloatArg(pos.x/getWindowWidth());					//2 -> cx
		m.addFloatArg(pos.y/getWindowHeight());					//3 -> cy
		m.addFloatArg(0);					//4
		m.addFloatArg(z);					//5	-> cz
		m.addFloatArg(0);					//6 rotation
		m.addIntArg(dev_id);				//7 -> device id

		bundle.addMessage(m);
	}
};

class KinServerSimApp : public AppBasic 
{
public:
	void prepareSettings( Settings *settings );
	void setup();
	void mouseDown( MouseEvent event );	
	void keyDown( KeyEvent event );
	void update();
	void draw();
private:
	Blob blobs[2]; 
	osc::Sender sender;
	osc::Bundle bundle;
	params::InterfaceGl	param;
};

void KinServerSimApp::prepareSettings( Settings *settings )
{
	settings->setResizable(false);
}

void KinServerSimApp::setup()
{
	//osc
	sender.setup("localhost", 7777);
	//blob
	blobs[0].id = 0;
	blobs[1].id = 1;
	//param GUI
	param = params::InterfaceGl( "param", Vec2i( 200, 100 ));
	{
		blobs[0].z = 2000;
		param.addParam( "z1", &blobs[0].z, "min=800 max=4000 step=100 keyIncr=a keyDecr=A");

		blobs[1].z = 2000;
		param.addParam( "z2", &blobs[1].z, "min=800 max=4000 step=100 keyIncr=b keyDecr=B");
	}
}

void KinServerSimApp::mouseDown( MouseEvent event )
{
	int idx = -1;
	if (event.isLeft())
		idx = 0;
	else if (event.isRight())
		idx = 1;
	else if (event.isMiddle())
	{
		blobs[0].visible = false;
		blobs[1].visible = false;
	}
	if (idx >= 0)
	{
		Blob& b = blobs[idx];
		b.visible = true;
		b.pos = event.getPos();
	}
}

void KinServerSimApp::update()
{
	bundle.clear();
	for (int i=0;i<2;i++)
	{
		osc::Message m;
		m.setAddress("/start"); 
		m.addIntArg(i);
		bundle.addMessage(m);
		blobs[i].addToBundle(bundle, i);
	}
	sender.sendBundle(bundle);
}

void KinServerSimApp::draw()
{
	// clear out the window with black
	gl::clear( Color( 0, 0, 0 ) );
	gl::setMatricesWindow(getWindowSize());
	for (int i=0;i<2;i++)
	{
		blobs[i].draw();
	}
	params::InterfaceGl::draw();
}

void KinServerSimApp::keyDown( KeyEvent event )
{
	if (event.getCode() == KeyEvent::KEY_ESCAPE)
		quit();
}

CINDER_APP_BASIC( KinServerSimApp, RendererGl )
