#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"
#include "cinder/osc/OscSender.h"
#include "cinder/osc/OscListener.h"

using namespace ci;
using namespace ci::app;
using namespace std;

class FileServerAppApp : public AppBasic {
	enum{
		PORT = 1234,
	};
  public:
	void setup()
	{
		connected = false;
		listener.setup(PORT);
	//	sender.setup();
	}
	void update()
	{
		while (listener.hasWaitingMessages())
		{
			osc::Message msg;
			listener.getNextMessage(&msg);
			const string& addr = msg.getAddress();

			if (connected == false)
			{//shake hands
				string remote_ip = msg.getRemoteIp();
				int remote_port = msg.getRemotePort();
				console()<<remote_ip<<" "<<remote_port<<endl;
				sender.setup(remote_ip, remote_port);
				connected = true;
			}

			if (addr == "/list")
			{
				//do list				
			}
			else if (addr == "/pull")
			{
				int file_idx = msg.getArgAsInt32(0);
				//do pull
			}
			else if (addr == "/push")
			{
				int file_idx = msg.getArgAsInt32(0);
				//do push
			}
		}
	}

private:
	osc::Listener listener;
	osc::Sender		sender;
	bool		connected;
};

CINDER_APP_BASIC( FileServerAppApp, RendererGl )
