#include "Arduino.h"
#include "cinder/Serial.h"
#include "cinder/app/AppBasic.h"
#include <vector>

using namespace ci;
using namespace ci::app;
using namespace std;

static Serial serial;
static bool isConnected = false;

void setupArduino()
{
    // print the devices
    const vector<Serial::Device> &devices( Serial::getDevices() );
    if (devices.empty())
    {
        console() << "Arduino is not connected." << endl;
        return;
    }

    for( vector<Serial::Device>::const_iterator deviceIt = devices.begin(); deviceIt != devices.end(); ++deviceIt ) {
        console() << "Device: " << deviceIt->getName() << endl;
    }


    try {
        serial = Serial( devices[0], 9600);
    }
    catch( ... ) {
        console() << "There was an error initializing the serial device!" << endl;
        return;
    }

    isConnected = true;
}

void sendArduinoMsg(int cmd)
{
    if (!isConnected) return;
    serial.writeByte(cmd);
    serial.flush(false, true);
}
