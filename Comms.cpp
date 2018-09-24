/*#include <iostream>
#include "Comms.h"
#include "wiringSerial.h"
#include <wiringPi.h>
#include <unistd.h>
#include "GPIOClass.h"

using namespace std;

int fd ; //serial file directory

bool Comms::CheckConnection()
{
	//NOT IMPLEMENTED - RETURNS TRUE FOR NOW
	return true;
}

bool Comms::ReadyComm()
{

  //open serial port
  if ((fd = serialOpen ("/dev/ttyACM0", 115200)) < 0)
  {
    fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
    return 0;
  }
  return 1;
}

bool Comms::SendRect(Rect toSend)
{
    char msg = '0' + 0;
    serialPutchar(fd, msg);
    float x = toSend.x * 255 / 320;
    msg = '0' + x;
    serialPutchar(fd, msg);
    msg= '0' + (240 - (toSend.y + toSend.height));
    serialPutchar(fd, msg);
    float width = toSend.width * 255 / 320;
    msg = '0' + width;
    serialPutchar(fd, msg);
    msg= '0' + toSend.height;
    serialPutchar(fd, msg);
	//NOT IMPLEMENTED - RETURNS TRUE TO TRIGGER "MESSAGE SENT" RESPONSE
	return true;
}

bool Comms::ClearScreen()
{
    char msg = '0' + 2;
    serialPutchar(fd, msg);
}

bool Comms::FlattenX()
{
    char msg = '0' + 3;
    serialPutchar(fd, msg);
}

bool Comms::FlattenY()
{
    char msg = '0' + 4;
    serialPutchar(fd, msg);
}

bool Comms::SendLine(int ax, int ay, int bx, int by, int thickness)
{
    char msg = '0' + 1;
    serialPutchar(fd, msg);
    float x = (float)ax  * 255 / 320;
    msg = '0' + x;
    serialPutchar(fd, msg);
    msg = '0' + (240 - ay);
    serialPutchar(fd, msg);
    float x2 = (float)bx  * 255 / 320;
    msg = '0' + x2;
    serialPutchar(fd, msg);
    msg = '0' + (240 - by);
    serialPutchar(fd, msg);

    cout << endl << "SENDING TO SERIAL: (" << x << ", " << ay << ")   (" << x2 << ", " << by << ", " << endl;
    
    for(int i = 1; i <= thickness; i++)
    {
        SendLine(ax, ay+i, bx, by+i, 0);
        SendLine(ax, ay-i, bx, by-i, 0);
        SendLine(ax+i, ay, bx+i, by, 0);
        SendLine(ax-i, ay, bx-i, by, 0);
    }
}*/
