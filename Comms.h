/*	------------------------------------
	COMMS
	------------------------------------
	Description:
		Used for communications between fishbot camera navigation software and fishbot movement hardware.
		It handles querying the current fishbot speed (for calculation purposes), testing connections to the external hardware/components, 
		and sending data as needed (primarily directional instructions).
	------------------------------------
	Fields:
		_commReady (Boolean)
	Functions:
		private bool CheckConnection()
		public void ReadyComm()
		public bool SendData(Dirs)
		public float GetSpeed()
	------------------------------------*/
#include "Directions.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/video.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"

using namespace cv;

class Comms
{
private:
	//boolean indicating whether comms intitial setup has been performed 
	bool _commReady;

	//Returns true if connection to external component is active, otherwise returns false
	bool CheckConnection();

public:
	Comms()
	{
		_commReady = false;
	}

	//Performs Comms initial setup/reset
	bool ReadyComm();
        
    bool ClearScreen();
        
    bool FlattenX();
        
    bool FlattenY();

	//Sends data to external hardware/components
	bool SendRect(Rect toSend);
        
    bool SendLine(int ax, int ay, int bx, int by, int thickness);
        
        
};