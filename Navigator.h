/*	------------------------------------
	NAVIGATOR
	------------------------------------
	Description:
		Virtual class for navigator children. Defines the core functionality that will be overridden by child
		classes.
	------------------------------------
	Fields:
		_communicator (Comms)
		_initialSetupComplete (Bool)
		_commandDirections (Dirs)
	Functions:
		public void InitialSetup()
		public float GetPollValue()
		public bool DecideDirections(vector<NavObject>, vector<NavObject>)
		public bool DebugDecideDirections(*Mat, vector<NavObject>, vector<NavObject>)
		public bool StopIdle()
		public bool SendDirectionCommands()
------------------------------------*/

#include "Directions.h"
#include "Comms.h"
#include "NavObject.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"


class Navigator
{
protected:
	//communicator object, used for external data transferring
	Comms _communicator;

	//the latest direction calculations
	Directions _commandDirections;

	//boolean flag indicating whether the latest direction calculations have been successfully sent via communicator
	bool _directionsSent;

	//boolean flag indicating whether initial setup has been completed or not. Default false
	bool _initialSetupComplete;
public:
	//default constructor
	Navigator()
	{
		_directionsSent = false;
		_initialSetupComplete = false;
		_commandDirections = Directions();
		_communicator = Comms();
	}

	//Acts as emergency stop, commands ALL movement to cease. Can be called external to navigator
	void StopIdle();

	//if initital setup complete flag is false, sets up and sets flag to true
	virtual void InitialSetup() = 0;

	//calculates and returns polling value (may query data from external source for calculations)
	virtual float GetPollValue() = 0;

	//takes in object data, which is then used to calculate the best direction to move in order to avoid collisions, and which direction flags should be used via external components
	//sets direction sent flag to false on 
	virtual bool DecideDirections(std::vector<NavObject> BeaconData, std::vector<NavObject> InputData) = 0;

	virtual bool DebugDecideDirections(cv::Mat* currentFrame, std::vector<NavObject> BeaconData, std::vector<NavObject> InputData) = 0;

	//sends direction data to external components, sets direction sent flag to true and returns true if sent successfully, otherwise returns false
	bool SendDirectionCommands();
};