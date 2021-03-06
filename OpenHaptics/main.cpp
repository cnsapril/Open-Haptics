#include "config.h"

/************************************************************************
 Print user menu in command line tool
************************************************************************/
void PrintHelp()
{
	static const char help[] = {\
		"Open Haptics\n\
		---\n\
		I: Input weight\n\
		L: Apply torque force to long objects\n\
		T: Apply texture\n\
		P: Print device state\n\
		Q: Quit the program\n\
		---"};

	printf("\n%s\n", help);
}

/************************************************************************
 Directly sets the force values in forceVecServo to the haptic device.
************************************************************************/
HDCallbackCode HDCALLBACK ServoSchedulerCallback(void *pUserData)
{
	HDErrorInfo error;
	
	hdBeginFrame(hdGetCurrentDevice());

	hdSetDoublev(HD_CURRENT_FORCE, forceVecServo);

	hdSetDoublev(HD_CURRENT_JOINT_TORQUE, torqueServo);

	hdEndFrame(hdGetCurrentDevice());

	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
		hduPrintError(stderr, &error, "Error while applying force and torque.");
		memset(forceVecServo, 0, sizeof(double) * FORCE_DIM);

		if (hduIsSchedulerError(&error))
		{
			return HD_CALLBACK_DONE;
		}
	}

	return HD_CALLBACK_CONTINUE;

	
}

/************************************************************************
 Copies force state in thread-safe manner.
************************************************************************/
HDCallbackCode HDCALLBACK UpdateForceCallback(void *pUserData)
{
	memcpy(forceVecServo, forceVecApp, sizeof(double) * FORCE_DIM);
	
	return HD_CALLBACK_DONE;
}

/************************************************************************
 Copies torque state in thread-safe manner.
************************************************************************/
HDCallbackCode HDCALLBACK UpdateTorqueCallback(void *pUserData)
{
	memcpy(torqueServo, torqueApp, sizeof(double) * TORQUE_DIM);

	return HD_CALLBACK_DONE;
}
/************************************************************************
 Callback function that retrieves the current device state
************************************************************************/
HDCallbackCode HDCALLBACK GetDeviceStateCallback(void *pUserData)
{
	DeviceStateStruct *pState = (DeviceStateStruct *) pUserData;

	hdGetDoublev(HD_CURRENT_POSITION, pState->position);

	return HD_CALLBACK_DONE;
}

/************************************************************************
 Read user input of weight and apply the value to forceVecApp.
*************************************************************************/
void SetForce()
{
	double inputMass;

	printf("\nInput the weight: between 0 and 300g: [0 - 300]\n");
	
	do
	{
		printf("Enter the weight and press return: ");
		scanf("%lf", &inputMass);
	} while(inputMass < 0 || inputMass > 300);

	double adjustedMass = (inputMass * ADJUST_FACTOR - ADJUST_OFFSET) / 1000.0;

	// Gravity is vertical force along the device's Y-axis
	forceVecApp.set(0.0, -(adjustedMass * GRAVITY_ACC), 0.0);

	hdScheduleSynchronous(UpdateForceCallback, 0, HD_DEFAULT_SCHEDULER_PRIORITY);
}

/************************************************************************
 Read user input of the weight and length and apply the value to 
 forceVecApp.
 ***********************************************************************/
void SetTorque()
{
	double inputMass;
	int inputLength;

	printf("\nInput the weigh: between 0 and 300g: [0 - 300]\n");
	do
	{
		printf("Enter the weight and press return: ");
		scanf("%lf", &inputMass);
	} while(inputMass < 0 || inputMass > 300);

	printf("\nInput the length of the object in cm:\n");
	scanf("%d", &inputLength);

	inputLength *= 10; // Convert input length to mm

	double halfLength = inputLength / 2.0; // Since the object is balanced, the center of gravity is at half of its length
	double gravity = inputMass * GRAVITY_ACC / 1000.0;

	hduVector3Dd force(0, -gravity, 0);

	DeviceStateStruct state;
	memset(&state, 0, sizeof(DeviceStateStruct));

	printf("\nCurrent gravity: %lf N\n", -force[1]);

	do
	{
		hdScheduleSynchronous(GetDeviceStateCallback, &state, 
			HD_DEFAULT_SCHEDULER_PRIORITY);

		double currentZPos = state.position[2]; // Current shift in Z-axis in mm
		
		if (currentZPos < 0) currentZPos = 0;

		double yPos = sqrt(halfLength * halfLength - currentZPos * currentZPos);

		hduVector3Dd forcePos(yPos, 0, 0);
		hduVector3Dd originPos(0, currentZPos, 0);
		hduVector3Dd posVector;


		hduVecSubtract(posVector, forcePos, originPos);

		torqueApp = crossProduct(posVector, force);

		hdScheduleSynchronous(UpdateTorqueCallback, 0, 
			HD_DEFAULT_SCHEDULER_PRIORITY);

	} while (!_kbhit());

}

/************************************************************************
 Prints the current state of the device.
************************************************************************/
void PrintDeviceState(HDboolean bContinuous)
{
	int i;
	DeviceStateStruct state;

	memset(&state, 0, sizeof(DeviceStateStruct));

	do
	{
		hdScheduleSynchronous(GetDeviceStateCallback, &state, 
			HD_DEFAULT_SCHEDULER_PRIORITY);

		printf("\n");

		printf("Position: ");
		for (i = 0; i < 3; i++)
		{
			printf(" %f", state.position[i]);
		}

		printf("\n");

		printf("Z-axis offset: %f", state.position[2]);

		if (bContinuous)
		{
			Sleep(500);
		}

	} while(!_kbhit() && bContinuous);
}

/************************************************************************
 Check for haptic device error   
************************************************************************/
int CheckError(char* s)
{
	HDErrorInfo error;
	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
		hduPrintError(stderr, &error, s);
		fprintf(stderr, "\nPress any key to quit.\n");
		_getch();

		return 0;
	}

	return 1;
}

/************************************************************************
 The main loop of execution. Detects and interprets keypresses. Monitors
 and initiates error recovery if necessary.
************************************************************************/
void mainLoop()
{
	int keypress;

	while (TRUE)
	{
		if (_kbhit())
		{
			keypress = _getch();
			keypress = toupper(keypress);

			switch (keypress)
			{
				case 'I': SetForce(); break;
				case 'L': SetTorque(); break;
				case 'T': printf("Apply texture... To be implemented"); break;
				case 'P': PrintDeviceState(TRUE);
				case 'Q': printf("Quiting..."); return; break;
				default: PrintHelp(); break;
			}
		}

		/* Check if the scheduled callback has stopped running */
		if (!hdWaitForCompletion(gCallbackHandle, HD_WAIT_CHECK_STATUS))
		{
			fprintf(stderr, "\nThe main scheduler callback has exited\n");
			fprintf(stderr, "\nPress any key to quit.\n");
			_getch();
			return;
		}
	}
}

/************************************************************************
 Main function - entry point
************************************************************************/
int main(int argc, char* argv[])
{
	/* Device initialization */
	HDstring model;
	HDboolean dDone = FALSE;

	HHD hHD = hdInitDevice(HD_DEFAULT_DEVICE);
	
	if (!CheckError("Failed to initialize haptic device"))
	{
		hdDisableDevice(hHD);
		return -1;
	}

	model = hdGetString(HD_DEVICE_MODEL_TYPE);
	printf("Initialized: %s\n", model);

	/* Force vector initialization */
	memset(forceVecApp, 0, sizeof(double) * FORCE_DIM);
	memset(forceVecServo, 0, sizeof(double) * FORCE_DIM);

	/* Callback function initialization */
	gCallbackHandle = hdScheduleAsynchronous(
		ServoSchedulerCallback, 0, HD_MAX_SCHEDULER_PRIORITY);
	
	if (!CheckError("Failed to schedule servoloop callback"))
	{
		hdDisableDevice(hHD);
		return -1;
	}

	hdEnable(HD_FORCE_OUTPUT); // Enable device to output force

	/* Start the haptic rendering loop */
	hdStartScheduler();
	if (!CheckError("Failed to start servoloop"))
	{
		hdDisableDevice(hHD);
		return -1;
	}

	PrintHelp();

	/* Start the main application loop */
	mainLoop();

	hdStopScheduler();
	hdUnschedule(gCallbackHandle);
	hdDisableDevice(hHD);

	return 0;

}