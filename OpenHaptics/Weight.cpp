/*****************************************************************************

Position Dimension:

x axis: Left to right, right has larger value
y axis: Bottom to top, top has larger value
z axis: Far to near, near has larger value

*******************************************************************************/
/*#ifdef  _WIN64
#pragma warning (disable:4996)
#endif

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#if defined(WIN32)
# include <windows.h>
# include <conio.h>
#else
#include<time.h>
#include "conio.h"
#define FALSE 0
#define TRUE 1
#endif

#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>*/

/* Assumes the unit is a PHANTOM from SensAble Technologies, Inc. 
#define MAX_INPUT_DOF   6   
#define MAX_OUTPUT_DOF  6

#define FORCE_DIM 3
#define GRAVITY 9.781
#define ADJUST_FACTOR 1 / 0.67
#define ADJUST_OFFSET 53

static int gNumMotors = 0;
static int gNumEncoders = 0;

static long alMotorDACValuesApp[MAX_OUTPUT_DOF];
static long alMotorDACValuesServo[MAX_OUTPUT_DOF];
static double forceApp[FORCE_DIM];
static double forceServo[FORCE_DIM];

void PrintHelp()
{
    static const char help[] = {\
"Weight Measure\n\
---\n\
I: Input weight\n\
P: Prints device state\n\
C: Continuously prints device state\n\
Q: Quits the program\n\
---"};
    
    printf("\n%s\n", help);
}

HDSchedulerHandle gCallbackHandle = HD_INVALID_HANDLE;

void mainLoop();

/*****************************************************************************
 Prints DAC values.
*****************************************************************************/
/*void PrintDACValues()
{
    int i;

    printf("Motor DAC Values:");
    for (i = 0; i < gNumMotors; i++)
    {
        printf(" %d", alMotorDACValuesApp[i]);
    }
    printf("\n\n");
}

/*****************************************************************************
 Directly sets the DAC values.
*****************************************************************************/
/*HDCallbackCode HDCALLBACK ServoSchedulerCallback(void *pUserData)
{
    HDErrorInfo error;

    hdBeginFrame(hdGetCurrentDevice());    
    
    assert(alMotorDACValuesServo);

    //hdSetLongv(HD_CURRENT_MOTOR_DAC_VALUES, alMotorDACValuesServo);

	hdSetDoublev(HD_CURRENT_FORCE, forceServo);

    hdEndFrame(hdGetCurrentDevice());

    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Error while applying forces");
        memset(alMotorDACValuesServo, 0, sizeof(long) * MAX_OUTPUT_DOF);

        if (hduIsSchedulerError(&error))
        {
            return HD_CALLBACK_DONE;
        }
    }

    return HD_CALLBACK_CONTINUE;
}

/*****************************************************************************
 Copies state in thread-safe manner.
*****************************************************************************/
/*HDCallbackCode HDCALLBACK UpdateMotorDACValuesCallback(void *pUserData)
{
    memcpy(forceServo,
           forceApp,
           sizeof(double) * FORCE_DIM);

    return HD_CALLBACK_DONE;
}

/*****************************************************************************
 Selects motor.
*****************************************************************************/
/*void MotorSelection(int *pMotorIndex)
{    
    do    
    {
        printf("\nEnter the motor index [0 - %d]: ", gNumMotors - 1);
        *pMotorIndex = _getch() - '0';
    }
    while (*pMotorIndex < 0 || *pMotorIndex > gNumMotors - 1);

    printf("\nMotor index selected: %d\n", *pMotorIndex);
}*/

/*****************************************************************************
 Schedules the UpdateMotorDACValuesCallback.
*****************************************************************************/
/*void SetDACValue(int nMotorIndex)
{
    double nValue;    

    printf("\nSet the weight: between 0 and 337g: [0 to 337 ]\n");
    do
    {
	printf("Enter value and press return: ");
#if defined(linux)
	    restore_term();
#endif
	scanf("%lf", &nValue);
    } 
    while (nValue < 0 || nValue > 337);

    double adjusted_mass = (nValue * ADJUST_FACTOR - ADJUST_OFFSET) / 1000.0;

	forceApp[1] = -(adjusted_mass * GRAVITY);

    PrintDACValues();

    hdScheduleSynchronous(UpdateMotorDACValuesCallback,
        0, HD_DEFAULT_SCHEDULER_PRIORITY);
}

/* Synchronization structure. */
/*typedef struct
{
    HDlong encoder_values[MAX_INPUT_DOF];
    HDlong motor_dac_values[MAX_OUTPUT_DOF];   
    hduVector3Dd position;
} DeviceStateStruct;

/*****************************************************************************
 Callback that retrieves state.
*****************************************************************************/
/*HDCallbackCode HDCALLBACK GetDeviceStateCallback(void *pUserData)
{
    DeviceStateStruct *pState = (DeviceStateStruct *) pUserData;

    hdGetLongv(HD_CURRENT_ENCODER_VALUES, pState->encoder_values);
    hdGetLongv(HD_CURRENT_MOTOR_DAC_VALUES, pState->motor_dac_values);
    hdGetDoublev(HD_CURRENT_POSITION, pState->position);

    return HD_CALLBACK_DONE;
}

/*****************************************************************************
 Callback that retrieves state.
*****************************************************************************/
/*void PrintDeviceState(HDboolean bContinuous)
{
    int i;
    DeviceStateStruct state;

    memset(&state, 0, sizeof(DeviceStateStruct));

    do
    {
        hdScheduleSynchronous(GetDeviceStateCallback, &state,
            HD_DEFAULT_SCHEDULER_PRIORITY);

        printf("\n");

        printf("Motor DAC Values:");
        for (i = 0; i < gNumMotors; i++)
        {
            printf(" %d", state.motor_dac_values[i]);
        }
        printf("\n");

        printf("Encoder Values:");
        for (i = 0; i < gNumEncoders; i++)
        {
            printf(" %d", state.encoder_values[i]);
        }
        printf("\n");

        printf("Position:");
        for (i = 0; i < 3; i++)
        {
            printf(" %f", state.position[i]);
        }
        printf("\n");

		printf("z-axis offset: %f", state.position[2]);

        if (bContinuous)
        {
#if defined(WIN32)
            Sleep(500);
#elif defined(linux)
		struct timespec timeOut;
		timeOut.tv_sec = 0;
		timeOut.tv_nsec = 5*100000000;
		nanosleep(&timeOut, NULL);
#endif
        }

    } while (!_kbhit() && bContinuous);
}



/*******************************************************************************
 Main function.
*******************************************************************************/
/*int main(int argc, char* argv[])
{  
    HDErrorInfo error;
    HDstring model;
    HDboolean bDone = FALSE;

    HHD hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        fprintf(stderr, "\nPress any key to quit.\n");
        _getch();

        return -1;
    }

    model = hdGetString(HD_DEVICE_MODEL_TYPE);
    printf("Initialized: %s\n", model);

    hdGetIntegerv(HD_OUTPUT_DOF, &gNumMotors);
    hdGetIntegerv(HD_INPUT_DOF, &gNumEncoders);

    memset(alMotorDACValuesApp, 0, sizeof(long) * MAX_OUTPUT_DOF);
    memset(alMotorDACValuesServo, 0, sizeof(long) * MAX_OUTPUT_DOF);
	memset(forceApp, 0, sizeof(double) * FORCE_DIM);
	memset(forceServo, 0, sizeof(double) * FORCE_DIM);

    #if defined(linux)
	nocbreak();
    #endif

    /* Schedule the haptic callback function for continuously monitoring the
       button state and rendering the anchored spring force */
 /*   gCallbackHandle = hdScheduleAsynchronous(
        ServoSchedulerCallback, 0, HD_MAX_SCHEDULER_PRIORITY);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to schedule servoloop callback");
        fprintf(stderr, "\nPress any key to quit.\n");
        _getch();

        hdDisableDevice(hHD);
        return -1;
    }

    hdEnable(HD_FORCE_OUTPUT);

	if (hdCheckCalibration() == HD_CALIBRATION_OK)
	{
		printf("Device calibrated.");
	}

    /* Start the haptic rendering loop */
/*    hdStartScheduler();
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to start servoloop");
        fprintf(stderr, "\nPress any key to quit.\n");
        _getch();

        hdDisableDevice(hHD);
        return -1;        
    }

    PrintHelp();

    /* Start the main application loop */
/*    mainLoop();

    hdStopScheduler();
    hdUnschedule(gCallbackHandle);
    hdDisableDevice(hHD);

    return 0;
}

/******************************************************************************
 The main loop of execution.  Detects and interprets keypresses.  Monitors and 
 initiates error recovery if necessary.
******************************************************************************/
/*void mainLoop()
{
    int keypress;
    int nMotorIndex = 0;

    while (TRUE)
    {
        if (_kbhit())
        {
            keypress = _getch();
            keypress = toupper(keypress);
            
            switch (keypress)
            {
                case 'I': SetDACValue(nMotorIndex); break;
                case 'P': PrintDeviceState(FALSE); break;
                case 'C': PrintDeviceState(TRUE); break;
                case 'Q': return;
                default: PrintHelp(); break;
            }
        }

        /* Check if the scheduled callback has stopped running */
/*        if (!hdWaitForCompletion(gCallbackHandle, HD_WAIT_CHECK_STATUS))
        {
            fprintf(stderr, "\nThe main scheduler callback has exited\n");
            fprintf(stderr, "\nPress any key to quit.\n");
            _getch();
            return;
        }
    }
}

/*****************************************************************************/
