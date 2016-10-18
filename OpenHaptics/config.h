/*****************************************************************************

Position Dimension:

x axis: Left to right, right has larger value
y axis: Bottom to top, top has larger value
z axis: Far to near, near has larger value

*******************************************************************************/
#ifdef _WIN64
#pragma warning (disable:4996)
#endif

/* Standard function include */
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>
#if defined(WIN32)
#include <windows.h>
#include <conio.h>
#else
#include <time.h>
#include "conio.h"
#define FALSE 0
#define TRUE 1
#endif

/* Haptics library include */
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>

/* Geometric Touch specifications */
#define MAX_INPUT_DOF 6
#define MAX_OUTPUT_DOF 3
#define FORCE_DIM 3
#define TORQUE_DIM 3
#define GRAVITY_ACC 9.781
#define ADJUST_FACTOR 1 / 0.67 // linear adjust scale for accurate weight generation
#define ADJUST_OFFSET 53

/* Force vectors */
hduVector3Dd forceVecApp;
hduVector3Dd forceVecServo;

/* Torque vectors */
static const hduVector3Dd nominalBaseTorque(200.0, 350.0, 200.0);
hduVector3Dd torqueApp;
hduVector3Dd torqueServo;

/* Haptic device handler */
HDSchedulerHandle gCallbackHandle = HD_INVALID_HANDLE; 

/* Synchronization structure */
typedef struct
{
	hduVector3Dd position;

} DeviceStateStruct;

/* Forward declaration */
void PrintHelp(void);
HDCallbackCode HDCALLBACK ServoSchedulerCallback(void);
HDCallbackCode HDCALLBACK UpdateForceCallback(void);
HDCallbackCode HDCALLBACK UpdateTorqueCallback(void);
HDCallbackCode HDCALLBACK GetDeviceStateCallback(void);
void SetForce(void);
void SetTorque(void);
void PrintDeviceState(HDboolean); // debug only
int CheckError(char*);
void mainLoop(void);