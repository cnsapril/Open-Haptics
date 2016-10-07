#ifdef _WIN64
#pragma warning (disable:4996)
#endif

/* Standard function include */
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
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
