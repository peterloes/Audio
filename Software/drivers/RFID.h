/***************************************************************************//**
 * @file
 * @brief	Header file of module RFID.c
 * @author	Ralf Gerhauser / Peter Loes
 * @version	2020-07-27
 ****************************************************************************//*
Revision History:
2018-03-26,rage - RFID_TRIGGERED_BY_LIGHT_BARRIER lets you select whether the
		- RFID reader is controlled by light-barriers or alarm times.
                - Added prototypes for IsRFID_Active() and IsRFID_Enabled().
2017-06-20,rage	Defined RFID_TYPE, extended RFID_CONFIG with it.
2017-05-02,rage	RFID_Init: Added RFID_PWR_OUT structure and NUM_RFID_READER.
2016-02-24,rage	Added prototype for RFID_PowerOff().
2014-11-25,rage	Initial version.
*/

#ifndef __INC_RFID_h
#define __INC_RFID_h

/*=============================== Header Files ===============================*/

#include <stdio.h>
#include <stdbool.h>
#include "em_device.h"
#include "config.h"		// include project configuration parameters
#include "Control.h"


/*=============================== Definitions ================================*/

    /*!@brief Duration in [s] during the RFID reader tries to read an ID. */
#ifndef DFLT_RFID_DETECT_TIMEOUT
    #define DFLT_RFID_DETECT_TIMEOUT		10
#endif


    /*!@brief RFID types. */
typedef enum
{
    RFID_TYPE_NONE = NONE,	// (-1) for no RFID at all
    RFID_TYPE_SR,		// 0: Short Range RFID reader
    RFID_TYPE_LR,		// 1: Long Range RFID reader
    NUM_RFID_TYPE
} RFID_TYPE;

/*!@brief Structure to specify the type of RFID readers and the power outputs */
typedef struct
{
    RFID_TYPE		RFID_Type;		//!< RFID type selection
    PWR_OUT		RFID_PwrOut;		//!< Power output selection
} RFID_CONFIG;

/*================================ Global Data ===============================*/

extern int32_t	 g_RFID_DetectTimeout;

extern RFID_TYPE g_RFID_Type;
extern PWR_OUT	 g_RFID_Power;
extern uint32_t	 g_RFID_AbsentDetectTimeout;
extern const char *g_enum_RFID_Type[];
extern char	 g_Transponder[18];

/*================================ Prototypes ================================*/

    /* Initialize the RFID module */
void	RFID_Init (void);

    /* Check if RFID reader is active */
bool	IsRFID_Active (void);

    /* Enable RFID reader */
void	RFID_Enable (void);

    /* Enable RFID Power reader */
void RFIDPower_Enable(void);


    /* Disable RFID reader */
void	RFID_Disable (void);

    /* Check if RFID reader is enabled */
bool	IsRFID_Enabled (void);

    /* Check if to power-on/off RFID reader, get tranponder number */
void	RFID_Check (void);

    /* Power RFID reader Off */
void	RFID_PowerOff (void);

    /* RFID Power Fail Handler */
void	RFID_PowerFailHandler (void);


#endif /* __INC_RFID_h */
