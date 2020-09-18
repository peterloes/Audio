/***************************************************************************//**
 * @file
 * @brief	Header file of module Control.c
 * @author	Ralf Gerhauser
 * @version	2017-01-25
 ****************************************************************************//*
Revision History:
2020-02-06,feeder is MomoAudio
2017-01-25,rage	Initial version.
*/

#ifndef __INC_Control_h
#define __INC_Control_h

/*=============================== Header Files ===============================*/

#include "config.h"		// include project configuration parameters

/*=============================== Definitions ================================*/

    /*!@brief Show module "Control" exists in this project. */
#define MOD_CONTROL_EXISTS

#ifndef DFLT_PLAY_TYPE
/*!@brief Default playback types for not random 1,2,3,4,5 or random: 6,7,8,9 
     * (in numbers).*/
    #define DFLT_PLAY_TYPE		1	// not random folder T001.wav
#endif

#ifndef DFLT_KEEP_PLAYING_DURATION
    /*!@brief Default PLAYING duration for the audio module (in seconds). */
    #define DFLT_KEEP_PLAYING_DURATION	120	// 2min
#endif

#ifndef DFLT_KEEP_RECORD_DURATION
    /*!@brief Default RECORD duration for the audio module (in seconds). */
    #define DFLT_KEEP_RECORD_DURATION	240	// 4min
#endif

    /*!@brief Power output selection - keep in sync with string array
     * @ref g_enum_PowerOutput and @ref l_PwrOutDef !
     */
typedef enum
{
    PWR_OUT_NONE = NONE,	// (-1) for no output at all
    PWR_OUT_UA1,		// 0: DC/DC (3V3) at pin UA1 (X10-1)
    PWR_OUT_UA2,		// 1: DC/DC ( 5V) at pin UA2 (X10-3)
    PWR_OUT_VDD_SERVO,		// 2: BATT_INPUT at pin VDD_SERVO (X14-1)
    PWR_OUT_VDD_LINEAR,		// 3: BATT_INPUT at pin VDD_LINEAR (X15-1)
    PWR_OUT_VDD_RFID1,		// 4: 3V3 at pin VDD_RFID1 (X8-3)
    PWR_OUT_VDD_RFID2,		// 5: 3V3 at pin VDD_RFID2 (X8-5)
    PWR_OUT_VDD_RFID3,		// 6: 3V3 at pin VDD_RFID3 (X8-7)
    PWR_OUT_RFID_GND_LB,	// 7: Gnd at pin RFID_GND_LB (X4-5)
    NUM_PWR_OUT
} PWR_OUT;

    /*!@brief Power control. */
//@{
#define PWR_OFF		false	//!< Switch power output off (disable power)
#define PWR_ON		true	//!< Switch power output on  (enable power)
//@}

/*================================ Global Data ===============================*/

extern const char *g_enum_PowerOutput[];

/*================================ Prototypes ================================*/

    /* Initialize control module */
void	ControlInit (void);

    /* Clear Configuration variables (set default values) */
void	ClearConfiguration (void);


    /* Inform the control module about a new transponder ID */
void	ControlUpdateID (char *transponderID);

    /* Switch power output on or off */
void	PowerOutput	(PWR_OUT output, bool enable);
bool	IsPowerOutputOn (PWR_OUT output);

    /* Power Fail Handler of the control module */
void	ControlPowerFailHandler (void);


#endif /* __INC_Control_h */
