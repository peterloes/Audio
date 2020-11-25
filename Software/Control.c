/***************************************************************************//**
 * @file
 * @brief	Sequence Control
 * @author      Ralf Gerhauser / Peter Loes  
 * @version	2018-08-03
 *
 * This is the automatic sequence control module.  It controls the power
 * outputs that may be activated via alarm times @ref alarm_times.
 * It controls peripheral units like Audio module playback & record.
 * This module also defines the configuration variables for the file
 * <a href="../../CONFIG.TXT"><i>CONFIG.TXT</i></a>.
 *
 ****************************************************************************//*
Revision History:
2020-01-03,rage	- Start and Stop Playback & Record
2017-05-02,rage	- ControlInit: Added CONTROL_INIT structure to specify the
		  power output.
		- PWR_OUT_DEF contains the bit address and logical enable level
		  for all power output pins.
		- ExtIntEnableAll() is called when switching the MOMOAudio on again
		  to consider the current state of all EXTI interrupt inputs.
		- Implemented PowerOutputs().
		- Removed CAM1 and CAM2 routines.
2017-01-25,rage	Initial version.
*/

/*=============================== Header Files ===============================*/


#include "em_cmu.h"
#include "em_int.h"
#include "em_gpio.h"
#include "ExtInt.h"
#include "LEUART.h"
#include "Logging.h"
#include "LightBarrier.h"
#include "AlarmClock.h"
#include "RFID.h"
#include "Audio.h"
#include "CfgData.h"
#include "Control.h"
#include "PowerFail.h"


/*=============================== Definitions ================================*/

    /*!@brief Structure to define power outputs. */
typedef struct
{
    __IO uint32_t *BitBandAddr;	// Bit band address of GPIO power enable pin
    bool	   HighActive;	// true: The power enable pin is high-active
} PWR_OUT_DEF;

    /*!@brief Macro to calculate a GPIO bit address for a port and pin. */
#define GPIO_BIT_ADDR(port, pin)					\
	IO_BIT_ADDR((&(GPIO->P[(port)].DOUT)), (pin))

/*!@brief Macro to extract the port number from a GPIO bit address.  */
#define GPIO_BIT_ADDR_TO_PORT(bitAddr)		(GPIO_Port_TypeDef)	\
	(((((uint32_t)(bitAddr) - BITBAND_PER_BASE) >> 5)		\
	 + PER_MEM_BASE - GPIO_BASE) / sizeof(GPIO_P_TypeDef))

    /*!@brief Macro to extract the pin number from a GPIO bit address.  */
#define GPIO_BIT_ADDR_TO_PIN(bitAddr)					\
	(((uint32_t)(bitAddr) >> 2) & 0x1F)
          
/*================================ Global Data ===============================*/

    /*!@brief CFG_VAR_TYPE_ENUM_2: Enum names for Power Outputs. */
const char *g_enum_PowerOutput[] = { "UA2", "UA", NULL };
          
/*================================ Local Data ================================*/

    /*!@brief Power output port and pin assignment - keep in sync with enums
     * @ref PWR_OUT and string array @ref g_enum_PowerOutput !
     */
static const PWR_OUT_DEF  l_PwrOutDef[NUM_PWR_OUT] =
{   //   BitBandAddr,              HighActive
    { GPIO_BIT_ADDR(gpioPortA,  4), true },	// PWR_OUT_UA2
    { GPIO_BIT_ADDR(gpioPortA,  6), true },	// PWR_OUT_UA
};

    /*!@brief Default value of the playback_type, set by PLAYBACK_TYPE. */
static int32_t		l_dfltPlayType = DFLT_PLAY_TYPE;

    /*!@brief Actual duration of the playback_type, set by ID. */
static int32_t		l_PlayType = DFLT_PLAY_TYPE;

    /*!@brief Default keep playback duration, set by PLAYBACK. */
static int32_t		l_dfltKeepPlayback = DFLT_KEEP_PLAYING_DURATION;

    /*!@brief Actual keep playback duration, set by ID. */
static int32_t		l_KeepPlayback = DFLT_KEEP_PLAYING_DURATION;

    /*!@brief Default keep Record duration, set by RECORD. */
static int32_t		l_dfltKeepRecord = DFLT_KEEP_RECORD_DURATION;

    /*!@brief Actual keep Record duration, set by ID. */
static int32_t		l_KeepRecord = DFLT_KEEP_RECORD_DURATION;

    /*!@brief List of configuration variables.
     * Alarm times, i.e. @ref CFG_VAR_TYPE_TIME must be defined first, because
     * the array index is used to specify the alarm number \<alarmNum\>,
     * starting with @ref alarm_times, when calling AlarmSet().
     */
static const CFG_VAR_DEF l_CfgVarList[] =
{
 { "ON_TIME_1",		       CFG_VAR_TYPE_TIME,	NULL		},
 { "OFF_TIME_1",	       CFG_VAR_TYPE_TIME,	NULL		},
 { "LB_FILTER_DURATION",       CFG_VAR_TYPE_INTEGER,    &g_LB_FilterDuration  },
 { "RFID_TYPE",		       CFG_VAR_TYPE_ENUM_1,	&g_RFID_Type	},
 { "RFID_POWER",	       CFG_VAR_TYPE_ENUM_2,	&g_RFID_Power	},
 { "RFID_DETECT_TIMEOUT",      CFG_VAR_TYPE_INTEGER,	&g_RFID_DetectTimeout },
 { "AUDIO_POWER",              CFG_VAR_TYPE_ENUM_2,     &g_AudioPower	},
 { "AUDIO_CFG_VC",             CFG_VAR_TYPE_INTEGER,	&g_AudioCfg_VC	},
 { "AUDIO_CFG_ST",             CFG_VAR_TYPE_INTEGER,	&g_AudioCfg_ST	},
 { "AUDIO_CFG_IM",             CFG_VAR_TYPE_INTEGER,	&g_AudioCfg_IM	},
 { "AUDIO_CFG_RQ",             CFG_VAR_TYPE_INTEGER,	&g_AudioCfg_RQ	},
 { "PLAYBACK",                 CFG_VAR_TYPE_DURATION,	&l_dfltKeepPlayback },
 { "RECORD",	               CFG_VAR_TYPE_DURATION,	&l_dfltKeepRecord   },
 { "PLAYBACK_TYPE",            CFG_VAR_TYPE_INTEGER,	&l_dfltPlayType     },
 { "ID",                       CFG_VAR_TYPE_ID,	        NULL	            },
 {  NULL,                      END_CFG_VAR_TYPE,        NULL		    }
};

/*!@brief Timer handle to keep the playback or record for a while. */
static volatile TIM_HDL	l_hdlPlayRec = NONE;

    /*!@brief List of all enum definitions. */
static const ENUM_DEF l_EnumList[] =
{
    g_enum_RFID_Type,		// CFG_VAR_TYPE_ENUM_1
    g_enum_PowerOutput,		// CFG_VAR_TYPE_ENUM_2
};

    /*!@brief Flag if playback should run. */
static volatile bool	l_flgPlaybackRun = true;

    /*!@brief Flag if playback is currently run. */
static volatile bool	l_flgPlaybackIsRun;

    /*!@brief Current state of the AudioRfidPower: true means ON, false means OFF. */
static volatile bool	l_flgAudioRfidPower;	// is false for default

/*!@brief If current ID appears twice lock: true means locked, false means unlocked. */
static volatile bool	l_flgTwiceIDLocked;	// is false for default
/*=========================== Forward Declarations ===========================*/

static void	PlayRecAction (TIM_HDL hdl);
static void	PlaybackRun (void);
static void	RecordRun (void);

static void	PowerControl (int alarmNum);

    /*!@brief Current state of the AudioPlaybackRun: true means ON, false means OFF. */
static volatile bool	l_flgAudioPlayRun;	// is false for default

    /*!@brief Current state of the AudioPlaybackStop: true means ON, false means OFF. */
static volatile bool	l_flgAudioPlayStop; // is false for default

    /*!@brief Current state of the AudiRecordRun: true means ON, false means OFF. */
static volatile bool	l_flgAudioRecRun; // is false for default

/*!@brief Current state of the AudiRecordStop: true means ON, false means OFF. */
static volatile bool	l_flgAudioRecStop; // is false for default

/*!@brief Current state of the PlaybackType:  1 to 9. */
static volatile int	AudioPlaybackType; // is <= 9 


/***************************************************************************//**
 *
 * @brief	Initialize control module
 *
 * This routine initializes the sequence control module.
 *
 ******************************************************************************/
void	ControlInit (void)
{
int	i;

    /* Introduce variable list to configuration data module */
    CfgDataInit (l_CfgVarList, l_EnumList);

    /* Initialize power output enable pins */
    for (i = 0;  i < NUM_PWR_OUT;  i++)
    {
	/* Configure Power Enable Pin, switch it OFF per default */
	GPIO_PinModeSet (GPIO_BIT_ADDR_TO_PORT(l_PwrOutDef[i].BitBandAddr),
			 GPIO_BIT_ADDR_TO_PIN (l_PwrOutDef[i].BitBandAddr),
			 gpioModePushPull, l_PwrOutDef[i].HighActive ? 0:1);
    }

    /* Get a timer handle to playback or record audio files for a while */
    if (l_hdlPlayRec == NONE)
	l_hdlPlayRec = sTimerCreate (PlayRecAction);
    
    /* Use same routine for all power-related alarms */
    for (i = FIRST_POWER_ALARM;  i <= LAST_POWER_ALARM;  i++)
	AlarmAction (i, PowerControl);

    /* Initialize configuration with default values */
    ClearConfiguration();
}


/***************************************************************************//**
 *
 * @brief	Clear Configuration
 *
 * This routine disables all alarm times and switches the corresponding power
 * outputs off.  It then sets all configuration variables to default values.
 * It must be executed <b>before</b> calling CfgRead() to to ensure the correct
 * settings for variables which are <b>not</b> set within a new configuration.
 *
 ******************************************************************************/
void	ClearConfiguration (void)
{
int	i;

    /* Disable all power-related alarms */
    for (i = FIRST_POWER_ALARM;  i <= LAST_POWER_ALARM;  i++)
    {
	if (AlarmIsEnabled(i))
	{
	    if (i >= ALARM_OFF_TIME)
		ExecuteAlarmAction(i);	// OFF-Time: Switch device off

	    AlarmDisable(i);		// Disable this alarm
	}
    }
    
    /* Disable RFID functionality */
    g_RFID_Type = RFID_TYPE_NONE;
    g_RFID_Power = PWR_OUT_NONE;
    l_flgTwiceIDLocked = false;
    
    /* Disable Audio functionality */
    g_AudioPower = PWR_OUT_NONE;
    g_AudioCfg_VC = 0;
    g_AudioCfg_ST = 0;
    g_AudioCfg_IM = 0;
    g_AudioCfg_RQ = 0;
    
}


/***************************************************************************//**
 *
 * @brief	Determine if AlarmTime ON / off 
 *
 * The Audio module and the RFID reader can be switched on and off by
 * setting alarm times.  This routine is used to determine if the AlarmTime
 * is currently on.
 *
 * @return
 * 	The value <i>true</i> if the Audio module
 *      and the RFID reader is on, <i>false</i> if not. See Lightbarrier.c
 *
 ******************************************************************************/
bool	IsAudioRfidOn (void)
{
    return l_flgAudioRfidPower;
}


/***************************************************************************//**
 *
 * @brief	Inform the control module about a new transponder ID
 *
 * This routine must be called to inform the control module about a new
 * transponder ID.
 *
 * @warning
 * This function calls a blocking delay routine, therefore it must not be
 * called from interrupt context!
 *
 ******************************************************************************/
void	ControlUpdateID (char *transponderID)
{
char	 line[120];
char	*pStr;
ID_PARM	*pID;

    /*!@brief Current state of of Audio Module is locked from audio.c */
static volatile bool	 isAudioLocked;
isAudioLocked  = IsAudioLocked();

    pStr = line;
    if(!isAudioLocked || !l_flgTwiceIDLocked)
    {      
    pID = CfgLookupID (transponderID);
    }
    if (pID == NULL)
    {
	/* specified ID not found, look for an "ANY" entry */
	pID = CfgLookupID ("ANY");
	if (pID == NULL)
	{
	    /* no "ANY" entry defined, treat ID as "UNKNOWN" */
	    pID = CfgLookupID ("UNKNOWN");
	    if (pID == NULL)
	    {
		/* even no "UNKNOWN" entry exists - abort */
		Log ("Transponder: %s not found - aborting", transponderID);
		return;
	    }
	    else
	    {
		pStr += sprintf (pStr, "Transponder: %s not found -"
				 " using UNKNOWN", transponderID);
	    }
	}
	else
	{
	    pStr += sprintf (pStr, "Transponder: %s not found -"
			     " using ANY", transponderID);
	}
    }
    else
    {
	pStr += sprintf (pStr, "Transponder: %s", transponderID);
    }

    if(!isAudioLocked || !l_flgTwiceIDLocked)
    { 
    /* prepare the associated variables */
    l_KeepPlayback = (pID->KeepPlayback == DUR_INVALID	? l_dfltKeepPlayback
							: pID->KeepPlayback);
    l_KeepRecord   = (pID->KeepRecord == DUR_INVALID	? l_dfltKeepRecord
							: pID->KeepRecord);
    l_PlayType     = (pID->PlayType  == DUR_INVALID	? l_dfltPlayType
							: pID->PlayType);
  
       /* append current parameters to ID */
       pStr += sprintf (pStr, ":%ld", l_KeepPlayback);
       pStr += sprintf (pStr, ":%ld", l_KeepRecord);
       pStr += sprintf (pStr, ":%ld", l_PlayType);
       
       l_flgTwiceIDLocked = true;
    }
    else
    {
       pStr += sprintf (pStr, " - Audio: Is locked"); 
    }
#ifdef LOGGING
     Log(line);
#endif    
  
       /* playback or record (may already be done) */
       if (l_KeepPlayback > 0)
       {
           /*
	    * A KEEP_PlAYBACK value of 1..n or "A" ANY means there was a transponder detected
	    */
               /* be sure to cancel PlayRec timer */
 	   if (l_hdlPlayRec != NONE)
           { 	
	      if(!isAudioLocked)
              {
                 /* start playing */
	         PlaybackRun();
                 sTimerStart (l_hdlPlayRec, l_KeepPlayback);
              }
           }
        }
        else
        {
          /* be sure to cancel PlayRec timer */
 	  if (l_hdlPlayRec != NONE)
          { 	
	    /* it follows a KEEP_RECORD duration */
	    if (l_KeepRecord > 0)
	    {
               if(!isAudioLocked)
               {
                   /* start record */
	           RecordRun(); 
                  /* start KeepRecord timer */
                  sTimerStart (l_hdlPlayRec, l_KeepRecord);
               }
            }
          }
      }
}


/***************************************************************************//**
 *
 * @brief	Playback & Record action
 *
 * This routine is called after the programmed KEEP_PLAYBACK or KEEP_RECORD
 * duration to initiate a playback or record action for the audio module.
 *
 ******************************************************************************/
static void	PlayRecAction (TIM_HDL hdl)
{
(void) hdl;		// suppress compiler warning "unused parameter"
    
        /*!@brief Current state of of Audio Module is locked from audio.c */
static volatile bool	 isAudioLocked;
isAudioLocked  = IsAudioLocked(); 

    /* determine current state */
    if (l_flgPlaybackRun)
    {
	/* playback time is over playback stop */
	l_flgPlaybackIsRun = false;
 
        /* Deactivate timer */
	if (l_hdlPlayRec != NONE)
	    sTimerCancel (l_hdlPlayRec);
        
        /* Playback stop has been set - inform Audio module 
         * via IsControlPlayStop */
        l_flgAudioPlayStop = true;
        l_flgAudioPlayRun = false;
        
        /* it follows a KEEP_RECORD duration */
	if (l_KeepRecord > 0)
	{
           /* start record */
	    RecordRun();
            /* start KeepRecord timer */
            sTimerStart (l_hdlPlayRec, l_KeepRecord);
        }
        l_flgPlaybackRun = false;
        l_flgTwiceIDLocked = false;
	
    }
    else 
    {
	/* record is run */
	l_flgPlaybackRun = true;
        
        /* Deactivate timer */
        if (l_hdlPlayRec != NONE)
	    sTimerCancel (l_hdlPlayRec);
        
         /* Record stop has been set - inform Audio module 
          * via IsControlRecStop */
         l_flgAudioRecStop = true;
         l_flgAudioRecRun = false;
         l_flgTwiceIDLocked = false;
    }
}


/***************************************************************************//**
 *
 * @brief	PlaybackRun for Control.c
 *
 * This routine initiates to playback sounds. This is done by setting
 * int AudioPlaybackType and l_flgAudioPlayRun. 
 *
 ******************************************************************************/
static void	PlaybackRun (void)
{
    l_flgPlaybackRun = true;
    
    if (! l_flgPlaybackIsRun) 
    {
        l_flgPlaybackIsRun = true;
       
        /* Playback run with a new Playback_Type has been set - inform Audio module 
         * via IsControlPlaybackType(), IsControlPlayRun */
        AudioPlaybackType = l_PlayType;
        l_flgAudioPlayRun = true;
        l_flgAudioPlayStop = false;
    }    
}


/***************************************************************************//**
 *
 * @brief	RecordRun for Control.c
 *
 * This routine initiates to record sounds. This is done by setting
 * l_flgAudioRecRun.
 *
 ******************************************************************************/
static void	RecordRun (void)
{
   l_flgPlaybackRun = false;
   
   if (!l_flgPlaybackIsRun)
   {
       l_flgPlaybackIsRun = false;
        
       /* Record run has been set - inform Audio module via IsControlRecRun */
       l_flgAudioRecRun = true;
       l_flgAudioRecStop = false;
   }
}


/***************************************************************************//**
 *
 * @brief	Determine if Audio Playback RUN  
 *
 * This routine is used to determine if playback run is currently on. See Audio.c
 *
 * @return
 * 	The value <i>true</i> if the Audio module playback is on,
 *      <i>false</i> if not.
 *
 ******************************************************************************/
bool	IsControlPlayRun ()
{
    return l_flgAudioPlayRun;
}


/***************************************************************************//**
 *
* @brief	Determine if Audio Playback_Type   
 *
 * This routine is used to determine new playback_type from 1 to 9. See Audio.c
 *
 * @return
 *     The value <i>1<=9</i> if the Audio module playback set.
 *
 ******************************************************************************/
int	IsControlPlaybackType ()
{
    return AudioPlaybackType;
}


/***************************************************************************//**
 *
 * @brief	Determine if Audio Record RUN  
 *
 * This routine is used to determine if record run is currently on. See Audio.c
 *
 * @return
 * 	The value <i>true</i> if the Audio module record is on,
 *      <i>false</i> if not. 
 *
 ******************************************************************************/
bool	IsControlRecRun (void)
{
    return l_flgAudioRecRun;
}


/***************************************************************************//**
 *
 * @brief	Determine if Audio Playback Stop  
 *
 * This routine is used to determine if playback stop is currently on.See Audio.c
 *
 * @return
 * 	The value <i>true</i> if the Audio module playback stop is on,
 *      <i>false</i> if not
 *
 ******************************************************************************/
bool	IsControlPlayStop (void)
{
    return l_flgAudioPlayStop;
}


/***************************************************************************//**
 *
 * @brief	Determine if Audio Record Stop  
 *
 * This routine is used to determine if record stop currently on.See Audio.c
 *
 * @return
 * 	The value <i>true</i> if the Audio module record stop is on,
 *      <i>false</i> if not.
 *
 ******************************************************************************/
bool	IsControlRecStop (void)
{
    return l_flgAudioRecStop;
}


/***************************************************************************//**
 *
 * @brief	Power-Fail Handler for Control Module
 *
 * This function will be called in case of power-fail to switch off devices
 * that consume too much power, e.g. the camera.
 *
 ******************************************************************************/
void	ControlPowerFailHandler (void)
{
int	i;

    if (l_hdlPlayRec != NONE)
	sTimerCancel (l_hdlPlayRec);

#ifdef LOGGING
    /* Generate Log Message */
    Log ("Switching all power outputs OFF");
#endif

    /* Switch off all power outputs immediately */
    for (i = 0;  i < NUM_PWR_OUT;  i++)
	PowerOutput ((PWR_OUT)i, PWR_OFF);
}


/******************************************************************************
 *
 * @brief	Switch the specified power output on or off
 *
 * This routine enables or disables the specified power output.
 *
 * @param[in] output
 *	Power output to be changed.
 *
 * @param[in] enable
 *	If true (PWR_ON), the power output will be enabled, false (PWR_OFF)
 *	disables it.
 *
 *****************************************************************************/
void	PowerOutput (PWR_OUT output, bool enable)
{
    
    /* Parameter check */
    if (output == PWR_OUT_NONE)
	return;		// power output not assigned, nothing to be done

    if ((PWR_OUT)0 > output  ||  output >= NUM_PWR_OUT)
    {
#ifdef LOGGING
	/* Generate Error Log Message */
	LogError ("PowerOutput(%d, %d): Invalid output parameter",
		  output, enable);
#endif
	return;
    }

    /* See if Power Output is already in the right state */
    if ((bool)*l_PwrOutDef[output].BitBandAddr == enable)
	return;		// Yes - nothing to be done

    /* Switch power output on or off */
    *l_PwrOutDef[output].BitBandAddr = enable;

#ifdef LOGGING
    Log ("Power Output %s %sabled",
	 g_enum_PowerOutput[output], enable ? "en":"dis");
#endif
}


/******************************************************************************
 *
 * @brief	Determine if the specified power output is switched on
 *
 * This routine determines the current state of a power output.
 *
 * @param[in] output
 *	Power output to be checked.
 *
 *****************************************************************************/
bool	IsPowerOutputOn (PWR_OUT output)
{
    /* Parameter check */
    if (output == PWR_OUT_NONE)
	return false;	// power output not assigned, return false (off)

    EFM_ASSERT ((PWR_OUT)0 <= output  &&  output < NUM_PWR_OUT);

    /* Determine the current state of this power output */
    return (*l_PwrOutDef[output].BitBandAddr ? true : false);
}


/***************************************************************************//**
 *
 * @brief	Alarm routine for Power Control
 *
 * This routine is called when one of the power alarm times has been reached.
 * The alarm number is an enum value @ref ALARM_ON_TIME_1.<br>
 * When an RFID reader has been installed, the function decides whether to
 * call RFID_Enable(), or RFID_Disable().  If Audio module has been configured,
 * this will also be switched on or off together the RFID reader.
 *
 ******************************************************************************/
static void	PowerControl (int alarmNum)
{
int	 pwrState;

    /* Parameter check */
    EFM_ASSERT (FIRST_POWER_ALARM <= alarmNum
		&& alarmNum <= LAST_POWER_ALARM);

    /* Determine switching state */
    pwrState = (alarmNum >= ALARM_OFF_TIME ? PWR_OFF:PWR_ON);

    /* RFID reader and Audio module are always switched on or off together */
    if (pwrState == PWR_ON)
    {
        l_flgAudioRfidPower = true;
        RFIDPower_Enable();
        AudioEnable();
    }
    else
    {
       l_flgAudioRfidPower = false;
       l_flgTwiceIDLocked = false;
       RFID_Disable();
       AudioDisable();
    }
    g_flgIRQ = true;	// keep on running
}
