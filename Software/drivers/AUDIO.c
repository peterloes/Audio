/***************************************************************************//**
 * @file
 * @brief	AUDIO
 * @author	Peter Loes
 * @version	2019-11-12
 *
 * This module provides the functionality to communicate with the AUDIO module.
 * It contains the following parts:
 * - USART driver to transmit and receive data from the Audio module.
 * - Handler for the received data
 * - Power management for FN-RM01 MP3 Audio Recorder and USART
 *
 * After powering up the Audio module, the following actions are performed:
 * -# Initialization of USART and the related Rx and Tx pins.
 * -# Configuring the Rx pin with a pull-up resistor to prevent the basic state
 *    of the Rx signal to go low again.
 * -# Waiting @ref POWER_UP_DELAY seconds for Audio module being ready.
 * -# After Audio module power up prompt 0xCAxx is send from Audio module
 *    MicroSD Card is inserted.
 * -# Sending <b>0x7E,0x03,0xC2,0xC5,0x7E</b> command
 *    4.4.6 Current work status
 * -# Sending <b>0x7E,0x03,0xC5,0xC8,0x7E</b> command 
 *     4.4.3 Total file numbers on SD card or USB flash
 * -# Sending <b>0x7E,0x03,0xCE,0xD1,0x7E</b> command
 *    4.4.9 Space left in the storage device
 * -# Sending <b>0x7E,0x04,0xAE,g_AudioCfg_VC,checksum_int,0x7E</b> command
 *    4.3.9. Volume control 1 to 31
 * -# Sending <b>0x7E,0x04,0xD2,0x01,0xD7,0x7E)</b> command
 *     4.3.13. Storage device
 * -# Sending <b>0x7E,0x04,0xD3,0x01,0xD8,0x7E</b> command
 *    4.3.14. Input Mode
 * -# Sending <b>0x7E,0x04,0xD4,0x01 0xD9,0x7E</b> command
 *    4.3.15. Recording quality 
 * 
 * - Include Playback_Type
 *  Playback_Type: 5 playback files T001.wav/mp3 -T005.wav/mp3 on MicroSDCard 
 *  Playback_Type: 1,2,3,4,5 not random, 1 for P001.wav over duration in seconds.
 *  Playback_Type: 6,7,8,9 random.
 *  random, 6 for P001.wav and P002.wav over duration in seconds.
 *  random, 9 for P001.wav,P002.wav,P003.wav,P004.wav and P005.wav over duration in seconds.

 * -# Sending <b>0x7E,0x07,0xA3,0x50,0x30,0x30,0x31,0x8B,0x7E</b> command
 *    4.3.2 Specify playback of a file by name 
 * -# Sending <b>0x7E,0x07,0xD6,0x52,0x30,0x30,0x31,0xC0,0x7E</b> command
 *    4.3.17 Specify recording of a file by name 
 * -# Sending <b>0x7E,0x03,0xAB,0xAE,0x7E</b> command
 *    4.3.6 Stop playback
 * -# Sending <b>0x7E,0x03,0xD9,0xDC,0x7E</b> command
 *    4.3.20 Stop recording 

 * @internal
 * As long as @ref l_flgAudioActivate is false, audio cannot be enabled.
 * The variable is set true after the configuration file has been read and
 * AudioInit() has been called.
 * @endinternal
 ****************************************************************************//*

Revision History:
2020-07-29,rage	Changed serial driver (avoid requirement for atomic execution).
2020-07-29,rage	Reworked power management and interrupt handling.
2019-11-12,Loes	Initial version.
*/

/*=============================== Header Files ===============================*/
#include <stdlib.h>   // Intializes random number generator 
#include <time.h>     // Intializes random number generator 

#include <stdio.h>
#include <string.h>
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_usart.h"
#include "em_emu.h"
#include "config.h"		// include project configuration parameters
#include "AlarmClock.h"
#include "Audio.h"
#include "LEUART.h"
#include "Logging.h"
#include "Control.h"

/*=============================== Definitions ================================*/

    // Module Debugging
#define MOD_DEBUG	0	// set 1 to enable debugging of this module

   /*!@brief Internal logical states of the AUDIO system. Used from control.c*/
typedef enum
{
    AUDIO_STATE_OFF,		 //!<   0: Audio system is OFF
    AUDIO_STATE_POWER_ON,	 //!<   1: Audio system is powered on
    AUDIO_GET_WORK_STATUS,       //!<   2: Get Audio work status
    AUDIO_GET_SPACE_LEFT,        //!<   3: Get Space Left in the storage device 
    AUDIO_GET_FILE_NUMBERS,      //!<   4: Get Total file numbers
    AUDIO_STATE_SEND_VC,         //!<   5: Send Volume Level Parameter [xx]
    AUDIO_STATE_SEND_ST,         //!<   6: Send Storage Device Parameter [x]  
    AUDIO_STATE_SEND_IM,         //!<   7: Send Input Mode Parameter [x]
    AUDIO_STATE_SEND_RQ,         //!<   8: Send recording quality Parameter [x]
    AUDIO_SEND_PLAYBACK,         //!<   9: Send play specific file [P001-P005]
    AUDIO_SEND_RECORD,	         //!<  10: Send record specific file [R001]
    AUDIO_SEND_PLAYBACK_STOP,    //!<  11: Send Stop playback
    AUDIO_SEND_RECORD_STOP,	 //!<  12: Send Stop recording
    AUDIO_STATE_OPERATIONAL,	 //!<  13: Audio module is operational
    AUDIO_STATE_RECOVER,	 //!<  14: Try to recover after communication error
    END_AUDIO_STATE
} AUDIO_STATE;


    /*!@brief Time in [s] to wait for Audio being ready after power-up. */
#define POWER_UP_DELAY		5

    /*!@brief Maximum Communication Error Count before giving up. */
#define MAX_COM_ERROR_CNT	10


/*=========================== Typedefs and Structs ===========================*/

/*!@brief Local structure to hold UART specific parameters */
typedef struct
{
    USART_TypeDef *	   const UART;		//!< UART device to use
    CMU_Clock_TypeDef	   const cmuClock_UART;	//!< CMU clock for the UART
    IRQn_Type		   const UART_Rx_IRQn;	//!< Rx interrupt number
    GPIO_Port_TypeDef	   const UART_Rx_Port;	//!< Port for RX pin
    uint32_t		   const UART_Rx_Pin;	//!< Rx pin on this port
    IRQn_Type		   const UART_Tx_IRQn;	//!< Tx interrupt number
    GPIO_Port_TypeDef	   const UART_Tx_Port;	//!< Port for TX pin
    uint32_t		   const UART_Tx_Pin;	//!< Tx pin on this port
    uint32_t		   const UART_Route;	//!< Route location
    uint32_t		   const Baudrate;	//!< Baudrate for the Audio
    USART_Databits_TypeDef const DataBits;	//!< Number of data bits
    USART_Parity_TypeDef   const Parity;	//!< Parity mode
    USART_Stopbits_TypeDef const StopBits;	//!< Number of stop bits
} USART_ParmsAudio;


/*========================= Global Data and Routines =========================*/

    /*!@brief Audio power output. */
PWR_OUT   g_AudioPower = PWR_OUT_NONE;

    /*!@brief Volume Control: Parameter [xx]. */
uint32_t  g_AudioCfg_VC;

    /*!@brief Storage Device: Parameter [xx]. */
uint32_t  g_AudioCfg_ST;
  
   /*!@brief Recording input mode: Parameter [xx]. */
uint32_t  g_AudioCfg_IM;

   /*!@brief Recording quality(bit rate): Parameter [xx]. */
uint32_t  g_AudioCfg_RQ;

/*================================ Local Data ================================*/  

    /*!@brief Retrieve information after AUDIO module has been initialized. */
static bool	 l_flgInit;

    /*!@brief Flag that determines if AUDIO module is in use. */
static bool	 l_flgAudioActivate;

 /*! USART parameters for Audio communication */
static const USART_ParmsAudio l_Audio_USART =
{
    USART0, cmuClock_USART0,		//!< select USART0
    USART0_RX_IRQn, gpioPortE, 11,	//!< Rx is PE11
    USART0_TX_IRQn, gpioPortE, 10,	//!< Tx is PE10
    USART_ROUTE_LOCATION_LOC0,		//!< routed thru location #0
    9600, usartDatabits8,		//!< Communication parameters for the
    usartNoParity, usartStopbits1	//!< FN-RM01 MP3 Audio 9600/8/N/1
};

    /*! Flag if AUDIO should be powered on. */
static volatile bool	l_flgAudioOn;

    /*! Flag if AUDIO module is currently powered on. */
static volatile bool	l_flgAudioIsOn;

    /*! Current state of the Audio system. */
volatile AUDIO_STATE l_State;

    /*! Timer handle for "Communication Watchdog". */
static volatile TIM_HDL	l_hdlWdog = NONE;


    /*! Variables for the communication with the AUDIO module. */
static char	l_TxBuffer[30];		//!< Transmit buffer 
static volatile uint8_t l_TxIdx;	//!< Index within the transmit buffer
static volatile bool	l_flgTxComplete;//!< true: Command has been sent
static volatile uint8_t	l_ComErrorCnt;	//!< Communication Error Count
static char	l_RxBuffer[150];	//!< Receive buffer
static volatile uint8_t	l_RxIdx;	//!< Index within the receive buffer

    /*!@brief Counter for l_RxBuffer [0x00] is really received. */
static volatile int l_CheckData = 1;

    /*!@brief l_RxBuffer Command completed set 0 (l_RxIdx). */
static volatile bool l_flgComCompleted = false;

    /*!@brief Converter hex into int */
static volatile int a, b, c, d;

    /*!@brief Current RecordFileNumber */
static volatile int RecordFileNumber;
static volatile int digit_1, digit_2, digit_3;

    /*!@brief Current state of the PlaybackType: 1 to 9 */
static volatile int AudioPlaybackType; // is 1 to 9

    /*!@brief Current state of the PlaybackFileNumber: <= 5 */
static volatile int PlaybackFileNumber; // is <= 5

    /*!@brief Current state of playback run/stop true means RUN, false means STOP. */
static volatile bool l_flgIsPlayAction;

    /*!@brief Current state of isRecordBlocked by playback. */
static volatile bool l_flgIsRecordBlocked;

    /*!@brief Current state of record run/stop true means RUN, false means STOP. */
static volatile bool l_flgIsRecAction;

   /*!@brief Current state of the check audio init is done: true means ON, false means OFF. */
static volatile bool l_flgAudioInitIsDone;	// is false for default  

   /*!@brief Show only once Message "Playback and Record are locked": true means ON, false means OFF. */
static volatile bool l_flgSingleAction;

   /*!@brief Log "Playback and Record are locked" is shown onces: true means ON, false means OFF. */
static volatile bool l_flgLocked;
 
/*=========================== Forward Declarations ===========================*/

       /*! AUDIO Communication Timeout */
static void AudioComTimeout(TIM_HDL hdl);

      /* Power On AUDIO */
static void AudioPowerOn(void);

    /*! Start sending a Command Sequence to Audio module. */
void   AudioSendCmdSeq (AUDIO_STATE state);

    /*! Check AUDIO data */
static void CheckAudioData(void);

    /*! AUDIO USART Setup Routine */
static void AudioUartSetup(void);

    /* Playback Actions with Playback_Type <= 9 */
void   AudioPlayback(void);

    /* Record Actions */
void   AudioRecord(void); 


/***************************************************************************//**
 *
 * @brief	Initialize AUDIO Module
 *
 * This routine initializes the GPIO pins which are connected to the external
 * AUDIO module. The GPIO ports and pins have been defined in the header file.
 *
 ******************************************************************************/
void	AudioInit (void)
{
     /* Check if AUDIO module is already in use */
    if (l_flgAudioActivate)
	AudioPowerOff();	// power-off and reset audio module and UART
       l_flgAudioIsOn = false;
         
        /* Now the AUDIO module isn't active any more */
    l_flgAudioActivate = false;
    
    if (g_AudioPower == PWR_OUT_NONE)
	return;
    
        /* AUDIO module should be activated and initialized */
    l_flgAudioActivate = true;
    l_flgInit = true;

#ifdef LOGGING
    Log ("Initializing Audio for Power Output %s",
	 g_enum_PowerOutput[g_AudioPower]);
#endif
    
#ifdef LOGGING   
    Log ("Audio Volume control VC is %ld", g_AudioCfg_VC);
#endif
    if (g_AudioCfg_VC < 1 | g_AudioCfg_VC > 32)
	LogError("Volume control must be between 1 and 31");
        
#ifdef LOGGING   
    Log ("Audio Storage device ST is %ld",
	 g_AudioCfg_ST);
#endif
    if (g_AudioCfg_ST > 1)
	LogError("Storage device must be between 0 and 1");

#ifdef LOGGING   
    Log ("Audio Audio-recording input mode IM is %ld", g_AudioCfg_IM);
#endif
    if (g_AudioCfg_IM > 2)
	LogError("Audio-recording input mode IM must be between 0 and 2");
    
#ifdef LOGGING   
    Log ("Audio Recording quality (bit rate) mode RQ is %ld", g_AudioCfg_RQ);
#endif
    if (g_AudioCfg_RQ > 3)
	LogError("Recording quality (bit rate) must be between 0 and 3");
     
    /* Create timer for a "Communication Watchdog" */
    if (l_hdlWdog == NONE)
	l_hdlWdog = sTimerCreate (AudioComTimeout);
    
    drvLEUART_sync();	// to prevent UART buffer overflow
}


/***************************************************************************//**
 *
 * @brief	Receive from control.c new Play_Type
 *
 * This routine get the new PlayType from control.c
 *
 ******************************************************************************/
void AudioPlayback()
{

time_t t;  
srand((unsigned) time(&t)); // Init only once ??

   if (AudioPlaybackType <= 5)
   {
      PlaybackFileNumber = AudioPlaybackType;
      /*! Start sending a Command Sequence to AUDIO module. */
      AudioSendCmdSeq(AUDIO_SEND_PLAYBACK);
   }
   else
   {
      if (AudioPlaybackType == 6)
      {
         PlaybackFileNumber = rand() % 2 + 1;// generate number between 1 and 2
      } 
      else if (AudioPlaybackType == 7)
      {
         PlaybackFileNumber = rand() % 3 + 1;// generate number between 1 and 3
      }    
      else if (AudioPlaybackType == 8)
      {
         PlaybackFileNumber = rand() % 4 + 1;// generate number between 1 and 4
      }
      else if (AudioPlaybackType == 9)
      {
         PlaybackFileNumber = rand() % 5 + 1;// generate number between 1 and 5
      }
      /*! Start sending a Command Sequence to AUDIO module. */
      AudioSendCmdSeq(AUDIO_SEND_PLAYBACK);
   }
}


/***************************************************************************//**
 *
 * @brief	Receive from control.c Record
 *
 * This routine send number of record files into AUDIO.c  
 *
 ******************************************************************************/
void    AudioRecord(void)
{
    RecordFileNumber++;    
    digit_1 = (RecordFileNumber % 10);                       // 001
    digit_2 = (((RecordFileNumber - digit_1) % 100) / 10);   // 010
    digit_3 = (((RecordFileNumber - digit_2) % 1000) / 100); // 100
    
    if (digit_3 >= 9 && digit_2 >= 8 && digit_1 >= 0)
    {
      /* [R980.wav] reached */
       LogError("Audio: Supports maximum 999 record files");
    }
    
    digit_1 = digit_1 + 48; // 0 into 48 dez
    digit_2 = digit_2 + 48; // 0 into 48 dez
    digit_3 = digit_3 + 48; // 0 into 48 dez
      
    /*! Start sending a Command Sequence to AUDIO module. */
    AudioSendCmdSeq(AUDIO_SEND_RECORD); 
}


/***************************************************************************//**
 *
 * @brief	Enable AUDIO module
 *
 * This routine enables the audio module, i.e. it notifies the audio module 
 * software to power up and initialize the module and the related hardware.
 * It is usually called by PowerControl().
 *
 * @see AudioDisable(), AudioTimedDisable(), AudioCheck()
 *
 ******************************************************************************/
void AudioEnable(void)
{
    /* initiate power-on of the AUDIO hardware */
    l_flgAudioOn = true;
}

/***************************************************************************//**
 *
 * @brief	Disable AUDIO module
 *
 * This routine immediately disables the AUDIO module.
 *
 * @see AudioEnable(), AudioCheck()
 *
 ******************************************************************************/
void AudioDisable (void)
{
    if (l_flgAudioOn)
    {
	l_flgAudioOn = false;    
       
	/* Audio module should be powered OFF */
	if (l_flgAudioIsOn)
	{
	    AudioPowerOff();
	    l_flgAudioIsOn = false;
	}
    }
}


/***************************************************************************//**
 *
 * @brief	Power Audio module On
 *
 * This routine powers the Audio module on and initializes the related hardware.
 *
 ******************************************************************************/
static void AudioPowerOn (void)
{
    if (l_flgAudioActivate)
    {      
#ifdef LOGGING
	/* Generate Log Message */
	Log ("Audio is powered ON");
#endif
         
    /* (Re-)initialize variables */
    l_flgTxComplete = false;
    l_RxIdx = l_TxIdx = 0;
   
    /* Module Audio requires EM1, set bit in bit mask */
    Bit(g_EM1_ModuleMask, EM1_MOD_AUDIO) = 1;
    
    /* Prepare UART to communicate with AUDIO module */
    AudioUartSetup();
    
    /* Set Power Enable Pin for the scales hardware to ON */
    PowerOutput (g_AudioPower, PWR_ON);
     
    /* Wait some time until Audio is up and running */
    l_State = AUDIO_STATE_POWER_ON;
    /* Log Message see state =  AUDIO_STATE_POWER_ON */
    
    l_flgAudioInitIsDone = false;
    l_flgLocked = false;
   }
}


/***************************************************************************//**
 *
 * @brief	Audio Check
 *
 * This routine is called from main.c
 *
 ******************************************************************************/
void	AudioCheck (void)
{
   bool	isControlPlayRun, isControlPlayStop;
   bool	isControlRecRun, isControlRecStop;
   int  isControlPlaybackType;
  
   /* Get current state of playback run and stop from control.c */
   isControlPlayRun  = IsControlPlayRun();
   isControlPlayStop = IsControlPlayStop();
   
   /* Get current state of record run and stop from control.c*/
   isControlRecRun  = IsControlRecRun();
   isControlRecStop = IsControlRecStop();
   
   /* Get current state of playback type from control.c */
   isControlPlaybackType = IsControlPlaybackType();  
   AudioPlaybackType = isControlPlaybackType;
   
   /* Audio Power */
   if (l_flgAudioOn)
   {
	/* Audio module should be powered ON */
	if (!l_flgAudioIsOn)
	{
	    AudioPowerOn();
	    l_flgAudioIsOn = true;
	}
    }
    else
    {
	/* Audio module should be powered OFF */
	if (l_flgAudioIsOn)
	{
	    AudioPowerOff();
	    l_flgAudioIsOn = false;
            l_flgSingleAction = false;
   	}
    }   

     /* Start Audio Playback */
   if (isControlPlayRun && !isControlPlayStop && !l_flgIsPlayAction)
   {
     if (l_flgAudioInitIsDone)
     {
        l_flgIsPlayAction = true;
        l_flgSingleAction = false;
        l_flgIsRecordBlocked = true;
        AudioPlayback();
     }
     else 
     {  
        if(!l_flgSingleAction)
        {
        /* Generate Log Message */
          Log ("Audio: Playback and Record are locked");
          l_flgSingleAction = true;
          l_flgLocked = false;
        }
      }
   }

   /* Stop Audio Playback */
   if (isControlPlayStop && !isControlPlayRun  && l_flgIsPlayAction)   
   {
      l_flgIsPlayAction = false;
      /*! Start sending a Command Sequence to AUDIO module. */
      AudioSendCmdSeq(AUDIO_SEND_PLAYBACK_STOP);
   }
   
   /* Start Audio Record */
   if (isControlRecRun && !isControlRecStop && !l_flgIsRecAction && !l_flgIsRecordBlocked)
   {
      if (l_flgAudioInitIsDone)
      {
         l_flgIsRecAction = true;
         l_flgSingleAction = false; 
         AudioRecord();
      }
      else
      {
         if(!l_flgSingleAction)
         {
            /* Generate Log Message */
            Log ("Audio: Playback and Record are locked");
            l_flgSingleAction = true;
            l_flgLocked = false;
         }
      }
   } 

   /* Stop Audio Record */
   if (isControlRecStop && !isControlRecRun && l_flgIsRecAction) 
   {
      l_flgIsRecAction = false; 
      /*! Start sending a Command Sequence to AUDIO module. */
      AudioSendCmdSeq(AUDIO_SEND_RECORD_STOP);
   }
}
/***************************************************************************//**
 *
 * @brief	Determine if Audio Module is locked  
 *
 * This routine is used to determine if l_flgSingleAction is currently on.
 * See Control.c
 *
 * @return
 * 	The value <i>true</i> if the Audio module is locked,
 *      <i>false</i> if not.
 *
 ******************************************************************************/
bool	IsAudioLocked ()
{
    return l_flgLocked;
}


/***************************************************************************//**
 *
 * @brief	Power Off Audio module
 *
 * This routine powers the AUDIO module immediately off.
 *
 ******************************************************************************/
void AudioPowerOff (void)
{

    /* Set Power Enable Pin for the Audio to OFF */
    PowerOutput (g_AudioPower, PWR_OFF);
       
    /* Set Power Enable Pin for the Audio to OFF */
    l_State = AUDIO_STATE_OFF;
      
    /* Clear Audio-related error conditions */
    ClearError(ERR_SRC_AUDIO);
  
    /* Disable clock for USART module */
    CMU_ClockEnable(l_Audio_USART.cmuClock_UART, false);

    /* Disable Rx and Tx pins */
    GPIO_PinModeSet(l_Audio_USART.UART_Rx_Port,
		    l_Audio_USART.UART_Rx_Pin, gpioModeDisabled, 0);
    GPIO_PinModeSet(l_Audio_USART.UART_Tx_Port,
		    l_Audio_USART.UART_Tx_Pin, gpioModeDisabled, 0);
    
    /* Module Audio is no longer active, clear bit in bit mask */
    Bit(g_EM1_ModuleMask, EM1_MOD_AUDIO) = 0;

#ifdef LOGGING
    /* Generate Log Message */
    Log ("Audio is powered off");
#endif
 
    /* Reset Audio Playback and Record */
    l_flgLocked = false;
    l_flgSingleAction = true;
    l_flgIsPlayAction = false;
    l_flgAudioInitIsDone = false;
}
  

/***************************************************************************//**
 *
 * @brief	Audio Power Fail Handler
 *
 * This function will be called in case of power-fail to bring the Audio
 * hardware into a quiescent, power-saving state.
 *
 ******************************************************************************/
void	AudioPowerFailHandler (void)
{
 
    /* Switch AUDIO module off */
    l_flgAudioOn = false;

    if (l_flgAudioIsOn)
    {
	AudioPowerOff();
	l_flgAudioIsOn = false;
    }
}


/***************************************************************************//**
 *
 * @brief	Audio Communication Timeout
 *
 * This routine is called from the RTC interrupt handler, after the specified
 * amount of time has elapsed to notify a "Communication Timeout" with the
 * audio module.  This means the firmware did not respond within time.
 * The error is logged, then the recovery of the audio module is initiated.
 *
 ******************************************************************************/
static void AudioComTimeout(TIM_HDL hdl)
{
AUDIO_STATE	startState;

    (void) hdl;		// suppress compiler warning "unused parameter"

    /* Check error count */
    if (l_ComErrorCnt > MAX_COM_ERROR_CNT)
    {
	l_State = AUDIO_STATE_OFF;
	AudioDisable();
	return;
    }

    if (l_State == AUDIO_STATE_RECOVER)
    {
	l_State = AUDIO_STATE_POWER_ON;
	AudioEnable();
	return;
    }

    /* Check for power-up problems */
    if (l_ComErrorCnt == 0  &&  l_State == AUDIO_GET_WORK_STATUS)
    {
#ifdef LOGGING
	LogError ("Audio: Timeout during initialization"
		  " - Audio not connected?");
#endif
      
      l_State = AUDIO_STATE_OFF;
      AudioDisable();
      return;
    }

       /* See if power-up time of audio module is over */
    if (l_State == AUDIO_STATE_POWER_ON)
    {
	if (l_flgInit)
	{
	    l_flgInit = false;		// do this only once
#ifdef LOGGING
	    Log ("Audio should be ready, retrieving hard- and software"
		 " information");
#endif
	     
            startState = AUDIO_GET_WORK_STATUS;
        }
	else
	{
#ifdef LOGGING
	    Log ("Audio should be ready, sending configuration values");
#endif
	    startState = AUDIO_GET_FILE_NUMBERS;

	}
	l_flgTxComplete = true;
	AudioSendCmdSeq(startState);

	return;
    }
    
    /* Otherwise it is a real timeout, i.e. error */
    l_ComErrorCnt++;	// increase error count

#ifdef LOGGING
    LogError ("Audio: %d. Communication Timeout in state %d",
	      l_ComErrorCnt, l_State);
#endif

    /* Otherwise initiate recovery of the audio module */
    if (l_ComErrorCnt < MAX_COM_ERROR_CNT)
    {
	/* Immediately disable and power off the audio system */
	AudioDisable();	// calls AudioPowerOff(), sets AUDIO_STATE_OFF

	/* Try to recover in 60 seconds */
	l_State = AUDIO_STATE_RECOVER;
#ifdef LOGGING
    Log ("Try to recover Audio");
#endif
        /* We need up to 60 sec. to check state AUDIO_GET_SPACE_LEFT */
        if (l_hdlWdog != NONE)
        sTimerStart (l_hdlWdog, 60);
    }
    else
    {
#ifdef LOGGING
	LogError ("Audio: MAX_COM_ERROR_CNT (%d) exceeded", MAX_COM_ERROR_CNT);
#endif
    }
}


/***************************************************************************//**
 *
 * @brief	Start sending a Command Sequence to Audio
 *
 * This routine starts to send the specified command of a complete sequence.
 * The next command is usually selected by CheckAudioData().
 *
 * @param[in] state
 *	Must be of type @ref AUDIO_STATE.  Specifies the command to send.
 *
 ******************************************************************************/
void AudioSendCmdSeq(AUDIO_STATE state)
{
char  buffer[40];
const char *cmd;
int  checksum_int;

    cmd = buffer;
    switch (state)
    {
       case AUDIO_GET_WORK_STATUS:  // 4.4.2 Current work status (send)
            sprintf(buffer, "%c%c%c%c%c", 0x7E, 0x03, 0xC2, 0xC5, 0x7E);
            break;
       
        case AUDIO_GET_SPACE_LEFT:  // 4.4.9 Space left in the storage device
            sprintf(buffer, "%c%c%c%c%c", 0x7E, 0x03, 0xCE, 0xD1, 0x7E);
            break;
            
       case AUDIO_GET_FILE_NUMBERS:  // 4.4.3 Total file numbers on SD card or USB flash (send)
            sprintf(buffer, "%c%c%c%c%c", 0x7E, 0x03, 0xC5, 0xC8, 0x7E);
            break;

       case AUDIO_STATE_SEND_VC:    // 4.3.9. Volume control 1 to 31 (send)
            if (g_AudioCfg_VC != 0)
            {
               /* checksum integer calculation [0x04, 0xAE, 0x1F]*/
               checksum_int = 4 + 174 + g_AudioCfg_VC;
               sprintf(buffer, "%c%c%c%c%c%c", 0x7E, 0x04, 0xAE, g_AudioCfg_VC, checksum_int, 0x7E);
            }
            else
            {
               /* send dummy "Stop playback" without 0x00 go into next state */
               sprintf(buffer, "%c%c%c%c%c", 0x7E, 0x03, 0xAB, 0xAE, 0x7E);
            }
            break;
                 
       case AUDIO_STATE_SEND_ST:    // 4.3.13. Storage device (send)
            if (g_AudioCfg_ST != 0)   
            {
               /* 01: shift to USB flash drive */
               sprintf(buffer, "%c%c%c%c%c%c", 0x7E, 0x04, 0xD2, 0x01, 0xD7, 0x7E);
            }
            else
            {  
               /* send dummy "Stop playback" without 0x00 go into next state */
               sprintf(buffer, "%c%c%c%c%c", 0x7E, 0x03, 0xAB, 0xAE, 0x7E);
            }              
            break;
          
       case AUDIO_STATE_SEND_IM:    // 4.3.14. Input Mode (send)
            if (g_AudioCfg_IM != 0)   
            {
                if(g_AudioCfg_IM == 1)
                {
                   /* 01: connect with LINE-IN */
                  sprintf(buffer, "%c%c%c%c%c%c", 0x7E, 0x04, 0xD3, 0x01, 0xD8, 0x7E);
                }
                if(g_AudioCfg_IM == 2)
                {
                   /* 02: connect with 2-channel Aux-In signal */
                  sprintf(buffer, "%c%c%c%c%c%c", 0x7E, 0x04, 0xD3, 0x02, 0xD9, 0x7E);
                }
            }
            else
            {  
               /* send dummy "Stop playback" without 0x00 go into next state */
               sprintf(buffer, "%c%c%c%c%c", 0x7E, 0x03, 0xAB, 0xAE, 0x7E);
            }
            break;

       case AUDIO_STATE_SEND_RQ:   // 4.3.15. Recording quality (send)
            if (g_AudioCfg_RQ != 0)
            {
               if(g_AudioCfg_RQ == 1)
               {
                  /* 01: 96kbps */
                  sprintf(buffer, "%c%c%c%c%c%c", 0x7E, 0x04, 0xD4, 0x01, 0xD9, 0x7E);
               }
               else
               {
                  if(g_AudioCfg_RQ == 2)
                  {
                     /* 02: 64kbps */
                     sprintf(buffer, "%c%c%c%c%c%c", 0x7E, 0x04, 0xD4, 0x02, 0xDA, 0x7E);
                  }
                  else
                  {  
                     /* 03: 32kps */
                     sprintf(buffer, "%c%c%c%c%c%c", 0x7E, 0x04, 0xD4, 0x03, 0xDB, 0x7E);
                  }
               }
            }
            else
            {  
               /* send dummy "Stop playback" without 0x00 go into next state */
               sprintf(buffer, "%c%c%c%c%c", 0x7E, 0x03, 0xAB, 0xAE, 0x7E);
            }
            break;
         
       case AUDIO_SEND_PLAYBACK: // 4.3.2 Specify playback of a file by name [P001-P005] (send)
            l_flgLocked = true;
            if (PlaybackFileNumber <= 5)
            {
               PlaybackFileNumber = PlaybackFileNumber + 48;// 49 dez. -> 0x31
               /* checksum integer calculation [0x07, 0xA3, 0x50, 0x30, 0x30]*/
               checksum_int = 7 + 163 + 80 + 48 + 48 + PlaybackFileNumber;//18B
               sprintf(buffer, "%c%c%c%c%c%c%c%c%c", 0x7E, 0x07, 0xA3, 0x50, 0x30, 0x30, PlaybackFileNumber, checksum_int, 0x7E);
            }           
            else
            {
               /* send dummy "Stop playback" without 0x00 go into next state */
               sprintf(buffer, "%c%c%c%c%c", 0x7E, 0x03, 0xAB, 0xAE, 0x7E);
            }
            break;
            
       case AUDIO_SEND_RECORD: // 4.3.17 Specify recording of a file by name [R001.wav] (send)
            l_flgLocked = true;
            /* checksum integer calculation [0x07,0xD6,0x52,0x30,0x30,0x31]*/
            checksum_int = 7 + 214 + 82 + digit_3 + digit_2 + digit_1;
            sprintf(buffer, "%c%c%c%c%c%c%c%c%c", 0x7E, 0x07, 0xD6, 0x52, digit_3, digit_2, digit_1, checksum_int, 0x7E);
            // checksum decimal: 7+214+82+48+48+49 = 448 dec. / 0x1C0
            break;
       
       case AUDIO_SEND_PLAYBACK_STOP: // 4.3.6 Stop playback (send)
	    sprintf(buffer, "%c%c%c%c%c", 0x7E, 0x03, 0xAB, 0xAE, 0x7E);
             //cmd = "7E03ABAE7E";
	    break;
 
       case AUDIO_SEND_RECORD_STOP: // 4.3.20 Stop recording (send) 
             sprintf(buffer, "%c%c%c%c%c", 0x7E, 0x03, 0xD9, 0xDC, 0x7E);
             //cmd = "7E03D9DC7E";
             break;
             
    //  case AUDIO_STATE_OPERATIONAL: // 4.4.6 Current status SD or USB flash (send)
    //       /* send dummy "Stop playback" without 0x00 go into next state */
    //        sprintf(buffer, "%c%c%c%c%c", 0x7E, 0x03, 0xAB, 0xAE, 0x7E);
    //      break;
    

     default:
#ifdef LOGGING
	    LogError("Audio AudioSendCmdSeq(): INVALID STATE %d", state);
#endif       
	    cmd =  NULL;
	    state = AUDIO_STATE_OFF;
    }

    l_State = state;		// save current state into local variable

    if (cmd != NULL)
	SendCmd(cmd);
}


/***************************************************************************//**
 *
 * @brief	Send Command
 *
 * Send a command to the Audio module.
 *
 * @param[in] pCmdStr
 *	Address pointer of the string to be written into the transmit buffer.
 *
 ******************************************************************************/
void SendCmd(const char *pCmdStr)
{
  
   /* Check if previous command has been written already */
    if (! l_flgTxComplete)
    {
#ifdef LOGGING
       LogError("Audio SendCmd(%s): Previous command still pending", pCmdStr);
#endif
    }  

   /* Check length */
    unsigned int len = strlen(pCmdStr);
    if (len > sizeof(l_TxBuffer) - 2)
    {
#ifdef LOGGING
	LogError("Audio SendCmd(%s): Command too long (%d bytes)", pCmdStr, len);
#endif
	return;		// ignore this command
    }

    /* Copy command string into transmit buffer */
    strcpy(l_TxBuffer, pCmdStr);

    /* Clear flag, reset index */
    l_flgTxComplete = false;
    l_TxIdx = 0;

    /* Enable Tx interrupt to start sending */
    USART_IntSet(l_Audio_USART.UART, USART_IF_TXBL);
    USART_IntEnable(l_Audio_USART.UART, USART_IEN_TXBL);

    /* Start watchdog */
    /* We need up to 60 sec. to check state AUDIO_GET_SPACE_LEFT */
    if (l_hdlWdog != NONE)
	sTimerStart (l_hdlWdog, 60);
}


/***************************************************************************//**
 *
 * @brief	Check Audio Data
 *
 * This routine is called from Audio RX Handler to check the Work Status 
 * and other information from the audio module.
 *
 ******************************************************************************/
static void CheckAudioData(void)
{
    /* Cancel watchdog timer */
    if (l_hdlWdog != NONE)
	sTimerCancel(l_hdlWdog);

#if MOD_DEBUG	// for debugging only
    Log("Audio Data: '%s' state=%d", l_RxBuffer, l_State);
#endif
    /* Consider state */
    switch (l_State)
    {
        case AUDIO_STATE_POWER_ON: // Prompt after power-up 4.4.6 Current status SD or USB (answer)
          if (strncmp(l_RxBuffer, "Ê", 1) == 0)  
          {      
             /* Connection Status operation code 0xCA is received */
             if (l_CheckData >= 2)  //l_CheckData = 1
             {  
                 if (strncmp(l_RxBuffer, "Ê\0", 2) == 0) // 0x00: both MicroSD card and USB flash connected
	         {
                    Log ("Audio: Both MicroSD card and USB flash drive inserted");
                 }
                 if (strncmp(l_RxBuffer, "Ê", 2) == 0) // 0x01: MicroSD card connected only	         
                 {
                    Log ("Audio: MicroSD card inserted");
                 }
                 if (strncmp(l_RxBuffer, "Ê", 2) == 0) // 0x02: USB flash connected only
                 { 
                    Log ("Audio: USB flash inserted");
                 }
                 if (strncmp(l_RxBuffer, "Ê", 2) == 0) // 0x03: neither MicroSD card or USB flash drive connected
                 {
                    Log ("Audio: MicroSD card or USB flash removed");
                 }
#ifdef LOGGING
                 /* Generate Log Message */
 	         Log ("Waiting %ds for Audio module being ready to accept commands...",
	               POWER_UP_DELAY);
#endif
                 /* After delay call AudioComTimeout */
                 if (l_hdlWdog != NONE)
	         sTimerStart (l_hdlWdog, POWER_UP_DELAY);
 
                 l_CheckData = 0;
                 l_flgComCompleted = true;
             }
             l_CheckData++;
          }
          else
          {
             /* Connection status 0xCA is not received */
             LogError("Audio: Connection MicroSD card or USB flash execution failed");
             SetError(ERR_SRC_AUDIO);	// indicate error via LED
             l_flgComCompleted = true;
          }
          break;
          
       case AUDIO_GET_WORK_STATUS:	 // 4.4.2 Current work status 0xC2 (answer)
          if (strncmp(l_RxBuffer, "Â", 1) == 0)
	  {      
               /* Connection Status operation code 0xCA is received */
              if (l_CheckData >= 2)  // l_CheckData = 1
              {  
                 /* Work Status operation code 0xC2 is received */
                 if (strncmp(l_RxBuffer, "Â", 2) == 0) // 0x01: Playing
                 {
                    Log("Audio: Work Status Playing");
                    AudioSendCmdSeq(l_State+1);
                 } 
                 if (strncmp(l_RxBuffer, "Â", 2) == 0) // 0x02: Stopped
                 {
                    Log("Audio: Work Status Stopped");
                    l_State = AUDIO_STATE_OPERATIONAL;
                 }
                 if (strncmp(l_RxBuffer, "Â", 2) == 0) // 0x03: Paused
                 {
                    Log ("Audio: Work Status Paused");
                    Log ("Audio: Waiting up to 50s for capacity left (µSD 32GB)");  
                    AudioSendCmdSeq(l_State+1);
                 }
                 if (strncmp(l_RxBuffer, "Â", 2) == 0) // 0x04: Recording
                 {
                    Log ("Audio: Work Status Recording");
                    AudioSendCmdSeq(l_State+1);
                 }
                 if (strncmp(l_RxBuffer, "Â", 2) == 0) // 0x05: Fast forward/backward
                 {
                    Log ("Audio: Work Status Fast forward/backward");
                    AudioSendCmdSeq(l_State+1);
                 }
                 l_CheckData = 0;
                 l_flgComCompleted = true;
              }
              l_CheckData++;
          }
          else
          {
              /* Work Status replay 0xC2 is not received */
              LogError("Audio: Work Status execution failed");
              SetError(ERR_SRC_AUDIO);	// indicate error via LED
              l_flgComCompleted = true;
          }
          break;
          
       case AUDIO_GET_SPACE_LEFT:   // 4.4.9 Space left in the storage device (answer)
          if (strncmp(l_RxBuffer, "Î", 1) == 0)     
          {
              /* Connection Status operation code 0xCE is received */
              if (l_CheckData >= 3) //l_CheckData = 1
              { 
                  /* Operation code 0xCE is received */
                  if (l_RxBuffer[2] != 0x00) // no space left on SD Card or USB flash
                  {
                     // Umrechnung hex into dez
                     a = ((l_RxBuffer[1] >> 4) & 0x0F) * 4096;// 16x16x16
                     b =  (l_RxBuffer[1] & 0x0F) * 256;       // 16x16
                     c = ((l_RxBuffer[2] >> 4) & 0x0F) * 16;  // 16
                     d =  (l_RxBuffer[2] & 0x0F) * 1;         // 1
                    
                     Log ("Audio: Capacity left (Mb) %d", a+b+c+d);
                     
                     l_CheckData = 0;
                     l_flgComCompleted = true;
                     AudioSendCmdSeq(l_State+1);
                 }
                 else
                 {
                     LogError("Audio: No Space left");
                     SetError(ERR_SRC_AUDIO);	// indicate error via LED
                     l_CheckData = 0;
                     l_flgComCompleted = true;
                 }
              }
              l_CheckData++;
           }
           else
           {
              /* "Î" command execution failed */
              LogError("Audio: Get Space Volume execution failed");
              SetError(ERR_SRC_AUDIO);	// indicate error via LED
              l_CheckData = 0;
           } 
           break;   
 
        case AUDIO_GET_FILE_NUMBERS:  // 4.4.3 Total file numbers in root directory 0xC5 (answer)
          if (strncmp(l_RxBuffer, "Å", 1) == 0) // tested for 160 files
	  { 
              /* Connection Status operation code 0xC5 is received */
              if (l_CheckData >= 3) //l_CheckAUDIOData = 1
              {     
                  if (l_RxBuffer[2] != 0x00) // file number low digits
                  {
                      // Umrechnung hex into dez
                       a = ((l_RxBuffer[1] >> 4) & 0x0F) * 4096;// 16x16x16
                       b =  (l_RxBuffer[1] & 0x0F) * 256;       // 16x16
                       c = ((l_RxBuffer[2] >> 4) & 0x0F) * 16;  // 16
                       d =  (l_RxBuffer[2] & 0x0F) * 1;         // 1
                       
                       RecordFileNumber = (a+b+c+d)-5;
                       
                       Log ("Audio: Total file numbers %d (Includes 5 playback files)", a+b+c+d);
                       Log ("Audio: Next Record file is [R%03d.wav]", RecordFileNumber + 1);
                       l_CheckData = 0;
                       l_flgComCompleted = true;
                       AudioSendCmdSeq(l_State+1);
                   }
                   else
                   {
                      LogError("Audio: No file numbers");
                      SetError(ERR_SRC_AUDIO);	// indicate error via LED
                      l_CheckData = 0;
                      l_flgComCompleted = true;
                   }
              }
              l_CheckData++;
          }
          else
          {
             LogError("Audio: No file numbers execution failed");
             SetError(ERR_SRC_AUDIO);	// indicate error via LED
             l_CheckData = 0;
          }
          break;
 
       case AUDIO_STATE_SEND_VC:    // 4.3.9. Volume control (answer)
          if (strncmp(l_RxBuffer, "", 1) == 0)     
          { 
              /* 0x01 command execution failed */
              LogError("Audio: Volume execution failed");
              SetError(ERR_SRC_AUDIO);	// indicate error via LED
          }
          else
          {
              if (g_AudioCfg_VC == 0)
              {
                 Log ("ERROR Audio: Volume %i value must be between 1 and 31", g_AudioCfg_VC);
              }
              else
              {
                  Log ("Audio: Volume %i is executed successfully", g_AudioCfg_VC);
              }
          }
          l_CheckData = 1;
          l_flgComCompleted = true;
          AudioSendCmdSeq(l_State+1);
          break;
   
       case AUDIO_STATE_SEND_ST:    // 4.3.13. Storage device (answer)
           if (strncmp(l_RxBuffer, "", 1) == 0)
           {
              /* 0x01 command execution failed */ 
              LogError("Audio: Storage device execution failed");
              SetError(ERR_SRC_AUDIO);	// indicate error via LED
           }
           else
           {
              if (g_AudioCfg_ST == 0)
              {
                 Log ("Audio: MicroSD card is supported");
              }
              else 
              {
                 Log ("Audio: USB flash drive is supported");
              }
           }
           l_CheckData = 1;
           l_flgComCompleted = true;
           AudioSendCmdSeq(l_State+1);
           break; 
          
       case AUDIO_STATE_SEND_IM:    // 4.3.14. Input mode (answer)
          if (strncmp(l_RxBuffer, "", 1) == 0)     
          {
             /* 0x01 command execution failed */  
             LogError("Audio: Input Mode execution failed");
             SetError(ERR_SRC_AUDIO);	// indicate error via LED
          }
          else
          {
             if (g_AudioCfg_IM == 0)
             {  
                Log ("Audio: Input Mode connected with MIC");
             }
             if (g_AudioCfg_IM == 1)
             {  
                Log ("Audio: Input Mode connected with LINE-IN");
             }
             if (g_AudioCfg_IM == 2)
             {  
                Log ("Audio: Input Mode connected with 2-channel AUX");
             }
          }
          l_CheckData = 1;          
          l_flgComCompleted = true;
          AudioSendCmdSeq(l_State+1);
          break;
   
      case AUDIO_STATE_SEND_RQ:   // 4.3.15. Recording quality (answer)
         if (strncmp(l_RxBuffer, "", 1) == 0)     
         {  
            /* 0x01 command execution failed */   
            LogError("Audio: Recording quality execution failed");
            SetError(ERR_SRC_AUDIO);	// indicate error via LED
            l_flgAudioInitIsDone = false;
         }
         else
         { 
            if (g_AudioCfg_RQ == 0)
            {  
               Log ("Audio: Recording quality is 128 Kbps");
            }
            if (g_AudioCfg_RQ == 1)
            {  
               Log ("Audio: Recording quality is 96 Kbps");
            }
            if (g_AudioCfg_RQ == 2)
            {  
               Log ("Audio: Recording quality is 64 Kbps");
            }
            if (g_AudioCfg_RQ == 3)
            {  
               Log ("Audio: Recording quality is 32 Kbps");
            }
            Log ("Audio module is operational now");
	    ClearError(ERR_SRC_AUDIO);	// command sequence completed
         }
         l_CheckData = 1;
         l_flgComCompleted = true;
         l_State = AUDIO_STATE_OPERATIONAL;
         l_flgLocked = false;
         l_flgAudioInitIsDone = true;
         break;
     
      case AUDIO_SEND_PLAYBACK: // 4.3.2 Specify playback of a file by name [P001-P005] (answer)
          if (strncmp(l_RxBuffer, "", 1) == 0) 
          {
               /* 0x01 command execution failed */
               LogError("Audio: Playback ON execution failed - Control Playback Type - Wait for Playback off");
          }
	  else
	  {
              if (PlaybackFileNumber == 1 || PlaybackFileNumber == 49)// 49 dez. -> 0x31
              {   
                 Log ("Audio: Playback ON [P001.x]");
	      }
              if (PlaybackFileNumber == 2 || PlaybackFileNumber == 50)// 50 dez. -> 0x32
              {   
                 Log ("Audio: Playback ON [P002.x]");
	      }
              if (PlaybackFileNumber == 3 || PlaybackFileNumber == 51)// 51 dez. -> 0x33
              {   
                 Log ("Audio: Playback ON [P003.x]");
	      }
              if (PlaybackFileNumber == 4 || PlaybackFileNumber == 52)// 52 dez. -> 0x34
              {   
                 Log ("Audio: Playback ON [P004.x]");
	      }
              if (PlaybackFileNumber == 5 || PlaybackFileNumber == 53)// 53 dez. -> 0x35
              {   
                 Log ("Audio: Playback ON [P005.x]");
	      }
          }
          PlaybackFileNumber = 0;
          l_flgComCompleted = true;
          l_State = AUDIO_STATE_OPERATIONAL;
       	  break;
              
      case  AUDIO_SEND_RECORD: // 4.3.17 Specify recording of a file by name [R001.wav] (answer)
          if (strncmp(l_RxBuffer, "", 1) == 0) 
          {
              /* 0x01 command execution failed */
              LogError("Audio: Storage device is full");
          }          
	  else
	  {
             /* 3-Digit Integer Value */ 
             Log ("Audio: Record ON [R%03d.wav]", RecordFileNumber);
          }
          if (strncmp(l_RxBuffer, "", 1) == 0)
          {
              /* 0x02 command execution failed */
              LogError("Audio: Record ON execution failed");
          }
          digit_1 = 0;
          digit_2 = 0;
          digit_3 = 0;
          l_flgComCompleted = true;
          l_State = AUDIO_STATE_OPERATIONAL;
          break;
            
       case AUDIO_SEND_PLAYBACK_STOP: // 4.3.6 Stop playback (answer)
	   if (strncmp(l_RxBuffer, "", 1) == 0) 
           {
	        /* 0x01 command execution failed */
                LogError("Audio: Playback off execution failed");
	    }    
            else
	    {
               Log("Audio: Playback off");
               l_flgLocked = false;
            }
            l_flgIsRecordBlocked = false;
            l_flgComCompleted = true;
            l_State = AUDIO_STATE_OPERATIONAL;
	    break;
            
	case AUDIO_SEND_RECORD_STOP: // 4.3.20 Stop recording (answer)
           if (strncmp(l_RxBuffer, "", 1) == 0) 
           {
                LogError("Audio: Record off execution failed");
           }
	   else
	   {
              Log("Audio: Record off");
              l_flgLocked = false;
           }
           l_flgComCompleted = true;
           l_State = AUDIO_STATE_OPERATIONAL;
	   break;
           
      case AUDIO_STATE_OPERATIONAL: // 4.4.6 Current status SD or USB (answer)
          if (strncmp(l_RxBuffer, "Ê", 1) == 0)  
	  {      
              /* Connection Status operation code 0xCA is received */
              if (l_CheckData >= 2)  // even or odd (ungerade) CheckData = 0
              {  
                 if (strncmp(l_RxBuffer, "Ê\0", 2) == 0) // 0x00: both MicroSD card and USB flash connected
	         {
                    Log ("Audio: Both MicroSD card and USB flash drive inserted");
                 }
                 if (strncmp(l_RxBuffer, "Ê", 2) == 0) // 0x01: MicroSD card connected only	         
                 {
                    Log ("Audio: MicroSD card inserted");
                    Log ("Remove and Insert SD Card to Refresh System");
                 }
                 if (strncmp(l_RxBuffer, "Ê", 2) == 0) // 0x02: USB flash connected only
                 { 
                    Log ("Audio: USB flash inserted");
                 }
                 if (strncmp(l_RxBuffer, "Ê", 2) == 0) // 0x03: neither MicroSD card or USB flash drive connected
                 {
                    Log ("Audio: MicroSD card or USB flash removed");
                 }
                 drvLEUART_sync();	// to prevent UART buffer overflow
                 l_CheckData = 0;
                 l_flgComCompleted = true;
              }
              l_CheckData++;
           }
           else
           {
              l_flgComCompleted = true;
           }
           break;
   
   
	default:			// unknown state
            LogError("Audio: Received \"%s\" for unhandled state %d",
		     l_RxBuffer, l_State);
	    SetError(ERR_SRC_AUDIO);		// indicate error via LED
            l_flgComCompleted = true;
    }
}


/*============================================================================*/
/*=============================== UART Routines ==============================*/
/*============================================================================*/

/* Setup UART in async mode for RS232 */
static USART_InitAsync_TypeDef uartInit = USART_INITASYNC_DEFAULT;


/**************************************************************************//**
 * @brief Audio UART Setup Routine
 *****************************************************************************/
static void AudioUartSetup(void)
{
    /* Enable clock for USART module */
    CMU_ClockEnable(l_Audio_USART.cmuClock_UART, true);

    /* Configure GPIO Rx and Tx pins - enable pull-up for Rx */
    GPIO_PinModeSet(l_Audio_USART.UART_Rx_Port,
		    l_Audio_USART.UART_Rx_Pin, gpioModeInputPull, 1);
    GPIO_PinModeSet(l_Audio_USART.UART_Tx_Port,
		    l_Audio_USART.UART_Tx_Pin, gpioModePushPull, 1);

    /* Prepare structure for initializing UART in asynchronous mode */
    uartInit.enable       = usartDisable;   // Don't enable UART upon initialization
    uartInit.refFreq      = 0;              // Set to 0 to use reference frequency
    uartInit.baudrate     = l_Audio_USART.Baudrate;
    uartInit.oversampling = usartOVS16;     // Oversampling. Range is 4x, 6x, 8x or 16x
    uartInit.databits     = l_Audio_USART.DataBits;
    uartInit.parity       = l_Audio_USART.Parity;
    uartInit.stopbits     = l_Audio_USART.StopBits;
#if defined( USART_INPUT_RXPRS ) && defined( USART_CTRL_MVDIS )
    uartInit.mvdis        = false;          // Disable majority voting
    uartInit.prsRxEnable  = false;          // Enable USART Rx via Peripheral Reflex System
    uartInit.prsRxCh      = usartPrsRxCh0;  // Select PRS channel if enabled
#endif

    /* Initialize USART with uartInit structure */
    USART_InitAsync(l_Audio_USART.UART, &uartInit);

    /* Prepare UART Rx and Tx interrupts */
    USART_IntClear(l_Audio_USART.UART, _USART_IFC_MASK);
    USART_IntEnable(l_Audio_USART.UART, USART_IEN_RXDATAV);
    NVIC_SetPriority(l_Audio_USART.UART_Rx_IRQn, INT_PRIO_UART);
    NVIC_SetPriority(l_Audio_USART.UART_Tx_IRQn, INT_PRIO_UART);
    NVIC_ClearPendingIRQ(l_Audio_USART.UART_Rx_IRQn);
    NVIC_ClearPendingIRQ(l_Audio_USART.UART_Tx_IRQn);
    NVIC_EnableIRQ(l_Audio_USART.UART_Rx_IRQn);
    NVIC_EnableIRQ(l_Audio_USART.UART_Tx_IRQn);

    /* Enable I/O pins at UART location #2 */
    l_Audio_USART.UART->ROUTE  = USART_ROUTE_RXPEN
				| USART_ROUTE_TXPEN
				| l_Audio_USART.UART_Route;

    /* Enable UART receiver only */
    USART_Enable(l_Audio_USART.UART, usartEnable);
}


/**************************************************************************//**
 * @brief USART0 RX IRQ Handler
 *****************************************************************************/
void USART0_RX_IRQHandler(void)
{
uint8_t rxData;

    /* Check for RX data valid interrupt */
    if (l_Audio_USART.UART->IF & USART_IF_RXDATAV)
    {
	/* Get byte from RX data register */
	rxData = l_Audio_USART.UART->RXDATA;
        
#if MOD_DEBUG	// for debugging only
	Log("DBG: 0x%02X ('%c')", rxData,rxData < ' '? '.':rxData);
#endif
	switch (rxData)
	{
           case 0xFF: // may be sent after power-up - just ignore
           break;
                
           default:	// any other character
	      if (l_RxIdx < sizeof(l_RxBuffer) - 2)
              {	
	          if (l_flgComCompleted)
                  {
                      l_RxIdx = 0;
                      l_RxBuffer[1] = EOS;
                      l_RxBuffer[2] = EOS;
                      l_RxBuffer[3] = EOS;
                      l_flgComCompleted = false;
                  }
                  l_RxBuffer[l_RxIdx++] = rxData;
                  CheckAudioData();
               }  
	       else if (l_RxIdx < sizeof(l_RxBuffer) - 1)
               {
		   l_RxBuffer[l_RxIdx++] = EOS;
		   LogError("Audio: l_RxBuffer full - Data=\"%s\"",
			     l_RxBuffer);
	       }
	       break;               
              
	} // switch (rxData)
    }
}


/**************************************************************************//**
 * @brief USART0 TX IRQ Handler
 *****************************************************************************/
void USART0_TX_IRQHandler(void)
{
uint8_t  txData;

    /* Check TX buffer level status */
    if (l_Audio_USART.UART->IF & USART_IF_TXBL)
    {
	/* Get the next character from the transmit buffer */
	txData = l_TxBuffer[l_TxIdx];

	if (txData != 0)
	{
	    /* Transmit next character */
	    l_Audio_USART.UART->TXDATA = (uint32_t)txData;
	    l_TxIdx++;
	}
	else
	{
	    /* Disable Tx interrupt if no more bytes in buffer */
	    USART_IntDisable(l_Audio_USART.UART, USART_IEN_TXBL);

	    /* Set flag to indicate data has been transmitted completely */
	    l_flgTxComplete = true;
	}
    }
}
