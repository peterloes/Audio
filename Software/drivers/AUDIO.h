/***************************************************************************//**
 * @file
 * @brief	Header file of module AUDIO.c
 * @author	Peter Loes
 * @version	2019-11-12
 ****************************************************************************//*
Revision History:
2019-11-13,rage	Added global configuration values.
2017-11-08,Loes	Initial version.
*/

#ifndef __INC_AUDIO_h
#define __INC_AUDIO_h

/*=============================== Header Files ===============================*/

#include "config.h"		// include project configuration parameters
#include "Control.h"

/*================================ Global Data ===============================*/

extern PWR_OUT   g_AudioPower;
extern uint32_t  g_AudioCfg_VC;
extern uint32_t  g_AudioCfg_ST;
extern uint32_t  g_AudioCfg_IM;
extern uint32_t  g_AudioCfg_RQ;

/*================================ Prototypes ================================*/

    /* Initialize the AUDIO module */
void	AudioInit (void);

    /* Enable AUDIO module */
void	AudioEnable (void);

    /* Disable AUDIO module*/
void	AudioDisable (void);

    /* Check if to power-on/off AUDIO module */
void   AudioCheck (void);

    /* Power AUDIO module Off */
void	AudioPowerOff (void);

    /* AUDIO Power Fail Handler */
void	AudioPowerFailHandler (void);

/* Send Command to Audio */
void	SendCmd (const char *pCmdStr);

    /* Receive from the control module PLAYBACK with new PLAY_TYPE */
void	ControlUpdatePLAYTYPE (int l_PlayType);

    /* Receive from the control module Record */
void    ControlUpdateRECORD(void); 

    /* Receive from the control module playback_stop */
void	ControlPLAYBACKSTOP (void);

    /* Receive from the control module record_stop */
void	ControlRECORDSTOP (void);


#endif /* __INC_AUDIO_h */
