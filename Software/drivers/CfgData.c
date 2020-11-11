/***************************************************************************//**
 * @file
 * @brief	Configuration Data
 * @author	Ralf Gerhauser
 * @version	2018-11-13
 *
 * This module reads and parses a configuration file from the SD-Card, and
 * stores the data into a database.  It also provides routines to get access
 * to these parameters.
 *
 ****************************************************************************//*
Revision History:
2019-06-01,rage	- Bugfix in getString: Corrected pointer increment and check
		  for comment or end of line.
2018-11-13,rage	- The list of Transponder IDs is no more kept in memory, instead
		  the configuration file is read for each compare.  This allows
		  a higher number of IDs to be used (no memory limitations).
		- CfgRead BugFix: Open file in read-only mode.
		- Use of drvLEUART_sync() to prevent FIFO overflows.
		- CfgParse: Consider timezone MEZ or MESZ for alarms.
		- Implemented getInteger() to get and verify integer values.
2017-05-02,rage	- Renamed CAM1_DURATION to CAM_DURATION and Cam1Duration to
		  CamDuration.
		- CfgParse: Added new data type CFG_VAR_TYPE_INTEGER.
2017-01-25,rage	Initial version.
*/

/*=============================== Header Files ===============================*/

#include <stdlib.h>
#include <ctype.h>
#include <string.h>

#include "Logging.h"
#include "LEUART.h"
#include "AlarmClock.h"
#include "CfgData.h"
#include "ff.h"		// FS_FAT12/16/32
#include "diskio.h"	// DSTATUS
#include "microsd.h"
#include "Control.h"

/*=============================== Definitions ================================*/

    /* local debug: show a list of all IDs and settings */
#define CONFIG_DATA_SHOW	1

/*================================ Local Data ================================*/

    /*! Local pointer to list of configuration variables */
static const CFG_VAR_DEF *l_pCfgVarList;

    /*! Local pointer to list of enum definitions */
static const ENUM_DEF    *l_pEnumDef;

    /*! File handle for log file */
static FIL	l_fh;

    /*! Root pointer to ID list */
static ID_PARM *l_pFirstID;
static ID_PARM *l_pLastID;

    /*! Number of IDs in configuration file */
static uint16_t	l_ID_Cnt;

    /*! Flag tells if data has been loaded from file */
static bool	l_flgDataLoaded;

/*=========================== Forward Declarations ===========================*/

static ID_PARM *CfgReadFindID (char *filename, char *transponderID);
static void  CfgDataClear (void);
static ID_PARM *CfgParse (int lineNum, char *line, char *transponderID);
static bool  skipSpace (char **ppStr);
static char *getString (char **ppStr);
static int32_t getInteger (char **ppStr, int lineNum, int varIdx, int32_t minVal);


/***************************************************************************//**
 *
 * @brief	Configuration data initialization
 *
 * This routine must be called once to initialize the configuration data module.
 *
 * @param[in] pCfgVarList
 *	List of configuration variables, the associated data type, and the
 *	address of the respective variable.
 *
 * @param[in] pEnumDef
 *	List of enumerations, each entry contains a NULL-terminated array of
 *	strings with enum names.  The position of a string represents its enum
 *	value, starting with 0 for the first string.  The first element of
 *	<i>pEnumDef</i> is associated with @ref CFG_VAR_TYPE_ENUM_1, the second
 *	entry with @ref CFG_VAR_TYPE_ENUM_2, and so on.
 *
 * @note
 *	Parameter <b>pCfgVarList</b> must point to a persistent data structure,
 *	i.e. this must be valid over the whole life time of the program.
 *
 ******************************************************************************/
void	CfgDataInit (const CFG_VAR_DEF *pCfgVarList, const ENUM_DEF *pEnumDef)
{
    /* Parameter check */
    EFM_ASSERT(pCfgVarList != NULL);

    /* Save configuration */
    l_pCfgVarList = pCfgVarList;
    l_pEnumDef = pEnumDef;
}


/***************************************************************************//**
 *
 * @brief	Read configuration file
 *
 * This routine reads the specified configuration file.
 *
 * @param[in] filename
 *	Name of the configuration file to be read.
 *
 ******************************************************************************/
void	CfgRead (char *filename)
{
    /* read configuration file, store variables */
    CfgReadFindID (filename, NULL);
}


/***************************************************************************//**
 *
 * @brief	Read configuration file and find specified transponder ID
 *
 * This routine is used
 * reads the specified configuration file.
 *
 * @param[in] filename
 *	Name of the configuration file to be read.
 *
 * @param[in] transponderID
 *	Transponder ID to find in the configuration file.
 *
 * @return
 *	Only relevant if @param transponderID is specified for comparison.
 *	Returns a pointer to a @ref ID_PARM structure if ID has been found,
 *	or NULL otherwise.
 *
 ******************************************************************************/
static ID_PARM *CfgReadFindID (char *filename, char *transponderID)
{
ID_PARM	*pID = NULL;	// Pointer to the parameter set of the specified ID
FRESULT	 res;		// FatFs function common result code
int	 lineNum;	// current line number
size_t	 i;		// index within line buffer
UINT	 cnt = 0;	// number of bytes read
char	 line[200];	// line buffer (resides on stack)


  
    /* Flush Log Buffer and keep SD-Card power on */
    LogFlush(true);
    
    /* Log reading of the configuration file */
    if (transponderID == NULL)
    {
	Log ("Reading Configuration File %s", filename);

	/* Discard previous configuration data */
	CfgDataClear();

	/* Assume data can be loaded */
	l_flgDataLoaded = true;
	l_ID_Cnt = 0;
    }

    /* Open the file */
    res = f_open (&l_fh, filename,  FA_READ | FA_OPEN_EXISTING);
    if (res != FR_OK)
    {
	LogError ("CfgRead: FILE OPEN - Error Code %d", res);
        l_flgDataLoaded = false;
	l_fh.fs = NULL;		// invalidate file handle

	/* Power off the SD-Card Interface */
	MICROSD_PowerOff();
	return NULL;
    }
    
    /* Read configuration file line by line */
    for (lineNum = 1;  ;  lineNum++)
    {
	/* Read line char by char because f_gets() does not check read errors */
	for (i = 0;  i < sizeof(line);  i++)
	{
	    res = f_read(&l_fh, line + i, 1, &cnt);
	    if (res != FR_OK)
	    {
		LogError ("CfgRead: FILE READ - Error Code %d at line %d, pos %d%",
			  res, lineNum, i, IsFileHandleValid(&l_fh) ? ""
			  : ", handle not valid");
		l_flgDataLoaded = false;
		break;
	    }
	    if (cnt == 0)
		break;		// end of file detected

	    if (line[i] == '\r')
		i--;		// ignore <CR>
	    else if (line[i] == '\n')
		break;		// read on complete line - process it
	}
	if (res != FR_OK)
	    break;		// abort on error

	/* terminate line buffer with EOS */
	line[i] = EOS;		// substitute <NL> with EOS

	/* Parse line (and compare transponder ID) */
	pID = CfgParse (lineNum, line, transponderID);

	/* Check for end of file or ID found */
	if (cnt == 0  ||  pID != NULL)
	    break;		// end of file detected

	if (i >= sizeof(line))
	{
	    LogError ("CfgRead: Line %d too long (exceeds %d characters)",
		      lineNum, sizeof(line));
	    break;
	}

	drvLEUART_sync();	// to prevent UART buffer overflow
    }

    /* close file after reading data */
    f_close(&l_fh);

    /* Power off the SD-Card Interface */
    MICROSD_PowerOff();


#if CONFIG_DATA_SHOW
    /* show a list of all IDs and settings (will not be logged) */
    if (transponderID == NULL)
	CfgDataShow();
#endif
    return pID;
}


/***************************************************************************//**
 *
 * @brief	Clear current configuration data
 *
 * This routine frees all memory which was allocated by the current
 * configuration data.
 *
 ******************************************************************************/
static void  CfgDataClear (void)
{
ID_PARM	*pID, *pID_next;

    if (l_pFirstID)
    {
	/* free memory of ID list */
	for (pID = l_pFirstID;  pID != NULL;  pID = pID_next)
	{
	    pID_next = pID->pNext;

	    free (pID);
	}

	l_pFirstID = l_pLastID = NULL;
    }
}


/***************************************************************************//**
 *
 * @brief	Parse line for variable assignment or comparison
 *
 * This routine parses the given line buffer for a variable name and its value.
 * Comments and empty lines will be skipped.  When parameter transponderID is
 * specified, parsed data is not stored in the respective variables, instead
 * a comparison is made in case of an ID.
 *
 * @param[in] lineNum
 *	Line number, used to report location in configuration file in case of
 *	an error message is logged.
 *
 * @param[in] line
 *	Line buffer to be parsed.
 *
 * @param[in] transponderID
 *	Transponder ID if used for comparison, NULL otherwise.
 *
 * @return
 *	Only relevant if parameter transponderID is specified for comparison.
 *	Returns a pointer to a @ref ID_PARM structure if ID has been found,
 *	or NULL otherwise.
 *
 ******************************************************************************/
static ID_PARM *CfgParse(int lineNum, char *line, char *transponderID)
{
char	*pStr = line;
char	*pStrBegin;
char	 saveChar;
int	 i;
int	 varIdx;
int	 hour, minute;
int32_t	 duration, value = 0;
static ID_PARM ID_Parm;
ID_PARM	*pNewID;
ALARM_TIME *pAlarm;
CFG_VAR_TYPE cfgVarType;
const char **ppEnumName;


    line--;

    if (skipSpace(&pStr))	// skip white space
	return NULL;

    if (*pStr == '#')		// check for comment line
	return NULL;

    /* expect variable name - must start with alpha character */
    if (! isalpha((int)*pStr))
    {
	LogError ("Config File - Line %d, pos %ld: Invalid Variable Name",
		  lineNum, (pStr-line));
	return NULL;
    }

    /* find end of variable name */
    for (pStrBegin = pStr;  isalnum((int)*pStr) || *pStr == '_';  pStr++)
	;

    saveChar = *pStr;		// save character
    *pStr = EOS;		// terminate variable name for compare

    /* if parameter <transponderID> is specified, find "ID" */
    if (transponderID != NULL)
    {
	if (strcmp(pStrBegin, "ID") != 0)
	    return NULL;	// ignore all variables, except "ID"

	cfgVarType = CFG_VAR_TYPE_ID;
	varIdx = 0;	// pseudo, just to satisfy compiler
    }
    else
    {
	for (varIdx = 0;  l_pCfgVarList[varIdx].name != NULL;  varIdx++)
	    if (strcmp(pStrBegin, l_pCfgVarList[varIdx].name) == 0)
		break;

	if (l_pCfgVarList[varIdx].name == NULL)
	{
	    LogError ("Config File - Line %d, pos %ld: Unknown Variable '%s'",
		      lineNum, (pStrBegin-line), pStrBegin);
	    return NULL;
	}

	cfgVarType = l_pCfgVarList[varIdx].type;
    }

    *pStr = saveChar;		// restore character

    /* equal sign must follow */
    skipSpace(&pStr);		// skip white space

    if (*pStr != '=')
    {
	LogError ("Config File - Line %d, pos %ld: Missing '=' after %s",
		  lineNum, (pStr-line), pStrBegin);
	return NULL;
    }
    pStr++;

    if (skipSpace(&pStr))	// skip white space
    {
	LogError ("Config File - Line %d, pos %ld: Value expected after %s",
		  lineNum, (pStr-line), pStrBegin);
	return NULL;
    }

    /* value depends on the type of variable */
    switch (cfgVarType)
    {
	case CFG_VAR_TYPE_TIME:	// 00:00 to 23:59
	    if (! isdigit((int)*pStr))
	    {
		LogError ("Config File - Line %d, pos %ld, %s: Invalid time",
			  lineNum, (pStr-line), l_pCfgVarList[varIdx].name);
		return NULL;
	    }
	    hour = (int)*(pStr++) - '0';

	    if (isdigit((int)*pStr))
		hour = hour * 10 + (int)*(pStr++) - '0';

	    if (*pStr != ':' || ! isdigit((int)pStr[1]) || ! isdigit((int)pStr[2]))
	    {
		LogError ("Config File - Line %d, pos %ld, %s: Invalid time",
			  lineNum, (pStr-line), l_pCfgVarList[varIdx].name);
		return NULL;
	    }
	    minute = ((int)pStr[1] - '0') * 10 + (int)pStr[2] - '0';
	    pStr += 3;

	    /* all times are given in MEZ - add +1h for MESZ */
	    if (g_isdst)
	    {
		if (++hour > 23)
		    hour = 0;
	    }

	    /* store hours and minutes into variable if one is defined */
	    if (l_pCfgVarList[varIdx].pData != NULL)
	    {
		pAlarm = (ALARM_TIME *)l_pCfgVarList[varIdx].pData;
		pAlarm->Hour   = hour;
		pAlarm->Minute = minute;
	    }

            /* set alarm time and enable it */
	    AlarmSet (FIRST_POWER_ALARM + varIdx, hour, minute);
	    AlarmEnable (FIRST_POWER_ALARM + varIdx);
	    break;


	case CFG_VAR_TYPE_DURATION:	// 0 to n seconds
	    duration = getInteger (&pStr, lineNum, varIdx, 0);
	    *((int32_t *)l_pCfgVarList[varIdx].pData) = duration;
	    break;


	case CFG_VAR_TYPE_ID:	// {ID}:{KEEP_PLAYBACK}:{KEEP_RECORD}:{PLAYBACK_TYPE}
	    /* initialize structure */
	    ID_Parm.pNext = NULL;
	    ID_Parm.KeepPlayback   = DUR_INVALID;
	    ID_Parm.KeepRecord = DUR_INVALID;
	    ID_Parm.PlayType = DUR_INVALID;

	    /* get transponder ID */
	    for (pStrBegin = pStr;  isalnum((int)*pStr);  pStr++);
            
            /* must be followed by ':', space, or EOS */
	    if (*pStr != ':'  &&  ! isspace((int)*pStr)  &&  *pStr != EOS)
	        break;			// generate error message
                   
            
            /* if parameter <transponderID> is specified, compare it */
	    if (transponderID != NULL)
	    {
		saveChar = *pStr;	// save character
		*pStr = EOS;		// terminate transponder ID string

		if (strcmp (pStrBegin, transponderID) != 0)
		    return NULL;	// ID does not match

		*pStr = saveChar;	// restore character

		/* Transponder ID does match, get further parameters... */
	    }

	    /* see if {KEEP_PLAYBACK} value follows */
	    if (*pStr == ':')
	    {
		*(pStr++) = EOS;	// terminate transponder ID string

		if (isdigit((int)*pStr))
		{
		    duration = getInteger (&pStr, lineNum, varIdx, 0);
	            ID_Parm.KeepPlayback = duration;
		}
	    }

	    /* see if {KEEP_RECORD} value follows */
	    if (*pStr == ':')
	    {
		pStr++;

		/* duration in seconds, or empty */
	        if (isdigit((int)*pStr))
		{
		    duration = getInteger (&pStr, lineNum, varIdx, 0);
		    ID_Parm.KeepRecord = duration;
		}
	    }

	    /* see if {PLAYBACK_TYPE} value follows */
	    if (*pStr == ':')
	    {
		pStr++;

		/* field must be a duration in seconds, or empty */
		if (isdigit((int)*pStr))
		{
		    duration = getInteger (&pStr, lineNum, varIdx, 1);
		    if (duration < 0)
			return NULL;	// ERROR

		    ID_Parm.PlayType = duration;
		}
	    }

	    /* if <transponderID> has been found, return parameters */
	    if (transponderID != NULL)
	    {
		return &ID_Parm;
	    }

	    l_ID_Cnt++;		// count ID

	    /* only special IDs "ANY" and "UNKNOWN" are kept in memory */
	    if (strcmp (pStrBegin, "ANY") == 0
	    ||  strcmp (pStrBegin, "UNKNOWN") == 0)
	    {
		/* allocate memory an store ID and parameters */
		pNewID = malloc(sizeof(ID_Parm) + strlen(pStrBegin) + 1);
		if (pNewID == NULL)
		{
		    LogError ("Config File - Line %d, pos %ld, ID: OUT OF MEMORY",
			      lineNum, (pStr-line));
		    return NULL;
		}

		*pNewID = ID_Parm;
		strcpy(pNewID->ID, pStrBegin);

		if (l_pLastID)
		{
		    l_pLastID->pNext = pNewID;
		    l_pLastID = pNewID;
		}
		else
		{
		    l_pLastID = l_pFirstID = pNewID;
		}
	    }

	    break;
           
	case CFG_VAR_TYPE_INTEGER:	// a positive integer variable 0..n
            value = getInteger (&pStr, lineNum, varIdx, value);
	    if (value < 0)
		return NULL;		// ERROR

	    *((int32_t *)l_pCfgVarList[varIdx].pData) = value;
	    break;
            
        case CFG_VAR_TYPE_ENUM_1:	// ENUM types
	case CFG_VAR_TYPE_ENUM_2:
	case CFG_VAR_TYPE_ENUM_3:
	case CFG_VAR_TYPE_ENUM_4:
	case CFG_VAR_TYPE_ENUM_5:
            
            /* get enum string */
	    pStrBegin = getString(&pStr);
	    if (pStrBegin == NULL)
	    {
		LogError ("Config File - Line %d, %s: "
			  "No enum value specified", lineNum,
			  l_pCfgVarList[varIdx].name);
		return NULL;			// error
	    }

	    if (l_pEnumDef == NULL)
	    {
		LogError ("Config File - Line %d, %s: "
			  "No enum names defined", lineNum,
			  l_pCfgVarList[varIdx].name);
		return NULL;
	    }

	    ppEnumName = l_pEnumDef[l_pCfgVarList[varIdx].type
				    - CFG_VAR_TYPE_ENUM_1];

	    for (i = 0;  ppEnumName[i] != NULL;  i++)
		if (strcmp (ppEnumName[i], pStrBegin) == 0)
		    break;
	    if (ppEnumName[i] == NULL)
	    {
		LogError ("Config File - Line %d, %s: "
			  "Enum name %s is not valid", lineNum,
			  l_pCfgVarList[varIdx].name, pStrBegin);
		return NULL;
	    }
	    /* Type cast (PWR_OUT) is used for ALL types of enums */
	    *((PWR_OUT *)l_pCfgVarList[varIdx].pData) = (PWR_OUT)i;
	    break;        


	default:		// unsupported data type
	    LogError ("Config File - Line %d, pos %ld, %s: "
		      "Unsupported data type %d", lineNum, (pStr-line),
		      pStrBegin, cfgVarType);
	    return NULL;
    }

    /* check the rest of the line */
    if (skipSpace(&pStr))	// skip white space
	return NULL;

    if (*pStr == '#')		// check for comment line
	return NULL;

    LogError ("Config File - Line %d, pos %ld: Garbage at end of line",
	      lineNum, (pStr-line));

    return NULL;
}

// returns true if end of string has been reached
static bool skipSpace (char **ppStr)
{
    /* skip white space */
    while (isspace ((int)**ppStr))
	(*ppStr)++;

    /* return true if End Of String has been reached */
    return (**ppStr == EOS ? true : false);
}

// returns true if end of string has been reached
static int32_t getInteger (char **ppStr, int lineNum, int varIdx, int32_t minVal)
{
int32_t value;

    /* convert string to integer value */
    for (value = 0;  isdigit((int)**ppStr);  (*ppStr)++)
	value = value * 10 + (long)**ppStr - '0';

    /* verify allowed minimum value */
    if (value < minVal)
    {
	LogError ("Config File - Line %d, %s=%ld: Value must be >= %ld",
		  lineNum, l_pCfgVarList[varIdx].name, value);
	value = (-1);
    }

    return value;
}


// returns pointer to terminated string, or NULL in case of error
static char *getString (char **ppStr)
{
char	*pStrBegin = *ppStr;

    while (isalnum((int)**ppStr))
	(*ppStr)++;

    /* must be followed by space, comment, or EOS */
    if (**ppStr != EOS)
    {
	**ppStr = EOS;		// terminate string
	(*ppStr)++;		// continue with next characters

	/* skip optional white space at the end of the string */
	while (isspace ((int)**ppStr))
	    (*ppStr)++;

	/* must be followed by comment or EOS */
	if (**ppStr != '#'  &&  **ppStr != EOS)
	    pStrBegin = NULL;	// return NULL for error
    }

    /* return begin of the string or NULL for error */
    return pStrBegin;
}


/***************************************************************************//**
 *
 * @brief	Lookup transponder ID in configuration data
 *
 * This routine searches the specified transponder ID in the @ref ID_PARM list.
 *
 * @param[in] transponderID
 *	Transponder ID to lookup.
 *
 * @return
 * 	Address of @ref ID_PARM structure of the specified ID, or NULL if the
 * 	ID could not be found.
 *
 ******************************************************************************/
ID_PARM *CfgLookupID (char *transponderID)
{
ID_PARM	*pID;

    /* only special IDs "ANY" and "UNKNOWN" are kept in memory */
    if (strcmp (transponderID, "ANY") == 0
    ||  strcmp (transponderID, "UNKNOWN") == 0)
    {
	for (pID = l_pFirstID;  pID != NULL;  pID = pID->pNext)
	    if (strcmp (pID->ID, transponderID) == 0)
		return pID;

	return NULL;	// special ID not found
    }

    /* transponder IDs must be read from the CONFIG file for comparison */
    return CfgReadFindID (CONFIG_FILE_NAME, transponderID);
}


/***************************************************************************//**
 *
 * @brief	Show all configuration data
 *
 * This debug routine shows all configuration data on the serial console.
 *
 ******************************************************************************/
void	 CfgDataShow (void)
{
char	 line[200];
char	*pStr;
int	 i, idx;
int8_t	 hour, minute;
int32_t	 duration, value;
ALARM_TIME *pAlarm;
ID_PARM	*pID;
const char **ppEnumName;


    drvLEUART_sync();	// to prevent UART buffer overflow

    if (l_pCfgVarList == NULL  ||  ! l_flgDataLoaded)
    {
	drvLEUART_puts ("No Configuration Data loaded\n");
	return;
    }
    
    drvLEUART_puts ("All times are displayed for timezone ");
    drvLEUART_puts (g_isdst ? "MESZ\n":"MEZ\n");
    
    /* log all values read from configuration file */
    for (i = 0;  l_pCfgVarList[i].name != NULL;  i++)
    {
	pStr = line;

	if (l_pCfgVarList[i].type == CFG_VAR_TYPE_ID)
	    continue;		// IDs are shown below

	pStr += sprintf (pStr, "%-27s : ", l_pCfgVarList[i].name);

	switch (l_pCfgVarList[i].type)
	{
	    case CFG_VAR_TYPE_TIME:		// 00:00 to 23:59
		if (! AlarmIsEnabled (FIRST_POWER_ALARM + i))
		{
		    pStr += sprintf (pStr, "disabled");
		    break;
		}
		pAlarm = (ALARM_TIME *)l_pCfgVarList[i].pData;
		if (pAlarm != NULL)
		{
		    hour   = pAlarm->Hour;
		    minute = pAlarm->Minute;
		}
		else
		{
		    AlarmGet (FIRST_POWER_ALARM + i, &hour, &minute);
		}
		pStr += sprintf (pStr, "%02d:%02d", hour, minute);
		break;

	    case CFG_VAR_TYPE_DURATION:	// 0 to n seconds, or A for always
		duration = *((int32_t *)l_pCfgVarList[i].pData);
		if (duration == DUR_INVALID)
		    pStr += sprintf (pStr, "invalid");
		else
		    pStr += sprintf (pStr, "%ld", duration);
		break;


	    case CFG_VAR_TYPE_ID: // {ID}:{PLAYBACK_TYPE}:{KEEP_PLAYBACK}:{KEEP_RECORD}
		/* IDs will be handled separately */
		break;


	    case CFG_VAR_TYPE_INTEGER:	// a positive integer variable 0..n
          	value = *((uint32_t *)l_pCfgVarList[i].pData);
		pStr += sprintf (pStr, "%lu", value);
		break;
            
            case CFG_VAR_TYPE_ENUM_1:	// ENUM types
	    case CFG_VAR_TYPE_ENUM_2:
	    case CFG_VAR_TYPE_ENUM_3:
	    case CFG_VAR_TYPE_ENUM_4:
	    case CFG_VAR_TYPE_ENUM_5:
            if (l_pEnumDef == NULL)
	    {
		pStr += sprintf (pStr, "ERROR: No enum names defined");
		    break;
            }
	    /* Type cast (PWR_OUT) is used for ALL types of enums */
	    idx = *((PWR_OUT *)l_pCfgVarList[i].pData);
	    if (idx < 0)
	    {
		pStr += sprintf (pStr, "not set");
               break;
	    }
	    ppEnumName = l_pEnumDef[l_pCfgVarList[i].type
		       - CFG_VAR_TYPE_ENUM_1];
            pStr += sprintf (pStr, "%s", ppEnumName[idx]);
	   break;
        
           default:		// unsupported data type
		LogError ("l_pCfgVarList[%d], %s: Unsupported data type %d",
			  i, l_pCfgVarList[i].name, l_pCfgVarList[i].type);
		break;
	}

	sprintf (pStr, "\n");
	drvLEUART_puts (line);
	drvLEUART_sync();	// to prevent UART buffer overflow
    }

    /* print number of IDs read from the config file */
    sprintf (line, "Number of IDs        : %d\n", l_ID_Cnt);
    drvLEUART_puts (line);

    /* print list of special IDs */
    if (l_pFirstID == NULL)
    {
	drvLEUART_puts ("Warning: Special IDs \"ANY\" and/or \"UNKNOWN\" have"
			" not been defined\n");
    }
    else
    {
	drvLEUART_puts ("                     "
			": KEEP_PLAYBACK : KEEP_RECORD : PLAYBACK_TYPE\n");

	for (pID = l_pFirstID;  pID != NULL;  pID = pID->pNext)
	{
	    pStr = line;

	    pStr += sprintf (pStr, "%-20s", pID->ID);

	    pStr += sprintf (pStr, " :  ");
	    duration = pID->KeepPlayback;
	    if (duration == DUR_INVALID)
		pStr += sprintf (pStr, "default");
	    else
		pStr += sprintf (pStr, "%7ld", duration);

	    pStr += sprintf (pStr, "  :    ");
	    duration = pID->KeepRecord; 
	    if (duration == DUR_INVALID)
		pStr += sprintf (pStr, "default");
	    else
		pStr += sprintf (pStr, "%7ld", duration);

	    pStr += sprintf (pStr, "  :   ");
	    duration = pID->PlayType;
	    if (duration == DUR_INVALID)
		pStr += sprintf (pStr, "default");
	    else
		pStr += sprintf (pStr, "%7ld", duration);

	    sprintf (pStr, "\n");
	    drvLEUART_puts (line);
	    drvLEUART_sync();		// to prevent UART buffer overflow
	}
    }
}
