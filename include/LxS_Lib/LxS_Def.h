//=====================================================================================================================
// This code is from Leuze electronic GmbH + Co. KG. It is free software, WITHOUT ANY GUARANTEES OR WARRANTY
// (without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE).
// Although the author has attempted to find and correct any bugs in this free software, the author is not responsible
// for any damage or losses of any kind caused by the use or misuse of the program. 
// The author is under no obligation to provide support, service, corrections, or upgrades to this software.

/**
  @file         LxS_Def.h
  @brief        Types, Constants and Definitions for LxS-Lib
  @author       Archipov Slava
  @version      1.0.1
  */
//=====================================================================================================================

#ifndef   __DEVEL_LXS_DEF_H__
#define   __DEVEL_LXS_DEF_H__

//---------------------------------------------------------------------------------------------------------------------
// common interface
//
#include <stdint.h>

//---------------------------------------------------------------------------------------------------------------------
// global defines
//
#define LXS_DEFAULT_SERV          (5634)                            //!< default PC-port
#define LXS_DEFAULT_PORT          (9008)                            //!< default LxS-port
#define LXS_DEFAULT_ADDR          ((const char *)("192.168.60.3"))  //!< default LxS-address
#define LXS_IN_BUFFER_LEN         (2048)                            //!< incoming buffer length
#define LXS_MIN_TASK_NR           (0)                               //!< minimal task number
#define LXS_MAX_TASK_NR           (15)                              //!< maximal task number
#define LXS_TRIGGER_DELAY         (250)                             //!< delay for trigger (250ms)
#define LXS_TRIGGER_WAIT          (50)                              //!< wait befor request data (old/new)
#define ADDR_IN                   struct sockaddr_in                //!< ethernet address
#define ENDPOINT                  sockaddr *                        //!< ethernet end-point


//! Main Namespace for LxS-Lib
namespace LxS
{
    //---------------------------------------------------------------------------------------------------------------------
    /**
      @enum       eData
      @brief      different possibilities for data-packages
      */
    typedef enum
    {
        LXS_DATA_UNKNOWN  = 0,    //!< unknown/new package
        LXS_DATA_Z        = 1,    //!< package with Z-Data
        LXS_DATA_X        = 2,    //!< package with X-Data
        LXS_DATA_ZX       = 3,    //!< package with ZX-Data
        LXS_DATA_FIELD    = 4,    //!< package with LxS-Informations
    } eData;

    //---------------------------------------------------------------------------------------------------------------------
    /**
      @enum       eCmd
      @brief      possible command-codes and responses within data-pakages
      */
    typedef enum
    {
        LXS_CMD_SET_LASER_GATE                        = 0x0001,   //!< Turns the laser on / off
        LXS_CMD_TRIGGER_SINGLE_MEASUREMENT            = 0x0003,   //!< Tiggers a single measurement
        LXS_CMD_GET_X_COORDINATES                     = 0x0011,   //!< Reads the X coordinates after single trigger
        LXS_RESP_X_COORDINATES                        = 0x0012,   //!< Response telegram of GET_X_COORDINATES. Contains X- profile data.
        LXS_CMD_GET_Z_COORDINATES                     = 0x0013,   //!< Reads the Z coordinates after single trigger.
        LXS_RESP_Z_COORDINATES                        = 0x0014,   //!< Response telegram of GET_Z_COORDINATES. Contains Z- profile data.
        LXS_CMD_SET_ENCODER_VALUE                     = 0x0029,   //!< Set encoder value
        LXS_CMD_GET_SUPPORTED_PARAMETERS              = 0x0043,   //!< Get the structure of the supported parameters
        LXS_CMD_GET_PARAMETER_SET                     = 0x0045,   //!< Get parameter values of a parameter set
        LXS_CMD_SET_PARAMETER_SET                     = 0x0047,   //!< Set values of a parameter set
        LXS_CMD_GET_INSPECTION_TASK                   = 0x0049,   //!< Get number of the actual inspection task.
        LXS_CMD_SET_INSPECTION_TASK                   = 0x004B,   //!< Set actual inspection task
        LXS_CMD_SET_DEFAULT_CONFIGURATION             = 0x004D,   //!< Set Configuration back to default values
        LXS_CMD_GET_PARAMETER_BY_ID                   = 0x0051,   //!< Read parameter value by id number
        LXS_CMD_SET_SCAN_NUMBER                       = 0x0053,   //!< Set scan number
        LXS_CMD_GET_STATUS_TELEGRAM                   = 0x0055,   //!< Get state telegram
        LXS_CMD_SET_USER_PARAMETER                    = 0x0059,   //!< Set value of some special user parameters
        LXS_CMD_GET_USER_PARAMETER                    = 0x005B,   //!< Get value of some special user parameters
        LXS_RESP_USER_PARAMETER                       = 0x005C,   //!< Get value of some special user parameters
        LXS_CMD_GET_ZX_COORDINATES					  = 0x005F,   //!< Reads Z and X-profile data in one Telegramm (only high variant)
        LXS_RESP_ZX_COORDINATES						  = 0x0060,   //!< Reads Z and X-profile data in one Telegramm (only high variant)
        LXS_CMD_GET_ERROR_TEXT                        = 0x0061,   //!< Get error text from error number
        LXS_CMD_GET_USER_ACCESS_LEVEL                 = 0x0063,   //!< Get actual user access level
        LXS_CMD_SET_SINGLE_INSPECTION_TASK_PARAMETER  = 0x006D,   //!< Set a single Parameter in actual Inspection Task
        LXS_CMD_GET_SINGLE_INSPECTION_TASK_PARAMETER  = 0x006F,   //!< Get a single Parameter from actual Inspection Task
        LXS_RESP_TASK_PARAMETER                       = 0x0070,   //!< Get value of some special user parameters
        LXS_CMD_EXECUTE_AREA_SCAN_BASIC_TEACH         = 0x0071,   //!< Executes the basic area scan teach (only LRS)
        LXS_CMD_EXECUTE_TRACK_SCAN_TEACH              = 0x0073,   //!< Executes the track scan teach (only LRS)
        LXS_CMD_EXECUTE_AREA_SCAN_ADVAN_TEACH         = 0x0075,   //!< Executes the advanced area scan teach (only LRS)
        LXS_CMD_ENTER_COMMAND_MODE                    = 0x3132,   //!< Switch into command mode
        LXS_CMD_EXIT_COMMAND_MODE                     = 0x3133,   //!< Switch into measurement mode
        LXS_RESP_ACK_SUCCESS                          = 0x4141,   //!< Generic success status code
        LXS_RESP_ACK_FAILURE                          = 0x414E,   //!< Generic failure status code
        LXS_CMD_CONNECT_TO_SENSOR                     = 0x434E,   //!< Connect to sensor
        LXS_CMD_DISCONNECT_FROM_SENSOR                = 0x4443,   //!< Disconnect from sensor
        LXS_CMD_ETHERNET_ACTIVATION                   = 0x4541,   //!< Activation per ethernet
        LXS_CMD_ETHERNET_TRIGGER                      = 0x4554,   //!< Trigger per ethernet
        LXS_RESP_DATA_FIELD                           = 0x5354,   //!< State telegram
        LXS_RESP_DATA_X                               = 0x5858,   //!< X-coordinate data
        LXS_RESP_XZ_HIGH_DATA                         = 0x5A58,   //!< X and Z data in one Package (High Devices)
        LXS_RESP_DATA_Z                               = 0x5A5A,   //!< Z-coordinate data
    } eCmd;

    //---------------------------------------------------------------------------------------------------------------------
    /**
      @enum       eMode
      @brief      Enumeration of the mode of a LxS_Device
      */
    typedef enum
    {
        LXS_MODE_INVALID  = 0x00,   //!< no communication with the device (Invalid mode)
        LXS_MODE_MEASURE  = 0x10,   //!< Measurement mode
        LXS_MODE_MENU     = 0x20,   //!< Menu mode
        LXS_MODE_COMMAND  = 0x40,   //!< Command mode
        LXS_MODE_ERROR    = 0x80,   //!< Error mode
    } eMode;

    //---------------------------------------------------------------------------------------------------------------------
    /**
      @enum       eType
      @brief      The type is identified by part number of the LxS_Device
      */
    typedef enum
    {
        LXS_TYPE_GENERAL        = 0,			    //!< a common LxS device
        LXS_TYPE_LPS_36_EN      = 50111324,		//!< LPS 36 with encoder
        LXS_TYPE_LPS_36         = 50111325,		//!< LPS 36
        LXS_TYPE_LES_36_VC      = 50111326,		//!< LES 36 with analog output
        LXS_TYPE_LES_36_PB      = 50111327,		//!< LES 36 with profibus
        LXS_TYPE_LES_36_EN_PB   = 50111328,	  //!< LES 36 with encoder and profibus
        LXS_TYPE_LES_36_HI_VC6  = 50111329,	  //!< LES 36 (Hi-variant) with analog output and 6 Outputs
        LXS_TYPE_LRS_36_6       = 50111330,		//!< LRS 36/6
        LXS_TYPE_LES_36_HI_PB   = 50111331,   //!< LES 36 (Hi-variant) with profibus
        LXS_TYPE_LRS_36_PB      = 50111332,		//!< LRS 36 with profibus
        LXS_TYPE_LES_36_VC6     = 50111333,		//!< LES 36 with analog output and 6 IOs
        LXS_TYPE_LPS_36_HI_EN   = 50111334,	  //!< LPS 36 (Hi-variant) with encoder
        LXS_TYPE_LPS_36_HI      = 50111335,		//!< LPS 36 (Hi-variant)
        LXS_TYPE_LRS_36_EN_PB   = 50114007,	  //!< LRS 36 with encoder and profibus
        LXS_TYPE_LPS_36_30      = 50114750,		//!< LPS 36.30 with polfilter
        LXS_TYPE_LRS_36_6_10    = 50115418,		//!< LRS 36/6.10 with plastic window
    } eType;

    //---------------------------------------------------------------------------------------------------------------------
    /**
      @typedef    CallBackFunc
      @brief      CallBack-Funktion for DataHandler
      */
    typedef void (* CallBackFunc)(eData, uint8_t *, int);


} // end namespace LxS

#endif // __DEVEL_LXS_DEF_H__
