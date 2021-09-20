//=====================================================================================================================
// This code is from Leuze electronic GmbH + Co. KG. It is free software, WITHOUT ANY GUARANTEES OR WARRANTY
// (without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE).
// Although the author has attempted to find and correct any bugs in this free software, the author is not responsible
// for any damage or losses of any kind caused by the use or misuse of the program. 
// The author is under no obligation to provide support, service, corrections, or upgrades to this software.
//--------------------------------------------------------------------------------------------------------------------
/**
  @file         LxS_Config.h
  @brief        Definition of a Class that represents general LxS-Configuration
  @author       Archipov Slava
  @version      1.0.0
  */
//=====================================================================================================================

#ifndef   __DEVEL_LXS_CONFIG_H__
#define   __DEVEL_LXS_CONFIG_H__

//---------------------------------------------------------------------------------------------------------------------
// common interface
//
#include "LxS_Lib/LxS_Def.h"
#include "LxS_Lib/LxS_ParamSet.h"

//! Main Namespace for LxS-Lib
namespace LxS
{
    //-----------------------------------------------------------------------------------------------------------------
    /**
      @class      Config
      @brief      Class that represents general LxS-Configuration
      @details    A configuration consists of 4 different parameter sets (LxS::ParamSet). The main interesting parameter
      sets for the user are "general" and "task[]". In the "temp" parameter set there are no user relevant parameters.
      And the "calibration" parameter set is a informal one. It is only readable.
      */
    class Config
    {
        public:
            eType         type;                         //!< Type of the LxS_Device
            ParamSet    general;                      //!< Common parameter set
            ParamSet    calibration;                  //!< Calibration parameter se
            ParamSet    temp;                         //!< Temporary parameter set
            ParamSet    task[(LXS_MAX_TASK_NR + 1)];  //!< Array of all 16 inspection task parameter sets
            Config(void);
            ~Config();
    };
}

#endif // __DEVEL_LXS_CONFIG_H__
