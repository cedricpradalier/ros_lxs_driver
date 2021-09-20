//=====================================================================================================================
// This code is from Leuze electronic GmbH + Co. KG. It is free software, WITHOUT ANY GUARANTEES OR WARRANTY
// (without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE).
// Although the author has attempted to find and correct any bugs in this free software, the author is not responsible
// for any damage or losses of any kind caused by the use or misuse of the program. 
// The author is under no obligation to provide support, service, corrections, or upgrades to this software.
/**
  @file         LxS_Config.cpp
  @brief        Implementation of a Class that represents general LxS-Configuration
  @author       Archipov Slava
  @version      1.0.0
  */
//=====================================================================================================================

//---------------------------------------------------------------------------------------------------------------------
// common interface
//
#include "LxS_Lib/LxS_Lib.h"

//! Main Namespace for LxS-Lib
using namespace LxS;

//-------------------------------------------------------------------------------------------------------------------
/**
  @fn       Config::Config(void)
  @brief    Constructor for LxS::Config-Class
  @details
  Create new Instance of LxS::Config-Class (empty).
  */
Config::Config(void)
{
    type = LXS_TYPE_GENERAL;
}

//-------------------------------------------------------------------------------------------------------------------
/**
  @fn       Config::~Config()
  @brief    Destructor for LxS::Config-Class
  @details
  Method killing a Instance of LxS::Config-Class.
  */
Config::~Config()
{
}
