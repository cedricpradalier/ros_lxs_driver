//=====================================================================================================================
// This code is from Leuze electronic GmbH + Co. KG. It is free software, WITHOUT ANY GUARANTEES OR WARRANTY
// (without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE).
// Although the author has attempted to find and correct any bugs in this free software, the author is not responsible
// for any damage or losses of any kind caused by the use or misuse of the program. 
// The author is under no obligation to provide support, service, corrections, or upgrades to this software.
/**
  @file         LxS_Param.cpp
  @brief        Implementation of a Class that represents a LxS-Parameter
  @author       Archipov Slava
  @version      1.0.0
  */
//=====================================================================================================================

//---------------------------------------------------------------------------------------------------------------------
// common interface
//
#include <assert.h>
#include "LxS_Lib/LxS_Lib.h"

//! Main Namespace for LxS-Lib
using namespace LxS;
//-------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
/**
  @fn       Param::Param(uint16_t * structure, uint16_t * data)
  @brief    Constructor for LxS::Param-Class
  @param    structure - pointer to structure of parameter
  @param    data - pointer to data of parameter
  @details
  Create new Instance of LxS::Param-Class from given Structure and Data.
  */
Param::Param() : Serializable()
{
    id = 0;
    dt = 0;
    min = 0;
    max = 0;
    flds = 0;
}

bool Param::readDatagramData(const std::vector<uint16_t> & input) {
    assert(input.size()>=8);
    id = input[0];
    dt = input[1];
    uint16_t cnt = input[2];
    assert(input.size()>=uint16_t(8+cnt));
    min = ((((uint32_t)(input[3])) << 16) | ((uint32_t)(input[4])));
    max = ((((uint32_t)(input[5])) << 16) | ((uint32_t)(input[6])));
    flds = input[7];
    data.resize(cnt);
    std::copy(input.begin()+8,input.end(),data.begin());
    return true;
}

bool Param::deserialize(uint16_t * structure, uint16_t * datap)
{
    assert(structure);
    id = structure[0];
    dt = structure[1];
    uint16_t cnt = structure[2];
    min = ((((uint32_t)(structure[3])) << 16) | ((uint32_t)(structure[4])));
    max = ((((uint32_t)(structure[5])) << 16) | ((uint32_t)(structure[6])));
    flds = structure[7];
    data.resize(cnt);
    std::copy(datap,datap+cnt,data.begin());
    return true;
}

bool  Param::serialize(std::vector<uint8_t> & buffer) const {
    serialize(buffer.begin(),buffer.end());
    return true;
}


bool Param::operator==(const Param & P) const {
    if (id != P.id) return false;
    if (dt != P.dt) return false;
    if (data.size() != P.data.size()) return false;
    if (min != P.min) return false;
    if (max != P.max) return false;
    if (flds != P.flds) return false;
    if (data != P.data) return false;
    return true;
}

//-------------------------------------------------------------------------------------------------------------------
/**
  @fn       Param::~Param()
  @brief    Destructor for LxS::Param-Class
  @details
  Method killing a Instance of LxS::Param-Class.
  */
Param::~Param()
{
}
