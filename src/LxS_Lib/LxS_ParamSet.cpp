//=====================================================================================================================
// This code is from Leuze electronic GmbH + Co. KG. It is free software, WITHOUT ANY GUARANTEES OR WARRANTY
// (without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE).
// Although the author has attempted to find and correct any bugs in this free software, the author is not responsible
// for any damage or losses of any kind caused by the use or misuse of the program. 
// The author is under no obligation to provide support, service, corrections, or upgrades to this software.
//--------------------------------------------------------------------------------------------------------------------
/**
  @file         LxS_ParamSet.cpp
  @brief        Implementation of a Class that represents a LxS-Parameter-Set
  @author       Archipov Slava
  @version      1.0.0
  */
//=====================================================================================================================

//---------------------------------------------------------------------------------------------------------------------
// common interface
//
#include "LxS_Lib/LxS_Lib.h"

//! Main Namespace for LxS-Lib
namespace LxS
{
    //-------------------------------------------------------------------------------------------------------------------
    /**
      @fn       ParamSet::ParamSet(int id, int stype, uint8_t * field, int f_len, uint8_t * data, int d_len)
      @brief    Constructor for LxS::ParamSet-Class
      @param    id - parameter id
      @param    stype - structure type
      @param    field - pointer to structure
      @param    f_len - length of structure
      @param    data - pointer to data
      @param    d_len - length of data
      @details
      Create new Instance of LxS::ParamSet-Class from given Structure and Data.
      */
    ParamSet::ParamSet() {
        id = -1;
        stype = 0;
    }

    bool ParamSet::deserialize(int id, int stype, uint8_t * field, unsigned int f_len, uint8_t * data, unsigned int d_len)
    {
        prm_struct.resize(f_len);
        std::copy(field,field+f_len,prm_struct.begin());
        id = id;
        stype = stype;
        unsigned int str_len = (f_len >> 1); // 8bit -> 16bit
        unsigned int dt_len = (d_len >> 1); // 8bit -> 16bit
        if ((str_len > 0) && (dt_len > 0)) // if structure and data not empty
        {
            uint16_t * ptr_struct = ((uint16_t *)(field)); // uint8_t* => short*
            uint16_t * ptr_data = ((uint16_t *)(data));    // uint8_t* => short*
            unsigned int p = 0, d = 0;

            while ((str_len > p) && (ptr_struct[p] != 0)) {
                Param prm;
                prm.deserialize(&(ptr_struct[p]), &(ptr_data[d])); // unmanaged : create new parameter
                prm_list.push_back(prm);
                d += ptr_struct[p + 2]; // increment pointers ...
                p += 8;
            };
        };
        return true;
    }

    //-------------------------------------------------------------------------------------------------------------------
    /**
      @fn       ParamSet::~ParamSet()
      @brief    Destructor for LxS::ParamSet-Class
      @details
      Method killing a Instance of LxS::ParamSet-Class.
      */
    ParamSet::~ParamSet()
    {
    }

    //-------------------------------------------------------------------------------------------------------------------
    /**
      @fn       LxS::Param * ParamSet::GetSettings(void)
      @brief    Returns Parameter-Set
      @return   A list with parameters
      @details
      Method returns a Parameter-Set as List of Parameters.
      */
    const ParamList & ParamSet::GetSettings() const
    {
        return prm_list;
    }

    //-------------------------------------------------------------------------------------------------------------------
    /**
      @fn       bool ParamSet::CheckStructure(LxS::ParamSet * prm)
      @brief    Compare own and given Structures
      @return   Both Structurea are same
      @details
      Compare own and given Structures.
      */
    bool ParamSet::CheckStructure(const ParamSet & prm) const
    {
        if (id != prm.id) return false;
        if (stype != prm.stype) return false;
        if (prm_struct != prm.prm_struct) return false;
        if (prm_list != prm.prm_list) return false;
        return true;
    }

}
