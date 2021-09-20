//=====================================================================================================================
// This code is from Leuze electronic GmbH + Co. KG. It is free software, WITHOUT ANY GUARANTEES OR WARRANTY
// (without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE).
// Although the author has attempted to find and correct any bugs in this free software, the author is not responsible
// for any damage or losses of any kind caused by the use or misuse of the program. 
// The author is under no obligation to provide support, service, corrections, or upgrades to this software.
//--------------------------------------------------------------------------------------------------------------------
/**
  @file         LxS_ParamSet.h
  @brief        Definition of a Class that represents a LxS-Parameter-Set
  @author       Archipov Slava
  @version      1.0.0
  */
//=====================================================================================================================

#ifndef   __DEVEL_LXS_PARAM_SET_H__
#define   __DEVEL_LXS_PARAM_SET_H__

//---------------------------------------------------------------------------------------------------------------------
// common interface
//
#include <list>
#include <vector>
#include "LxS_Lib/LxS_Serialization.h"
#include "LxS_Lib/LxS_Param.h"

//---------------------------------------------------------------------------------------------------------------------

//! Main Namespace for LxS-Lib
namespace LxS
{
    //-----------------------------------------------------------------------------------------------------------------
    /**
      @class      ParamSet
      @brief      Class that represents a LxS-Parameter-Set
      @details    A parameter set consists of an array of parameters (LxS::Param)
      */
    class ParamSet : public Serializable
    {
        protected:
            std::vector<uint8_t>  prm_struct;     //!< own copy of structure
            ParamList  prm_list;       //!< parameter-set

        public:
            int           id;             //!< own id
            int           stype;          //!< own structure type
            ParamSet();
            bool deserialize(int id, int stype, uint8_t * field, unsigned int f_len, uint8_t * data, unsigned int d_len);
            ~ParamSet();
            const ParamList &       GetSettings() const ;
            bool          CheckStructure(const ParamSet & prm) const ;

            uint32_t serialized_length() const {
                uint32_t sum = 4;
                for (ParamList::const_iterator lit=prm_list.begin();lit!=prm_list.end();lit++) {
                    sum += lit->serialized_length();
                }
                return sum;
            }

            template <class iterator>
                iterator deserialize(iterator it, const iterator & end) {
                    return it;
                }
            
            template <class iterator>
                iterator serialize(iterator it, const iterator & end) const {
                    pushw(it,end,id);
                    pushw(it,end,stype);
                    for (ParamList::const_iterator lit=prm_list.begin();lit!=prm_list.end();lit++) {
                        it = lit->serialize(it,end);
                    }
                    return it;
                }
    };
}


#endif // __DEVEL_LXS_PARAM_SET_H__
