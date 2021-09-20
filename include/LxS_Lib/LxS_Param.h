//=====================================================================================================================
// This code is from Leuze electronic GmbH + Co. KG. It is free software, WITHOUT ANY GUARANTEES OR WARRANTY
// (without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE).
// Although the author has attempted to find and correct any bugs in this free software, the author is not responsible
// for any damage or losses of any kind caused by the use or misuse of the program. 
// The author is under no obligation to provide support, service, corrections, or upgrades to this software.
//--------------------------------------------------------------------------------------------------------------------
/**
  @file         LxS_Param.h
  @brief        Definition of a Class that represents a LxS-Parameter
  @author       Archipov Slava
  @version      1.0.0
  */
//=====================================================================================================================

#ifndef   __DEVEL_LXS_PARAM_H__
#define   __DEVEL_LXS_PARAM_H__

//---------------------------------------------------------------------------------------------------------------------
// common interface
//
#include <list>
#include <vector>
#include <stdint.h>
#include <LxS_Lib/LxS_Serialization.h>

//---------------------------------------------------------------------------------------------------------------------

//! Main Namespace for LxS-Lib
namespace LxS
{
    //-----------------------------------------------------------------------------------------------------------------
    /**
      @class      Param
      @brief      Class that represents a LxS-Parameter
      @details    A parameter has 2 section. First the definition of a parameter (with elements like minimal limit,
      maximal limit and so on). And second the value(s) of the parameter.
      */
    class Param : public Serializable
    {
        private:

        public:
            std::vector<uint16_t>      data;     //!< data-pointer to values
            uint16_t        id;       //!< parameter id
            uint16_t        dt;       //!< parameter datatype
            uint32_t        min;      //!< minimal allowed value
            uint32_t        max;      //!< maximal allowed value
            uint16_t        flds;     //!< count of fields

            Param();
            ~Param();

            bool readDatagramData(const std::vector<uint16_t> & data);

            bool deserialize(uint16_t * structure, uint16_t * data);
            bool  serialize(std::vector<uint8_t> & buffer) const;

            uint32_t serialized_length() const {
                return 16 + data.size()*2;
            }

            template <class iterator>
                iterator deserialize(iterator it, const iterator & end) {
                    id=readw(it,end);
                    dt=readw(it,end);
                    uint16_t cnt=readw(it,end);
                    min=readl(it,end);
                    max=readl(it,end);
                    flds=readw(it,end);
                    data.resize(cnt);
                    for (size_t i=0;i<cnt;i++) {
                        data[i] = readw(it,end);
                    }
                    return it;
                }
            
            template <class iterator>
                iterator serialize(iterator it, const iterator & end) const {
                    pushw(it,end,id);
                    pushw(it,end,dt);
                    pushw(it,end,data.size());
                    pushl(it,end,min);
                    pushl(it,end,max);
                    pushw(it,end,flds);
                    for (size_t i=0;i<data.size();i++) {
                        pushw(it,end,data[i]);
                    }
                    return it;
                }

            bool operator==(const Param & P) const;
    };

    typedef std::list<Param> ParamList;
};


#endif // __DEVEL_LXS_PARAM_H__
