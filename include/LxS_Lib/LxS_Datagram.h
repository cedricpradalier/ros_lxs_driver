/**
  @file         LxS_Datagram.h
  @brief        Definition of a Class that represents a LxS-Datagram
  @version      1.0.0
  */
//=====================================================================================================================

#ifndef   __DEVEL_LXS_DATAGRAM_H__
#define   __DEVEL_LXS_DATAGRAM_H__

//---------------------------------------------------------------------------------------------------------------------
// common interface
//
#include <list>
#include <vector>
#include <stdint.h>
#include <LxS_Lib/LxS_Def.h>
#include <LxS_Lib/LxS_Param.h>
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
    class Datagram : public Serializable
    {
        private:


        public:
            bool        resp;
            eCmd        cmd;       
            eData       dt;
            uint16_t    pkt_num;       
            uint16_t    trans_num;       
            uint16_t    status;       
            uint32_t    encoder;       
            uint16_t    scan_num;       
            uint16_t    type;       
            std::vector<uint16_t>      data;     //!< data-pointer to values

            Datagram() : Serializable() {}
            Datagram(eCmd cmd) : Serializable(), resp(true), cmd(cmd), pkt_num(0), 
                trans_num(cmd), status(0), encoder(0), scan_num(0), type(0x0010) {}
            Datagram(eCmd cmd, uint16_t pkt_num) : Serializable(), resp(false), cmd(cmd), pkt_num(pkt_num), 
                trans_num(0), status(0), encoder(0), scan_num(0), type(0x0010) {}

            ~Datagram() {}


            eMode mode() const {return ((eMode)(status & 0x00F0));}
            bool activation() const {return ((status & 0x0100) == 0x0100);}
            bool ethernet() const {return ((status & 0x0001) == 0x0001);}
            bool warning() const {return ((status & 0x0200) == 0x0200);}
            bool error() const {return ((status & 0x2000) == 0x2000);}


            void dump() const {
                printf("Cmd %d DT %d pkt# %d trans# %d status %04X encoder %d scan# %d type %04X len %d\n",
                        int(cmd),int(dt),int(pkt_num),int(trans_num),int(status),int(encoder),int(scan_num),int(type),int(data.size()));
                for (size_t i=0;i<data.size();i++) {
                    printf("%04X ",int(data[i]));
                }
                printf("\n");
            }

            uint16_t serialized_length() const {
                return 30 + data.size()*2;
            }

            // iterator is an interator on a uint8_t container 
            template <class iterator>
                iterator deserialize(iterator it, const iterator & end) {
                    checkl(it,end,0xFFFFFFFF,true);
                    checkw(it,end,0x0000,true);
                    cmd = (LxS::eCmd)(readw(it,end));
                    checkw(it,end,0x0000,true);
                    pkt_num = readw(it,end);
                    checkw(it,end,0x0000,true);
                    trans_num = readw(it,end);
                    status = readw(it,end);
                    encoder = readl(it,end);
                    checkw(it,end,0x0000,true);
                    scan_num = readw(it,end);
                    type = readw(it,end);
                    uint16_t len = readw(it,end);
                    data.resize(len);
                    for (size_t i=0;i<data.size();i++) {
                        data[i] = readw(it,end);
                    }

                    if (mode() != LXS_MODE_ERROR) {
                        dt = LXS_DATA_UNKNOWN;
                        if (trans_num == 0x0000)  { // measuring-mode
                            dt = LXS_DATA_UNKNOWN;
                            if(cmd == LXS_RESP_DATA_Z) {
                                dt = LXS_DATA_Z;
                            } else if(cmd == LXS_RESP_DATA_X) {
                                dt = LXS_DATA_X;
                            } else if(cmd == LXS_RESP_XZ_HIGH_DATA) {
                                dt = LXS_DATA_ZX;
                            } else if(cmd == LXS_RESP_DATA_FIELD) {
                                dt = LXS_DATA_FIELD;
                            }
                        } else {
                            // response for Trigger-Handling ?
                            if ((trans_num == (uint16_t)LXS_CMD_GET_Z_COORDINATES) && (cmd == LXS_RESP_Z_COORDINATES)) {
                                dt = LXS_DATA_Z;
                            } else if ((trans_num == (uint16_t)LXS_CMD_GET_X_COORDINATES) && (cmd == LXS_RESP_X_COORDINATES)) {
                                dt = LXS_DATA_X;
                            } else if (trans_num == (uint16_t)LXS_CMD_GET_ZX_COORDINATES && (cmd == LXS_RESP_ZX_COORDINATES)) {
                                dt = LXS_DATA_ZX;
                            } else if (trans_num == (uint16_t)LXS_CMD_GET_STATUS_TELEGRAM && (cmd == LXS_RESP_DATA_FIELD)) {
                                dt = LXS_DATA_FIELD;
                            }
                        };
                    }



                    return it;
                }


            template <class iterator>
                iterator serialize(iterator it, const iterator & end) const{
                    pushl(it,end,0xFFFFFFFF);
                    pushw(it,end,0x0000);
                    pushw(it,end,cmd);
                    pushw(it,end,0x0000);
                    pushw(it,end,pkt_num);
                    pushw(it,end,0x0000);
                    pushw(it,end,trans_num);
                    pushw(it,end,status);
                    pushl(it,end,encoder);
                    pushw(it,end,0x0000);
                    pushw(it,end,scan_num);
                    pushw(it,end,type);
                    pushw(it,end,data.size());
                    for (size_t i=0;i<data.size();i++) {
                        pushw(it,end,data[i]);
                    }
                    return it;
                }

    };

    class CmdConnectToSensor : public Datagram {
        public:
            typedef enum {HICONNECT_OFF,HICONNECT_STD,HICONNECT_ON} ConnectMode;
            CmdConnectToSensor(uint16_t pkt_num) : Datagram(LXS_CMD_CONNECT_TO_SENSOR,pkt_num) { 
            }

            CmdConnectToSensor(ConnectMode mode, uint16_t pkt_num) : Datagram(LXS_CMD_CONNECT_TO_SENSOR,pkt_num) { 
                switch (mode) {
                    case HICONNECT_OFF:
                        break;
                    case HICONNECT_ON:
                        data.resize(1);
                        data[0] = 0x0001;
                        break;
                    case HICONNECT_STD:
                        data.resize(1);
                        data[0] = 0x0000;
                        break;
                }
            }
    };

    class CmdDisconnectFromSensor : public Datagram {
        public:
            CmdDisconnectFromSensor(uint16_t pkt_num) : Datagram(LXS_CMD_DISCONNECT_FROM_SENSOR,pkt_num) { }
    };

    class CmdEnterCommandMode : public Datagram {
        public:
            CmdEnterCommandMode(uint16_t pkt_num) : Datagram(LXS_CMD_ENTER_COMMAND_MODE,pkt_num) {
            }
    };

    class CmdExitCommandMode : public Datagram {
        public:
            CmdExitCommandMode(uint16_t pkt_num) : Datagram(LXS_CMD_EXIT_COMMAND_MODE,pkt_num) {
            }
    };

    class CmdEthernetTrigger : public Datagram {
        public:
            CmdEthernetTrigger(uint16_t pkt_num) : Datagram(LXS_CMD_ETHERNET_TRIGGER,pkt_num) {
            }
    };

    class CmdEthernetActivation : public Datagram {
        public:
            CmdEthernetActivation(bool activation, uint16_t pkt_num) : Datagram(LXS_CMD_ETHERNET_ACTIVATION,pkt_num) {
                data.resize(1);
                data[0] = activation?1:0;
            }
    };

    class CmdSetLaserGate : public Datagram {
        public:
            CmdSetLaserGate(uint16_t gate,uint16_t pkt_num) : Datagram(LXS_CMD_SET_LASER_GATE,pkt_num) {
                data.resize(1);
                data[0] = gate;
            }
    };

    class CmdTriggerSingleMeasurement : public Datagram {
        public:
            CmdTriggerSingleMeasurement(uint16_t pkt_num) : Datagram(LXS_CMD_TRIGGER_SINGLE_MEASUREMENT,pkt_num) {
            }
    };

    class CmdGetXCoordinates : public Datagram {
        public:
            CmdGetXCoordinates(uint16_t pkt_num) : Datagram(LXS_CMD_GET_X_COORDINATES,pkt_num) {
            }
    };

    class CmdGetZCoordinates : public Datagram {
        public:
            CmdGetZCoordinates(uint16_t pkt_num) : Datagram(LXS_CMD_GET_Z_COORDINATES,pkt_num) {
            }
    };

    class CmdGetZXCoordinates : public Datagram {
        public:
            CmdGetZXCoordinates(uint16_t pkt_num) : Datagram(LXS_CMD_GET_ZX_COORDINATES,pkt_num) {
            }
    };

    class CmdSetEncoderValue : public Datagram {
        public:
            CmdSetEncoderValue(uint32_t encoder,uint16_t pkt_num) : Datagram(LXS_CMD_SET_ENCODER_VALUE,pkt_num) {
                data.resize(2);
                data[0] = encoder & 0xFFFF;
                data[1] = encoder >> 16;
            }
    };

    class CmdSetScanNumber : public Datagram {
        public:
            CmdSetScanNumber(uint16_t scan,uint16_t pkt_num) : Datagram(LXS_CMD_SET_SCAN_NUMBER,pkt_num) {
                data.resize(1);
                data[0] = scan;
            }
    };

    class CmdSetInspectionTask : public Datagram {
        public:
            CmdSetInspectionTask(uint16_t task, bool save, uint16_t pkt_num) : Datagram(LXS_CMD_SET_INSPECTION_TASK,pkt_num) {
                data.resize(2);
                data[0] = task & 0xF;
                data[1] = save?1:0;
            }
    };

    class CmdGetInspectionTask : public Datagram {
        public:
            CmdGetInspectionTask(uint16_t pkt_num) : Datagram(LXS_CMD_GET_INSPECTION_TASK,pkt_num) {
            }
    };

    class CmdSetUserParameter : public Datagram {
        public:
            CmdSetUserParameter(uint32_t encoder,uint16_t pkt_num) : Datagram(LXS_CMD_SET_USER_PARAMETER,pkt_num) {
                data.resize(3);
                data[0] = encoder & 0xFFFF;
                data[1] = encoder >> 8;
                data[2] = encoder >> 8;
            }
    };

    class CmdGetUserParameter : public Datagram {
        public:
            CmdGetUserParameter(uint16_t param,uint16_t pkt_num) : Datagram(LXS_CMD_GET_USER_PARAMETER,pkt_num) {
                data.resize(1);
                data[0] = param;
            }
    };

    class CmdSetSingleTaskParameter : public Datagram {
        public:
            CmdSetSingleTaskParameter(uint16_t param, uint16_t val, bool save,uint16_t pkt_num) : Datagram(LXS_CMD_SET_SINGLE_INSPECTION_TASK_PARAMETER,pkt_num) {
                data.resize(3);
                data[0] = save?1:0;
                data[1] = param;
                data[2] = val;
            }
    };

    class CmdGetSingleTaskParameter : public Datagram {
        public:
            CmdGetSingleTaskParameter(uint16_t param,uint16_t pkt_num) : Datagram(LXS_CMD_GET_SINGLE_INSPECTION_TASK_PARAMETER,pkt_num) {
                data.resize(1);
                data[0] = param;
            }
    };


    class CmdGetSupportedParameters : public Datagram {
        public:
            CmdGetSupportedParameters(uint16_t dtype, uint16_t pkt_num) : Datagram(LXS_CMD_GET_SUPPORTED_PARAMETERS,pkt_num) {
                data.resize(1);
                data[0] = dtype;
            }
    };

    class CmdGetParameterSet : public Datagram {
        public:
            CmdGetParameterSet(uint16_t id, uint16_t pkt_num) : Datagram(LXS_CMD_GET_PARAMETER_SET,pkt_num) {
                data.resize(1);
                data[0] = id;
            }
    };

    class CmdGetParameterById : public Datagram {
        public:
            CmdGetParameterById(uint16_t id1, uint16_t id2, uint16_t pkt_num) : Datagram(LXS_CMD_GET_PARAMETER_BY_ID,pkt_num) {
                data.resize(2);
                data[0] = id1;
                data[1] = id2;
            }
    };

    class CmdSetParameterSet : public Datagram {
        public:
            CmdSetParameterSet(uint16_t pkt_num) : Datagram(LXS_CMD_SET_PARAMETER_SET,pkt_num) {
                // Not Implemented
            }
    };

    class CmdSetDefaultConfiguration : public Datagram {
        public:
            CmdSetDefaultConfiguration(uint16_t pkt_num) : Datagram(LXS_CMD_SET_DEFAULT_CONFIGURATION,pkt_num) {
                // Not Implemented
            }
    };

    class CmdGetStatusTelegram : public Datagram {
        public:
            CmdGetStatusTelegram(uint16_t pkt_num) : Datagram(LXS_CMD_GET_STATUS_TELEGRAM,pkt_num) {
                // Not Implemented
            }
    };

    class CmdGetErrorText : public Datagram {
        public:
            CmdGetErrorText(uint16_t pkt_num) : Datagram(LXS_CMD_GET_ERROR_TEXT,pkt_num) {
                // Not Implemented
            }
    };

    class CmdGetUserAccessLevel : public Datagram {
        public:
            CmdGetUserAccessLevel(uint16_t pkt_num) : Datagram(LXS_CMD_GET_USER_ACCESS_LEVEL,pkt_num) {
                // Not Implemented
            }
    };

    //////////////////////////////////////////////////////////
    // Response telegrams
    //
    // 

    class RespAckSuccess : public Datagram {
        public:
            RespAckSuccess(const Datagram & dg) : Datagram(dg) {
                assert(dg.cmd == LXS_RESP_ACK_SUCCESS);
            }
    };

    class RespAckFailure : public Datagram {
        public:
            RespAckFailure(const Datagram & dg) : Datagram(dg) {
                assert(dg.cmd == LXS_RESP_ACK_FAILURE);
            }
    };

    class RespUserParameter : public Datagram {
        protected:
            uint16_t value;
        public:
            RespUserParameter(const Datagram & dg) : Datagram(dg) {
                assert(dg.cmd == LXS_RESP_USER_PARAMETER);
                assert(data.size()==1);
                value=data[0];
            }
    };

    class RespSupportedParameters : public Datagram {
        protected:
            // Not parsed
        public:
            RespSupportedParameters(const Datagram & dg) : Datagram(dg) {
                assert(dg.cmd == LXS_CMD_GET_SUPPORTED_PARAMETERS+1);
                assert(data.size()>0);
            }
    };

    class RespParameterSet : public Datagram {
        protected:
            // ParamSet params;
        public:
            RespParameterSet(const Datagram & dg) : Datagram(dg) {
                assert(dg.cmd == LXS_CMD_GET_PARAMETER_SET+1);
                assert(data.size()>0);
            }
    };

    class RespTaskParameter : public Datagram {
        protected:
            Param param;
        public:
            RespTaskParameter(const Datagram & dg) : Datagram(dg) {
                assert(dg.cmd == LXS_RESP_USER_PARAMETER);
                assert(data.size()>=8);
                param.readDatagramData(data);
            }

            const Param & getParam() const {
                return param;
            }
    };


    class RespDataField : public Datagram {
        public:
            RespDataField(const Datagram & dg) : Datagram(dg) {
                assert(dg.cmd == LXS_RESP_DATA_FIELD);
                // Not implemented, not described in the documentation
            }
    };

    class RespXCoordinates : public Datagram {
        protected:
            std::vector<float> x;
        public:
            RespXCoordinates(const Datagram & dg) : Datagram(dg) {
                assert(dg.cmd == LXS_RESP_X_COORDINATES);
                x.resize(data.size());
                for (size_t i=0;i<data.size();i++) {
                    x[i] = data[i]/1000.0;
                }
            }

            const std::vector<float> & getX() const {
                return x;
            }
    };

    class RespDataX : public Datagram {
        protected:
            std::vector<float> x;
        public:
            RespDataX(const Datagram & dg) : Datagram(dg) {
                assert(dg.cmd == LXS_RESP_DATA_X);
                x.resize(data.size());
                for (size_t i=0;i<data.size();i++) {
                    x[i] = data[i]/1000.0;
                }
            }


            const std::vector<float> & getX() const {
                return x;
            }
    };


    class RespZCoordinates : public Datagram {
        protected:
            std::vector<float> z;
        public:
            RespZCoordinates(const Datagram & dg) : Datagram(dg) {
                assert(dg.cmd == LXS_RESP_Z_COORDINATES);
                z.resize(data.size());
                for (size_t i=0;i<data.size();i++) {
                    z[i] = data[i]/1000.0;
                }
            }


            const std::vector<float> & getZ() const {
                return z;
            }
    };

    class RespDataZ : public Datagram {
        protected:
            std::vector<float> z;
        public:
            RespDataZ(const Datagram & dg) : Datagram(dg) {
                assert(dg.cmd == LXS_RESP_DATA_Z);
                z.resize(data.size());
                for (size_t i=0;i<data.size();i++) {
                    z[i] = data[i]/1000.0;
                }
            }


            const std::vector<float> & getZ() const {
                return z;
            }
    };

    class RespZXCoordinates : public Datagram {
        protected:
            std::vector<float> x;
            std::vector<float> z;
        public:
            RespZXCoordinates(const Datagram & dg) : Datagram(dg) {
                assert(dg.cmd == LXS_RESP_Z_COORDINATES);
                x.resize(data.size()/2);
                z.resize(data.size()/2);
                for (size_t i=0;i<data.size()/2;i++) {
                    // Very uncertain...
                    z[i] = data[i*2+0]/1e-5;
                    x[i] = data[i*2+1]/1e-5;
                }
            }


            const std::vector<float> & getZ() const {
                return z;
            }
            const std::vector<float> & getX() const {
                return x;
            }
    };

    class RespXZHighData : public Datagram {
        protected:
            std::vector<float> x;
            std::vector<float> z;
        public:
            RespXZHighData(const Datagram & dg) : Datagram(dg) {
                assert(dg.cmd == LXS_RESP_XZ_HIGH_DATA);
                x.resize(data.size()/2);
                z.resize(data.size()/2);
                for (size_t i=0;i<data.size()/2;i++) {
                    // Very uncertain...
                    z[i] = data[i*2+0]/1e-5;
                    x[i] = data[i*2+1]/1e-5;
                }
            }


            const std::vector<float> & getZ() const {
                return z;
            }
            const std::vector<float> & getX() const {
                return x;
            }
    };

};


#endif // __DEVEL_LXS_DATAGRAM_H__
