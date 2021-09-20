//=====================================================================================================================
// This code is from Leuze electronic GmbH + Co. KG. It is free software, WITHOUT ANY GUARANTEES OR WARRANTY
// (without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE).
// Although the author has attempted to find and correct any bugs in this free software, the author is not responsible
// for any damage or losses of any kind caused by the use or misuse of the program. 
// The author is under no obligation to provide support, service, corrections, or upgrades to this software.
//--------------------------------------------------------------------------------------------------------------------
/**
  @file         LxS_Device.h
  @brief        Definition of a Class that represents a LxS-device
  @author       Archipov Slava
  @version      1.0.0
  */
//=====================================================================================================================

#ifndef   __DEVEL_LXS_DEVICE_H__
#define   __DEVEL_LXS_DEVICE_H__

//---------------------------------------------------------------------------------------------------------------------
// common interface
//
#include "LxS_Lib/LxS_Lib.h"

#include <boost/array.hpp>
#include <boost/asio.hpp>



//! Main Namespace for LxS-Lib
namespace LxS
{
    //-----------------------------------------------------------------------------------------------------------------
    /**
      @class      Device
      @brief      Class that represents a virtual LxS-device
      @details    In this class are all functions to control an LxS sensor over ethernet interface.
      */
    class Device
    {
        protected:
            bool          is_created;                   //!< Socket was created
            bool          is_connected;                 //!< Socket was connected to port and listening
            bool          is_running;                   //!< Thread is running
            bool          is_high;                      //!< marks a High-Resolution-device
            int           thread_count;                 //!< (internal) step counting

        private:
            bool          is_threaded;                  //!< LxS should be threaded
            boost::asio::io_service io_service;
            boost::asio::ip::udp::resolver resolver;
            boost::asio::ip::udp::socket socket;
            boost::asio::ip::udp::endpoint lxs_endpoint;
            boost::asio::ip::udp::endpoint dll_endpoint;

            int           time_out;                     //!< TimeOut for request/response
            int           pkg_num;                      //!< Package counting
            bool          do_send;                      //!< Flag, represent request state
            std::vector<uint8_t>       send_pkg;                     //!< Request package
            int           send_len;                     //!< Length of request package
            uint8_t          in_buffer[LXS_IN_BUFFER_LEN]; //!< Buffer for incoming packages
            bool          has_resp;                     //!< Flag, represent response state
            eCmd          resp_cmd;                     //!< Expected response command
            std::vector<uint8_t>        resp_pkg;                     //!< Response package
            int           resp_len;                     //!< Length of response package
            bool          async_sync;                   //!< Synchronisation for Async-Waiting

            eMode         last_mode;                    //!< Sensor-Mode from last incoming package
            uint16_t        last_state;                   //!< Sensor-State from last incoming package
            uint16_t        last_scan;                    //!< Scan-Number from last incoming package
            uint32_t        last_encoder;                 //!< Encoder-Value from last incoming package
            bool          last_activation;              //!< Activation-Flag from last incoming package
            bool          last_ethernet;                //!< Ethernet-Flag from last incoming package
            bool          last_warning;                 //!< Warning-Flag from last incoming package
            bool          last_error;                   //!< Error-Flag from last incoming package
            clock_t       last_trigger;                 //!< Time of last trigger event
            bool          last_EthActivState;           //!< Activation state set by ethernet activation command

            eCmd     internal_GetResponseCmd(eCmd cmd);
            bool          internal_CreateSocket(void);
            bool          internal_StartThread(void);
            uint8_t *        internal_CreatePackage(eCmd cmd, uint8_t * msg = NULL, int len = 0);
            void          internal_IncomingPackage(int len);
            uint8_t *        internal_Getuint8_ts(uint8_t * bts, int offset, int len);
            const ParamSet &    internal_GetParamSet(int id, int dtype);
            bool          internal_SetParamSet(int id, int dtype, ParamSet * prm, bool permanent = false);
            // static void   LxS_Thread(void * param) { THREAD_BLOCK(param); }

        public:
            Device(int port = LXS_DEFAULT_SERV, bool threaded = true);
            ~Device();
            void          OneThreadStep(void);
            void          SetDataHandler(CallBackFunc func);
            bool          SetTimeOut(int time);
            bool          Create(const char * addr = LXS_DEFAULT_ADDR, int port = LXS_DEFAULT_PORT);
            bool          Destroy(void);
            bool          Connect(bool high = false);
            bool          Disconnect(void);
            bool          ExecuteCommand(eCmd cmd, uint8_t * msg = NULL, int len = 0, bool blocking = true);
            bool          RawExecuteCommand(eCmd cmd, uint8_t * msg = NULL, int len = 0, bool blocking = true, eCmd answerCmd = LXS_RESP_ACK_SUCCESS);
            uint16_t        GetStatus(void);
            bool          SetLaserGate(bool state);
            bool          EthernetTrigger(bool blockingFlag = false, uint8_t cmdTrigRespMask = 7, uint16_t cmdTriggerWait = 49, uint16_t minWaitNonBlockingMode = 20);
            bool          SetSensorMode(eMode mode);
            eMode         GetSensorMode(void);
            bool          SetScanNumber(uint16_t scan);
            uint16_t        GetScanNumber(void);
            bool          SetEncoderValue(uint32_t enc);
            uint32_t        GetEncoderValue(void);
            bool          SetInspectionTask(uint16_t taskNum, bool permanent = false);
            uint16_t        GetInspectionTask(void);
            bool          SetActivation(bool state);
            bool          GetActivation(void);
            eType         GetDeviceType(void);
            const char *  GetDeviceName(eType type);
            Config *      GetDeviceConfiguration(void);
            bool          SetDeviceConfiguration(Config * cfg, bool permanent = true);
            ParamSet *    GetTaskSettings(uint16_t taskNum);
            bool          SetTaskSettings(uint16_t taskNum, ParamSet * prm, bool permanent = false);
            const char *  GetErrorText(uint16_t err_no);
            Param *       GetSingleTaskParam(uint16_t param_id);
            bool          SetSingleTaskParam(uint16_t param_id, Param * prm = NULL, bool permanent = false);
            Param *       GetSingleUserParam(uint16_t param_id);
            bool          SetSingleUserParam(uint16_t param_id, Param * prm = NULL, bool permanent = false);
            const char *  GetVersion(void);
            uint16_t *      ReadResponsePackage(uint16_t * len);
    };
}


#endif // __DEVEL_LXS_DEVICE_H__
