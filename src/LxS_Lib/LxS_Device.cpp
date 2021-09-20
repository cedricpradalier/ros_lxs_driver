//=====================================================================================================================
// This code is from Leuze electronic GmbH + Co. KG. It is free software, WITHOUT ANY GUARANTEES OR WARRANTY
// (without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE).
// Although the author has attempted to find and correct any bugs in this free software, the author is not responsible
// for any damage or losses of any kind caused by the use or misuse of the program. 
// The author is under no obligation to provide support, service, corrections, or upgrades to this software.
/**
@file         LxS_Device.cpp
@brief        Inplementation of a Class that represents a virtual LxS-device.
@author       Archipov Slava
@version      1.0.0
@details
In this class are all functions to control an LxS Sensor over ethernet interface.

*/
//=====================================================================================================================

//---------------------------------------------------------------------------------------------------------------------
// common interface
//
#include "LxS_Lib/LxS_Lib.h"

#define BIT0_IS_SET 0x01
#define BIT1_IS_SET 0x02
#define BIT2_IS_SET 0x04

using boost::asio::ip::udp;

//! Main Namespace for LxS-Lib
namespace LxS
{
  //-------------------------------------------------------------------------------------------------------------------
  /**
  @fn       Device::Device(int port, bool threaded)
  @brief    Constructor for LxS::Device-Class
  @param    port - (optional) PC port number for communication with a LxS-device
  @param    threaded - (optional) should be threaded
  @details
  Method creates a instance of LxS::Device-Class with a (free) PC-port and threaded behaviour.
  */
  Device::Device(int port, bool threaded) :
      resolver(io_service), socket(io_service)
  {
    is_threaded   = threaded;   // if internal Thread is used
    is_created    = false;      // no socket created
    is_connected  = false;      // no connection to sensor
    is_running    = false;      // no thread running
    is_high       = false;      // standard resolution
    time_out      = 1000;       // 1 second timeout
    pkg_num       = 0;          // no packages send
    do_send       = false;      // no packages to send
    resp_len      = 0;          // length of empty
    send_len      = 0;          // length of empty
    async_sync    = false;
    last_EthActivState  = false;
    last_ethernet = false;

  }

  //-------------------------------------------------------------------------------------------------------------------
  /**
  @fn       Device::~Device()
  @brief    Destructior for LxS::Device-Class
  @details
  Method killing a instance of LxS::Device-Class.
  */
  Device::~Device()
  {
    Disconnect();   // disconnect from sensor
    Destroy();      // destroy socket
  }

  //-------------------------------------------------------------------------------------------------------------------
  /**
  @fn       void Device::OneThreadStep(void)
  @brief    Single step while Threaded-Handling
  @details
  One single step for handling of connected socket while Threades-Handling is enabled.
  */
  void Device::OneThreadStep(void)
  {
    thread_count++; // just counting steps ...

    if (is_created) // only, if socket was already created ...
    {
      // Step 1: Sending packages

      if (do_send)  // we should send a package ?
      {
        // package was already created, just sending with socket ...
          socket.send_to(boost::asio::buffer(send_pkg), lxs_endpoint);
        // bool sr = (sendto(sock, ((PACKAGE)(send_pkg)), send_len, 0, ((ENDPOINT)(&lxs_end_point)), sz) == SOCKET_ERROR);
        // Sleep(0);

        /*
        int iState = LXS_IN_BUFFER_LEN;
        (void)setsockopt(sock, SOL_SOCKET, SO_SNDBUF, (char *)&iState, 4);
        (void)setsockopt(sock, SOL_SOCKET, SO_RCVBUF, (char *)&iState, 4);

        iState = 1;
        (void)setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, (char *)&iState, 4);
        (void)setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (char *)&iState, 4);

        iState = 0;
        (void)setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, (char *)&iState, 4);
        (void)setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (char *)&iState, 4);
        */

        // do_send = sr;
        // while (do_send)
        // {
        //   do_send = true;
        // }; //! @todo : !!! Error-handling for : Not sended package !!!
      };

      // Step 2: Receiving packages

      // do we have some responses or packages incoming ?
      size_t len = socket.receive_from(boost::asio::buffer(in_buffer), lxs_endpoint);
      // int len = recvfrom(sock, ((PACKAGE)(in_buffer)), LXS_IN_BUFFER_LEN, 0, ((ENDPOINT)(&lxs_end_point)), &sz);
      if (len > 0) // only, if package not empty ...
      {
        internal_IncomingPackage(len);  // parse incoming package
      };
    };
  }

  //-------------------------------------------------------------------------------------------------------------------
  /**
  @fn       void Device::SetDataHandler(LxS::CallBackFunc func)
  @brief    Register a DataHandler
  @param    func - Pointer to CallBack-function for DataHandler
  @details
  Method register new DataHandler-function (or unregister with NULL-value).
  */
  void Device::SetDataHandler(LxS::CallBackFunc func)
  {
    // cb_func = func; // just set new DataHandler ...
  }

  //-------------------------------------------------------------------------------------------------------------------
  /**
  @fn       bool Device::SetTimeOut(int time)
  @brief    Set TimeOut
  @param    time - new positive TimeOut-value in milliseconds
  @return   Operation-State (TimeOut was set)
  @details
  Method set new TimeOut-value for socket Request/Response waiting.
  */
  bool Device::SetTimeOut(int time)
  {
    bool ret = false;
    if ((is_created) && (time > 0)) // only, if socket was already created ...
    {                                     // and given time ist positive ...
      time_out = time; ret = true;  // set new value, done
    };
    return (ret);
  }

  //-------------------------------------------------------------------------------------------------------------------
  /**
  @fn       bool Device::Create(const char * addr, int port)
  @brief    Create Socket
  @param    addr - Ethernet address of LxS-device
  @param    port - Ethernet port of LxS-device
  @return   Operation-state (socket was created)
  @details
  Method trying to create new UDP-socket for communication with LxS-device.
  */
  bool Device::Create(const char * addr, int port)
  {
    if (!is_created)  // only one socket is needed ...
    {
      is_running = false; // stop thread anyway ...

      udp::resolver::query query(udp::v4(), addr, "lxs");
      lxs_endpoint = *resolver.resolve(query);
      socket = udp::socket(io_service, udp::endpoint(udp::v4(), port));

      is_created = internal_CreateSocket();   // create socket ...
      if ((is_created) && (is_threaded))      // if properly created and threaded ...
      {
        is_running = internal_StartThread();  // create also new thread
      };
    };
    return (is_created);
  }

  //-------------------------------------------------------------------------------------------------------------------
  /**
  @fn       bool Device::Destroy(void)
  @brief    Destroy Socket
  @return   Operation-state (socket was destroyed)
  @details
  Method trying to destroy UDP-Socket.
  */
  bool Device::Destroy(void)
  {
    if (is_created)       // only, if socket was already created ...
    {
      (void)closesocket(socket);
      is_created = false;
    };
    is_running = false;   // stop thread anyway ...
    return (!is_created);
  }

  //-------------------------------------------------------------------------------------------------------------------
  /**
  @fn       bool Device::Connect(void)
  @brief    Establish a IP-connection to a LxS-device
  @param    high - Connect as HI-device (FALSE = normal; TRUE = HI-device)
  @return   Operation-state (DLL is connected to device)
  */
  bool Device::Connect(bool high)
  {
    if ((is_created) && (!is_connected))  // only, if not connected ...
    {
      bool ret = false;
      is_high = high;
      is_connected = true; //! @remark : the trick !
      if (is_high)
      {
        uint8_t * msg = new uint8_t[2]; // unmanaged : create buffer
        msg[0] = 0x01; msg[1] = 0x00; // set to 0x0001 ...
        ret = ExecuteCommand(LxS::LXS_CMD_CONNECT_TO_SENSOR, msg, 2); // execute "High-Connect"
        delete[] msg; // unmanaged : free buffer
      }
      else
      {
        ret = ExecuteCommand(LxS::LXS_CMD_CONNECT_TO_SENSOR); // execute "Connect"
      };
      is_connected = (ret | last_ethernet); // if NACK => already connected ?
    };
    return ((is_created) && (is_connected));
  }

  //-------------------------------------------------------------------------------------------------------------------
  /**
  @fn       bool Device::Disconnect(void)
  @brief    Disconnect from LxS-device
  @return   Operation-state (DLL is disconnected)
  @details
  Disconnect IP-connection to a LxS-device.
  */
  bool Device::Disconnect(void)
  {
    bool isDisconnected = false;
    if (is_created)  // only, if socket was created ...
    {
      isDisconnected = ExecuteCommand(LxS::LXS_CMD_DISCONNECT_FROM_SENSOR, NULL, 0, true); // blocking
      if(isDisconnected) // true, if disconnection was sucessful
      {
        is_connected = false; // than connection state is false
      }
    };
    return (isDisconnected);
  }

  //-------------------------------------------------------------------------------------------------------------------
  /**
  @fn       bool Device::ExecuteCommand(LxS::eCmd cmd, uint8_t * msg, int len, bool blocking)
  @brief    Send a ethernet command telegram
  @param    cmd - the command
  @param    msg - pointer to package
  @param    len - length of package
  @param    blocking - should wait for response
  @return   Operation-state (package was sended and response was received)
  @details
  Method to send a ethernet command telegram.
  */
  bool Device::ExecuteCommand(LxS::eCmd cmd, uint8_t * msg, int len, bool blocking)
  {
    bool ret = false;
    if ((is_created) && (is_connected)) // only, if connected ...
    {
      bool expectedPacketReceived = false;
      delete[] send_pkg;        // unmanaged : free older ressources
      LxS::eCmd respCmd = internal_GetResponseCmd(cmd);
      do_send = false;
      send_pkg = internal_CreatePackage(cmd, msg, len); // create package with given command
      send_len = (30 + len);    // calculate package length
      resp_cmd = cmd;           // save command for compare with future response
      has_resp = !blocking;     // should we wait for response ?
      async_sync = !blocking;   // async->synchronisation
      clock_t send_time = clock();    // starting time in ms or timeout ...
      do_send = true;           // set "Start-Sending-Flag" for thread
      while ((do_send) && ((clock() - send_time) < time_out))
      { /*Sleep(0);*/ };              // waiting, until done or timeout
      if (!do_send)             // sending was OK, now waiting for response if blocking ...
      {
        clock_t resp_time = clock();  // starting time in ms or timeout ...
        while(!expectedPacketReceived && ((clock() - resp_time) < time_out) && blocking)
        {
          while ((!has_resp) && ((clock() - resp_time) < time_out))
          { /*Sleep(0);*/ };                // wait, until done or timeout
          // return := if device replied with ACK for given command
          expectedPacketReceived = (resp_cmd == respCmd) || (resp_cmd == LxS::LXS_RESP_ACK_FAILURE);
        }
        ret = ((has_resp) ? (resp_cmd == respCmd) : (false));
      };
    };
    return (ret);
  }

  //-------------------------------------------------------------------------------------------------------------------
  /**
  @fn       bool Device::RawExecuteCommand(LxS::eCmd cmd, uint8_t * msg, int len, bool blocking, eCmd answerCmd)
  @brief    Send a raw ethernet command telegram
  @param    cmd - the command
  @param    msg - pointer to package
  @param    len - length of package
  @param    blocking - should wait for response
  @param    answerCmd - command number of expected answer telegram
  @return   Operation-state (package was sended and response was received)
  @details
  Method to send a raw ethernet command telegram. Normally you can get the answer command you have to expect
  with the function LxS::Device::internal_GetResponseCmd. But if the command you want to use is not yet defined in the library.
  You can construct with RawExecuteCommand the missing command. 
  */
  bool Device::RawExecuteCommand(LxS::eCmd cmd, uint8_t * msg, int len, bool blocking, eCmd answerCmd)
  {
    bool ret = false;
    if ((is_created) && (is_connected)) // only, if connected ...
    {
      bool expectedPacketReceived = false;
      delete[] send_pkg;        // unmanaged : free older ressources
      LxS::eCmd respCmd = answerCmd;
      do_send = false;
      send_pkg = internal_CreatePackage(cmd, msg, len); // create package with given command
      send_len = (30 + len);    // calculate package length
      resp_cmd = cmd;           // save command for compare with future response
      has_resp = !blocking;     // should we wait for response ?
      async_sync = !blocking;   // async->synchronisation
      clock_t send_time = clock();    // starting time in ms or timeout ...
      do_send = true;           // set "Start-Sending-Flag" for thread
      while ((do_send) && ((clock() - send_time) < time_out))
      { /*Sleep(0);*/ };              // waiting, until done or timeout
      if (!do_send)             // sending was OK, now waiting for response if blocking ...
      {
        clock_t resp_time = clock();  // starting time in ms or timeout ...
        while(!expectedPacketReceived && ((clock() - resp_time) < time_out) && blocking)
        {
          while ((!has_resp) && ((clock() - resp_time) < time_out))
          { /*Sleep(0);*/ };                // wait, until done or timeout
          // return := if device replied with ACK for given command
          expectedPacketReceived = (resp_cmd == respCmd) || (resp_cmd == LxS::LXS_RESP_ACK_FAILURE);
        }
        ret = ((has_resp) ? (resp_cmd == respCmd) : (false));
      };
    };
    return (ret);
  }
  
  //-------------------------------------------------------------------------------------------------------------------
  /**
  @fn       uint16_t Device::GetStatus(void)
  @brief    Get state of LxS-device
  @return   Actual state
  @details
  Get actual State of LxS-device from last package.
  */
  uint16_t Device::GetStatus(void)
  {
    return (last_state); // just return ...
  }

  //-------------------------------------------------------------------------------------------------------------------
  /**
  @fn       bool Device::SetLaserGate(bool state)
  @brief    Turn laser on/off
  @param    state - state to be set (FALSE = off; TRUE = on)
  @return   Operation-state (command was sucessful executed)
  @details
  This function operates only in sensor "Command Mode". And if the sensor is additional in "Input Trigger Mode", 
  the laser only flashes on trigger pulse and on laser state "on". When the sensor state changes from "Command Mode"
  to "Measurement Mode", the laser state is switched to state "on".
  */
  bool Device::SetLaserGate(bool state)
  {
    bool ret = false;
    if ((is_created) && (is_connected)) // only, if connected ...
    {
      uint8_t * msg = new uint8_t[2]; // unmanaged : create buffer
      msg[0] = ((state) ? (0x01) : (0x00)); msg[1] = 0x00; // 0x0001 - on / 0x0000 - off
      ret = ExecuteCommand(LxS::LXS_CMD_SET_LASER_GATE, msg, 2);  // execute command ...
      delete[] msg; // unmanaged : free buffer
    };
    return (ret);
  }

  //-------------------------------------------------------------------------------------------------------------------
  /**
  @fn       bool Device::EthernetTrigger(void)
  @brief    Trigger sensor per ethernet telegram
  @param    blockingFlag [only in measure mode] - wait for profile telegram (FALSE = continue without wait; TRUE = wait is on)
  @param    cmdTrigRespMask [only in command mode] - to mask which process telegram should be requested (Bit0 = Z-Data; Bit1 = X-Data; Bit2 = State-Data)
  @param    cmdTriggerWait [only in command mode] - time to wait [in ms] after ACK is received.
  @param    minWaitNonBlockingMode [only in nonblocking measure mode] - time to wait [in ms] after "Ethernet Trigger"-command
  @return   Operation-State (Command was sucessful executed)
  @details
  The behaviour in "Measure Mode" is different to that in "Command Mode".
  If the sensor is in "measure mode", then the command "LXS_CMD_ETHERNET_TRIGGER" is send and the sensor answers
  with Z- and X-Data (LPS) or with State- Data (LRS and LES). Pay attention that the trigger command [in measure mode]
  only works if the operation mode is set to "Input Triggered".@n
  If the sensor is in "Command Mode", then first the command "LXS_CMD_TRIGGER_SINGLE_MEASUREMENT" is send,
  after that every single data-telegram (which depend on cmdTrigRespMask) is requested.
  Because of a bug in firmware versions less then V01.43, you have to set "cmdTriggerWait" parameter minimal to 50 ms.
  In versions greater or equal V01.43, you can set "cmdTriggerWait" parameter value to 0 ms.
  In "Measure Mode" you can trigger with a minimal time gap of 18 ms an in "Command Mode" with 50 ms.
  */
  bool Device::EthernetTrigger(bool blockingFlag, uint8_t cmdTrigRespMask, uint16_t cmdTriggerWait,  uint16_t minWaitNonBlockingMode)
  {
    bool ret = false;
    if ((is_created) && (is_connected)) // only, if connected ...
    {
      switch (last_mode)  // two different approaches, depending on the mode
      {
      case LxS::LXS_MODE_COMMAND: // ... command-mode
        {
          if (ExecuteCommand(LxS::LXS_CMD_TRIGGER_SINGLE_MEASUREMENT)) // execute trigger ...
          {
            bool r1 = true;
            bool r2 = true;
            bool r3 = true;
            if (resp_cmd == LxS::LXS_RESP_ACK_SUCCESS)  // if executed ...
            {
              Sleep(cmdTriggerWait + 1);
              if(is_high)
              {
                if(cmdTrigRespMask & (BIT0_IS_SET + BIT1_IS_SET))
                {
                  r1 = ExecuteCommand(LxS::LXS_CMD_GET_ZX_COORDINATES);   // get ZX-Data
                  r2 = r1;
                  Sleep(1);
                }
              }
              else
              {
                if(cmdTrigRespMask & BIT0_IS_SET)
                {
                  r1 = ExecuteCommand(LxS::LXS_CMD_GET_Z_COORDINATES);   // get Z-Data
                  Sleep(1);
                }
                if(cmdTrigRespMask & BIT1_IS_SET)
                {
                  r2 = ExecuteCommand(LxS::LXS_CMD_GET_X_COORDINATES);   // get X-Data
                  Sleep(1);
                }
              }

              if(cmdTrigRespMask & BIT2_IS_SET)
              {
                r3 = ExecuteCommand(LxS::LXS_CMD_GET_STATUS_TELEGRAM); // get Status
                Sleep(1);
              }
            }
            //r4 = (resp_cmd == LxS::LXS_RESP_DATA_FIELD);
            ret = (r1 && r2 && r3);
          }
          break;
        }
      case LxS::LXS_MODE_MEASURE: // ... measuring-mode
        {
          if(blockingFlag)
          {
            clock_t tm = clock();    // starting time in ms or timeout ...
            ret = ExecuteCommand(LxS::LXS_CMD_ETHERNET_TRIGGER, NULL, 0, false); // non-blocking trigger
            while ((async_sync) && ((clock() - tm) < time_out)) { Sleep(0); }; // wait for telegramm
          }
          else
          {
            ret = ExecuteCommand(LxS::LXS_CMD_ETHERNET_TRIGGER, NULL, 0, false); // non-blocking trigger
            Sleep(minWaitNonBlockingMode);
          }
          break;
        }
      };
    };
    return (ret);
  }

  //-------------------------------------------------------------------------------------------------------------------
  /**
  @fn       bool Device::SetSensorMode(LxS::eMode mode)
  @brief    Function to set a new sensor mode
  @return   Operation-state (new mode was set)
  @details
  Only "Measure Mode" and "Command Mode" can be set.
  */
  bool Device::SetSensorMode(LxS::eMode mode)
  {
    bool ret = false;
    if ((is_created) && (is_connected)) // only, if connected ...
    {
      if (mode == LxS::LXS_MODE_COMMAND)      // new mode should be "Command-Mode"
      {
        ret = ExecuteCommand(LxS::LXS_CMD_ENTER_COMMAND_MODE);  // execute ...
      }
      else if (mode == LxS::LXS_MODE_MEASURE) // new mode should be "Measuring-Mode"
      {
        ret = ExecuteCommand(LxS::LXS_CMD_EXIT_COMMAND_MODE);  // execute ...
      };
      ret = (ret | (last_mode == mode)); // done, or already in desired mode ?
    };
    return (ret);
  }

  //-------------------------------------------------------------------------------------------------------------------
  /**
  @fn       LxS::eMode Device::GetSensorMode(void)
  @brief    Request of actual sensor mode
  @return   Actual sensor mode
  @details
  Request of actual sensor mode.
  */
  LxS::eMode Device::GetSensorMode(void)
  {
    // extends return with "Invalid-Mode" for : "No Connection"
    return (((is_created) && (is_connected)) ? (last_mode) : (LxS::LXS_MODE_INVALID));
  }

  //-------------------------------------------------------------------------------------------------------------------
  /**
  @fn       bool Device::SetScanNumber(uint16_t scan)
  @brief    Set Scan-Number with new value
  @param    scan - New value for Scan-Number
  @return   Operation-state (Scan-Number was set)
  @details
  Overwrite the Scan-Number with new value.
  */
  bool Device::SetScanNumber(uint16_t scan)
  {
    bool ret = false;
    if ((is_created) && (is_connected)) // only, if connected ...
    {
      uint8_t * msg = new uint8_t[2]; // unmanaged : create buffer
      msg[0] = ((uint8_t)(scan & 0x00FF)); msg[1] = ((uint8_t)(scan >> 8)); // set new value ...
      ret = ExecuteCommand(LxS::LXS_CMD_SET_SCAN_NUMBER, msg, 2); // execute ...
      delete[] msg; // unmanaged : free buffer
    };
    return (ret);
  }

  //-------------------------------------------------------------------------------------------------------------------
  /**
  @fn       uint16_t Device::GetScanNumber(void)
  @brief    Request of actual Scan-Number
  @return   Actual Scan-Number
  @details
  Get actual Scan-Number from last incoming Package.
  */
  uint16_t Device::GetScanNumber(void)
  {
    return (last_scan);
  }

  //-------------------------------------------------------------------------------------------------------------------
  /**
  @fn       bool Device::SetEncoderValue(uint32_t enc)
  @brief    Set Encoder-Value
  @param    enc - New value for Encoder-Value
  @return   Operation-State (Encoder-Value was set)
  @details
  Overwrite the Encoder-Value with new value.
  */
  bool Device::SetEncoderValue(uint32_t enc)
  {
    bool ret = false;
    if ((is_created) && (is_connected)) // only, if connected ...
    {
      uint8_t * msg = new uint8_t[4]; // unmanaged : create buffer
      msg[0] = ((uint8_t)(enc & 0x000000FF));        // set new value ...
      msg[1] = ((uint8_t)((enc & 0x0000FF00) >> 8));
      msg[2] = ((uint8_t)((enc & 0x00FF0000) >> 16));
      msg[3] = ((uint8_t)((enc & 0xFF000000) >> 24));
      ret = ExecuteCommand(LxS::LXS_CMD_SET_ENCODER_VALUE, msg, 4); // execute ...
      delete[] msg; // unmanaged : free buffer
    };
    return (ret);
  }

  //-------------------------------------------------------------------------------------------------------------------
  /**
  @fn       uint32_t Device::GetEncoderValue(void)
  @brief    Request of actual Encoder-Value
  @return   Actual Encoder-Value
  @details
  Get actual Encoder-Value from last incoming package.
  */
  uint32_t Device::GetEncoderValue(void)
  {
    return (last_encoder);
  }

  //-------------------------------------------------------------------------------------------------------------------
  /**
  @fn       bool Device::SetInspectionTask(uint16_t taskNum, bool permanent)
  @brief    set a new inspection task
  @param    taskNum - inspection task number (0-15)
  @param    permanent - FALSE = volantile, TRUE = permanent
  @return   Operation-state (inspection task was set)
  @details
  "permanent" means, after power off / on the sensor starts with saved task number.
  */
  bool Device::SetInspectionTask(uint16_t taskNum, bool permanent)
  {
    bool ret = false;
    if ((is_created) && (is_connected)) // only, if connected ...
    {
      if ((taskNum >= LXS_MIN_TASK_NR) && (taskNum <= LXS_MAX_TASK_NR)) // ... and Task-Nr is OK
      {
        uint8_t * msg = new uint8_t[4]; // unmanaged : create buffer
        msg[0] = ((uint8_t)(taskNum & 0x00FF)); msg[1] = 0x00; // set task ...
        msg[2] = ((permanent) ? (0x01) : (0x00)); msg[3] = 0x00; // set permanent ...
        ret = ExecuteCommand(LxS::LXS_CMD_SET_INSPECTION_TASK, msg, 4); // execute ...
        delete[] msg; // unmanaged : free buffer
      };
    };
    return (ret);
  }

  //-------------------------------------------------------------------------------------------------------------------
  /**
  @fn       uint16_t Device::GetInspectionTask(void)
  @brief    Request actual inspection task number.
  @return   actual inspection task number (0 bis 15) or 0xFFFF on error.
  @details
  Request actual inspection task number.
  */
  uint16_t Device::GetInspectionTask(void)
  {
    uint16_t ret = 0xFFFF;
    if ((is_created) && (is_connected)) // only, if connected ...
    {
      if (ExecuteCommand(LxS::LXS_CMD_GET_INSPECTION_TASK)) // execute ...
      {
        if (resp_len == 2) { ret = ((uint16_t *)(resp_pkg))[0]; }; // return value
      };
    };
    return (ret);
  }

  //-------------------------------------------------------------------------------------------------------------------
  /**
  @fn       bool Device::SetActivation(bool state)
  @brief    Function to activate and deactivate LxS-device with ethernet telegram
  @param    state - state to be set (FALSE = deactivate; TRUE = activate)
  @return   Operation-state (activation was set)
  @details
  The state is saved internally in the member variable "last_EthActivState". This variable
  has no direct relation to the sensor variable. If the command was sucessful, it is considered that
  the member variable "last_EthActivState" has the same value as the sensor variable.
  Pay attention that this function only works in "Measure Mode" and that you didn't get an
  ACK-telegram.
  */
  bool Device::SetActivation(bool state)
  {
    bool ret = false;
    if ((is_created) && (is_connected)) // only, if connected ...
    {
      uint8_t * msg = new uint8_t[2]; // unmanaged : create buffer
      msg[0] = ((state) ? (0x01) : (0x00)); msg[1] = 0x00; // set new state ...
      last_EthActivState = state;
      clock_t tm = clock();    // starting time in ms or timeout ...
      ret = ExecuteCommand(LxS::LXS_CMD_ETHERNET_ACTIVATION, msg, 2, false); // execute, non-blocking ...
      //while ((async_sync) && ((clock() - tm) < time_out)) { Sleep(0); };
      delete[] msg; // unmanaged : free buffer
    };
    return (ret);
  }

  //-------------------------------------------------------------------------------------------------------------------
  /**
  @fn       bool Device::GetActivation(void)
  @brief    Get activation state
  @return   actual saved activation state
  @details
  Get last saved activation state from the member variable "last_EthActivState". This variable
  has no direct relation to the sensor variable. If the "SetActivation" command was sucessful, it is considered that
  the member variable "last_EthActivState" has the same value as the sensor variable.
  */
  bool Device::GetActivation(void)
  {
    return (last_EthActivState);
  }

  //-------------------------------------------------------------------------------------------------------------------
  /**
  @fn       LxS::eType Device::GetDeviceType(void)
  @brief    Request device type
  @return   Device type (or GENERAL as Generic-type)
  @details
  Request device type (Serial-Number) from connected device. Pay attention that this function only works in
  "Command Mode".
  */
  LxS::eType Device::GetDeviceType(void)
  {
    LxS::eType ret = LxS::LXS_TYPE_GENERAL;
    if ((is_created) && (is_connected)) // only, if connected ...
    {
      uint8_t * msg = new uint8_t[4]; // unmanaged : create buffer
      msg[0] = 0x01; msg[1] = 0x00; msg[2] = 0x08; msg[3] = 0x00; //0x0001:0x0008
      if (ExecuteCommand(LxS::LXS_CMD_GET_PARAMETER_BY_ID, msg, 4)) // execute ...
      {
        //! @remark : compatibility for old devices ...
        if (resp_len != 16) // case only for old devices (raised timeout with new devices)
        {
          msg[0]= 0x01; msg[1] = 0x00; msg[2] = 0x52; msg[3] = 0x08; // 0x0001:0x0852
          (void)ExecuteCommand(LxS::LXS_CMD_GET_PARAMETER_BY_ID, msg, 4); // execute ...
        };
        if (resp_len == 16) // now "resp_pkg" should be same for old and new devices ...
        {
          // Calculate Serial-Number from uint16_t-Package
          uint16_t * ptr = ((uint16_t *)(resp_pkg));
          int part_nr = 0;
          for (int i = 0; i < 8; i++) { part_nr *= 10; part_nr += ((uint8_t)(ptr[i] - 0x30)); };
          ret = ((LxS::eType)(part_nr)); // return ...
        };
      };
      delete[] msg; // unmanaged : free buffer
    };
    return (ret);
  }

  //-------------------------------------------------------------------------------------------------------------------
  /**
  @fn       const char * Device::GetDeviceName(LxS::eType type)
  @brief    Request description for device type
  @param    type - device type
  @return   Device name
  @details
  Request description for device type (Serial-Number).
  */
  const char * Device::GetDeviceName(LxS::eType type)
  {
    switch (type) // just number to string mapping
    {
    case LxS::LXS_TYPE_LPS_36_EN:	      return ((const char *)"LPS_36_EN");
    case LxS::LXS_TYPE_LPS_36:	        return ((const char *)"LPS_36");
    case LxS::LXS_TYPE_LES_36_VC:	      return ((const char *)"LES_36_VC");
    case LxS::LXS_TYPE_LES_36_PB:	      return ((const char *)"LES_36_PB");
    case LxS::LXS_TYPE_LES_36_EN_PB:	  return ((const char *)"LES_36_EN_PB");
    case LxS::LXS_TYPE_LES_36_HI_VC6:	  return ((const char *)"LES_36_HI_VC6");
    case LxS::LXS_TYPE_LRS_36_6:	      return ((const char *)"LRS_36_6");
    case LxS::LXS_TYPE_LRS_36_PB:	      return ((const char *)"LRS_36_PB");
    case LxS::LXS_TYPE_LPS_36_HI_EN:	  return ((const char *)"LPS_36_HI_EN");
    case LxS::LXS_TYPE_LPS_36_HI:	      return ((const char *)"LPS_36_HI");
    case LxS::LXS_TYPE_LRS_36_EN_PB:	  return ((const char *)"LRS_36_EN_PB");
    case LxS::LXS_TYPE_LPS_36_30:	      return ((const char *)"LPS_36_30");
    case LxS::LXS_TYPE_LRS_36_6_10:	    return ((const char *)"LRS_36_6_10");
    case LxS::LXS_TYPE_LES_36_VC6:	    return ((const char *)"LES_36_VC6");
    default:                            return ((const char *)"GENERAL");
    };
  }

  //-------------------------------------------------------------------------------------------------------------------
  /**
  @fn       LxS::Config * Device::GetDeviceConfiguration(void)
  @brief    Request actual device configuration
  @return   device configuration
  @details
  Request actual device configuration.
  */
  LxS::Config * Device::GetDeviceConfiguration(void)
  {
    LxS::Config * ret = NULL;
    if ((is_created) && (is_connected)) // only, if connected ...
    {
      ret = new LxS::Config(); // unmanaged : create config
      ret->type = GetDeviceType(); // request type
      for (int i = LXS_MIN_TASK_NR; i <= LXS_MAX_TASK_NR; i++) // request all tasks ...
      {
        ret->task[i] = internal_GetParamSet(i, 0);
      };
      ret->general = internal_GetParamSet(16, 1);
      ret->calibration = internal_GetParamSet(17, 2);
      ret->temp = internal_GetParamSet(19, 4);
    };
    //! @remark : user should free ressources with : delete[] ret; after using
    return (ret); //! @remark : !!! UNMANAGED RETURN !!!
  }

  //-------------------------------------------------------------------------------------------------------------------
  /**
  @fn       bool Device::SetDeviceConfiguration(LxS::Config * cfg)
  @brief    Download general settings to device
  @param    cfg - New device configuration
  @param    permanent - FALSE = volantile, TRUE = permanent
  @return   Operation-State (Configuration was set)
  @details
  Download general settings to device. The calibration parameter set is excluded, because this one is write protected.
  */
  bool Device::SetDeviceConfiguration(LxS::Config * cfg, bool permanent)
  {
    bool ret = false;
    if ((is_created) && (is_connected)) // only, if connected ...
    {
      LxS::eType ty = GetDeviceType(); // request type
      if (ty == cfg->type) // is configuration from same device ?
      {
        ret = true;
        for (int i = LXS_MIN_TASK_NR; i <= LXS_MAX_TASK_NR; i++) // set all tasks ...
        {
          ret = (ret & internal_SetParamSet(i,0,cfg->task[i], permanent));
        };
        ret = (ret & internal_SetParamSet(16,1,cfg->general, permanent));
        ret = (ret & internal_SetParamSet(19,4,cfg->temp, permanent));
      };
    };
    return (ret);
  }

  //-------------------------------------------------------------------------------------------------------------------
  /**
  @fn       LxS::ParamSet * Device::GetTaskSettings(uint16_t taskNum)
  @brief    Request actual parameter set for one inspection task
  @param    taskNum - inspection task number (0-15)
  @return   parameter set ("task[taskNum]")
  @details
  Request actual parameter-set for one device-task.
  */
  LxS::ParamSet * Device::GetTaskSettings(uint16_t taskNum)
  {
    LxS::ParamSet * ret = NULL;
    if ((is_created) && (is_connected)) // only, if connected ...
    {
      if ((taskNum >= LXS_MIN_TASK_NR) && (taskNum <= LXS_MAX_TASK_NR)) // ... and Task-Nr is OK
      {
        ret = internal_GetParamSet(taskNum, 0);
      };
    };
    //! @remark : user should free ressources with : ret->~ParamSet(); after using
    return (ret); //! @remark : !!! UNMANAGED RETURN !!!
  }

  //-------------------------------------------------------------------------------------------------------------------
  /**
  @fn       bool Device::SetTaskSettings(uint16_t taskNum, LxS::ParamSet * prm, bool permanent)
  @brief    Download settings for one device-task
  @param    taskNum - inspection task number
  @param    prm - Parameter-Set with Task-Settings
  @param    permanent - FALSE = volantile, TRUE = permanent
  @return   Operation-State (Task was set)
  @details
  Download settings for one device-task. After the download the sensor is set to the committed task number of the
  parameter set.
  */
  bool Device::SetTaskSettings(uint16_t taskNum, LxS::ParamSet * prm, bool permanent)
  {
    bool ret = false;
    if ((is_created) && (is_connected)) // only, if connected ...
    {
      if ((taskNum >= LXS_MIN_TASK_NR) && (taskNum <= LXS_MAX_TASK_NR)) // ... and Task-Nr is OK
      {
        ret = internal_SetParamSet(taskNum, 0, prm, permanent);
      };
    };
    return (ret);
  }

  //-------------------------------------------------------------------------------------------------------------------
  /**
  @fn       const char * Device::GetErrorText(uint16_t err_no)
  @brief    Request description for given Error-Number
  @param    err_no - Error-Number
  @return   Description of error as uint8_t array in ASCII-code.
  @details
  Request description for given Error-Number.
  */
  const char * Device::GetErrorText(uint16_t err_no)
  {
    uint8_t * ret = NULL;
    if ((is_created) && (is_connected)) // only, if connected ...
    {
      uint8_t * msg = new uint8_t[2]; // unmanaged : create buffer
      msg[0] = ((uint8_t)(err_no)); msg[1] = ((uint8_t)(err_no >> 8)); // set err_no ...
      if (ExecuteCommand(LxS::LXS_CMD_GET_ERROR_TEXT, msg, 2)) // execute ...
      {
        if (resp_len > 0) // has response ?
        {
          int len = (resp_len >> 1);
          ret = new uint8_t[len + 1]; // unmanaged : create answer
          uint16_t * ptr = ((uint16_t *)(resp_pkg)); // cast (uint8_t *) to (uint16_t *)
          for (int i = 0; i < len; i++)
          {
            ret[i] = ((uint8_t)(ptr[i])); // copy all values with cast to uint8_t
          };
          ret[len] = 0x00; // Null-Terminated String
        };
      };
      delete[] msg; // unmanaged : free buffer
    };
    //! @remark : user should free ressources with : delete[] ret; after using
    return ((const char *)ret); //! @remark : !!! UNMANAGED RETURN !!!
  }

  //-------------------------------------------------------------------------------------------------------------------
  /**
  @fn       LxS::Param * Device::GetSingleTaskParam(uint16_t param_id)
  @brief    Get a single parameter from actual inspection task
  @param    param_id - Parameter ID
  @return   returns Parameter-Datafield if command was sucessful and NULL if not
  @details
  Get a single Parameter from actual Inspection Task. Before calling this function the sensor must be
  in "Command Mode".
  */
  LxS::Param * Device::GetSingleTaskParam(uint16_t param_id)
  {
    LxS::Param * ret = NULL;
    if ((is_created) && (is_connected)) // only, if connected ...
    {
      uint8_t * msg = new uint8_t[2]; // unmanaged : create buffer
      msg[0] = ((uint8_t)(param_id)); msg[1] = ((uint8_t)(param_id >> 8)); // set param-id
      if (ExecuteCommand(LxS::LXS_CMD_GET_SINGLE_INSPECTION_TASK_PARAMETER, msg, 2)) // execute ...
      {
        if (resp_len > 16) // response must contains at least "structure-header"
        {
          uint16_t * ptr = ((uint16_t *)(resp_pkg));
          ret = new LxS::Param(&(ptr[0]), &(ptr[8])); // unmanaged : create parameter
        };
      };
      delete[] msg; // unmanaged : free buffer
    };
    //! @remark : user should free ressources with : ret->~Param(); after using
    return (ret); //! @remark : !!! UNMANAGED RETURN !!!
  }

  //-------------------------------------------------------------------------------------------------------------------
  /**
  @fn       bool Device::SetSingleTaskParam(uint16_t param_id, LxS::Param * prm, bool permanent)
  @brief    Set a single parameter in actual inspection task
  @param    param_id - Parameter ID
  @param    prm - Parameter with new values
  @param    permanent - FALSE = volantile, TRUE = permanent
  @return   Operation-state (task parameter was set)
  @details
  Set a single parameter in actual inspection task. Before calling this function the sensor must be
  in "Command Mode".
  */
  bool Device::SetSingleTaskParam(uint16_t param_id, LxS::Param * prm, bool permanent)
  {
    bool ret = false;
    if ((prm != NULL) && (is_created) && (is_connected)) // only, if connected ...
    {                                                                // ... and has Parameter ...
      int c2 = (prm->cnt << 1); // short to uint8_t-length
      uint8_t * msg = new uint8_t[c2 + 4]; // unmanaged : create buffer
      msg[0] = ((permanent) ? (0x01) : (0x00)); msg[1] = 0x00; // set permanent ...
      msg[2] = ((uint8_t)(param_id)); msg[3] = ((uint8_t)(param_id >> 8)); // set param-id
      memcpy(&(msg[4]), (uint8_t *)prm->data, c2); // copy param-data
      ret = ExecuteCommand(LxS::LXS_CMD_SET_SINGLE_INSPECTION_TASK_PARAMETER, msg, (c2 + 4)); // execute ...
      delete[] msg; // unmanaged : free buffer
    };
    return (ret);
  }

  //-------------------------------------------------------------------------------------------------------------------
  /**
  @fn       LxS::Param * Device::GetSingleUserParam(uint16_t param_id)
  @brief    Get a single User-Parameter
  @param    param_id - Parameter ID
  @return   returns Parameter-Datafield if command was sucessful and NULL if not
  @details
  Get a single User-Parameter from LxS-device. Before calling this function the sensor must be
  in "Command Mode".
  */
  LxS::Param * Device::GetSingleUserParam(uint16_t param_id)
  {
    LxS::Param * ret = NULL;
    if ((is_created) && (is_connected)) // only, if connected ...
    {
      uint8_t * msg = new uint8_t[2]; // unmanaged : create buffer
      msg[0] = 0x01; msg[1] = 0x00; // set 0x0001 ...
      if (ExecuteCommand(LxS::LXS_CMD_GET_SUPPORTED_PARAMETERS, msg, 2)) // execute ...
      {
        if (resp_len > 0) // has response ?
        {
          uint8_t * field = internal_Getuint8_ts(resp_pkg, 0, resp_len); // unmanaged : copy structure
          uint16_t * ptr = ((uint16_t *)(field));
          int str_len = (resp_len >> 1); // uint8_t to short-length
          int p = 0;
          bool found = false;
          while ((str_len > p) && (ptr[p] != 0)) // loop for all parameters in structure
          {
            if (ptr[p] == param_id) // if structure contains given parameter-id
            {
              found = true; // we has found structure for parameter !
              break;
            };
            p += 8;
          };
          if (found) // if found ...
          {
            msg[0] = ((uint8_t)(param_id)); msg[1] = ((uint8_t)(param_id >> 8)); // set param-id ...
            if (ExecuteCommand(LxS::LXS_CMD_GET_USER_PARAMETER, msg, 2)) // execute ...
            {
              if (resp_len > 0) // has response ?
              {
                ret = new LxS::Param(&(ptr[p]), (uint16_t *)resp_pkg); // unmanaged : create parameter
              };
            };
          };
          delete[] field; // unmanaged : free structure
        };
      };
      delete[] msg; // unmanaged : free buffer
    };
    //! @remark : user should free ressources with : ret->~Param(); after using
    return (ret); //! @remark : !!! UNMANAGED RETURN !!!
  }

  //-------------------------------------------------------------------------------------------------------------------
  /**
  @fn       bool Device::SetSingleUserParam(uint16_t param_id, LxS::Param * prm, bool permanent)
  @brief    Set a single User-Parameter
  @param    param_id - Parameter ID
  @param    prm - Parameter with new values
  @param    permanent - FALSE = volantile, TRUE = permanent
  @return   Operation-State (user parameter was set)
  @details
  Set a single User-Parameter. Before calling this function the sensor must be
  in "Command Mode".
  */
  bool Device::SetSingleUserParam(uint16_t param_id, LxS::Param * prm, bool permanent)
  {
    bool ret = false;
    if ((prm != NULL) && (is_created) && (is_connected)) // only, if connected ...
    {                                                                // ... and has Parameter ...
      int c2 = (prm->cnt << 1);
      uint8_t * msg = new uint8_t[c2 + 4];  // unmanaged : create buffer
      msg[0] = ((permanent) ? (0x01) : (0x00)); msg[1] = 0x00; // set permanent ...
      msg[2] = ((uint8_t)(param_id)); msg[3] = ((uint8_t)(param_id >> 8)); // set param-id
      memcpy(&(msg[4]), (uint8_t *)prm->data, c2); // copy param-data
      ret = ExecuteCommand(LxS::LXS_CMD_SET_USER_PARAMETER, msg, (c2 + 4)); // execute ...
      delete[] msg; // unmanaged : free buffer
    };
    return (ret);
  }

  //-------------------------------------------------------------------------------------------------------------------
  /**
  @fn       const char * Device::GetVersion(void)
  @brief    Get DLL-version
  @return   Version
  @details
  Get DLL-version as Null-Terminated-String.
  */
  const char * Device::GetVersion(void)
  {
    return (LXS_DLL_VERSION);
  }

  //-------------------------------------------------------------------------------------------------------------------
  /**
  @fn       uint16_t * Device::ReadResponsePackage(uint16_t * len)
  @brief    Read actual command response package with data length.
  @param    len gives back length of response package in words 
  @return   pointer on response package
  @details
  Read actual command response package with data length.
  */
  uint16_t * Device::ReadResponsePackage(uint16_t * len)
  {
    *len = resp_len / 2;
    uint16_t * ptr = ((uint16_t *)(resp_pkg)); // cast (uint8_t *) to (uint16_t *)
    return ptr;
  }


  //-------------------------------------------------------------------------------------------------------------------
  /**
  @fn       LxS::eCmd Device::internal_GetResponseCmd(LxS::eCmd cmd)
  @brief    This function returns the corresponding response command number to a given command number.
  @param	cmd command number.
  @return   corresponding response command number.
  */
  LxS::eCmd Device::internal_GetResponseCmd(LxS::eCmd cmd)
  {
    LxS::eCmd ret = LxS::LXS_RESP_ACK_SUCCESS;
    switch(cmd)
    {
    case LxS::LXS_CMD_SET_LASER_GATE:
    case LxS::LXS_CMD_TRIGGER_SINGLE_MEASUREMENT:
    case LxS::LXS_CMD_SET_ENCODER_VALUE:
    case LxS::LXS_CMD_SET_PARAMETER_SET:
    case LxS::LXS_CMD_SET_DEFAULT_CONFIGURATION:
    case LxS::LXS_CMD_SET_INSPECTION_TASK:
    case LxS::LXS_CMD_SET_USER_PARAMETER:
    case LxS::LXS_CMD_SET_SCAN_NUMBER:
    case LxS::LXS_CMD_SET_SINGLE_INSPECTION_TASK_PARAMETER:
    case LxS::LXS_CMD_ENTER_COMMAND_MODE:
    case LxS::LXS_CMD_EXIT_COMMAND_MODE:
    case LxS::LXS_CMD_CONNECT_TO_SENSOR:
    case LxS::LXS_CMD_DISCONNECT_FROM_SENSOR:
      {
        ret = LxS::LXS_RESP_ACK_SUCCESS;
        break;
      }
    case LxS::LXS_CMD_GET_X_COORDINATES:
    case LxS::LXS_CMD_GET_Z_COORDINATES:
    case LxS::LXS_CMD_GET_SUPPORTED_PARAMETERS:
    case LxS::LXS_CMD_GET_PARAMETER_SET:
    case LxS::LXS_CMD_GET_INSPECTION_TASK:
    case LxS::LXS_CMD_GET_PARAMETER_BY_ID:
    case LxS::LXS_CMD_GET_ERROR_TEXT:
    case LxS::LXS_CMD_GET_USER_PARAMETER:
    case LxS::LXS_CMD_GET_ZX_COORDINATES:
    case LxS::LXS_CMD_GET_USER_ACCESS_LEVEL:
    case LxS::LXS_CMD_GET_SINGLE_INSPECTION_TASK_PARAMETER:
    case LxS::LXS_CMD_EXECUTE_AREA_SCAN_BASIC_TEACH:
    case LxS::LXS_CMD_EXECUTE_TRACK_SCAN_TEACH:
    case LxS::LXS_CMD_EXECUTE_AREA_SCAN_ADVAN_TEACH:
      {
        ret = (LxS::eCmd)(cmd + 1);
        break;
      }
    case LxS::LXS_CMD_GET_STATUS_TELEGRAM:
      {
        ret = LxS::LXS_RESP_DATA_FIELD;
        break;
      }
    case LxS::LXS_CMD_ETHERNET_ACTIVATION:
    case LxS::LXS_CMD_ETHERNET_TRIGGER:
      {
        ret = cmd;
        break;
      }
    default:
      {
        ret = LxS::LXS_RESP_ACK_SUCCESS;
        break;
      }
    }
    return ret;
  }

  //-------------------------------------------------------------------------------------------------------------------
  /**
  @fn       bool Device::internal_CreateSocket(void)
  @brief    Create new socket
  @return   Operation-state (new socket was created)
  @details
  Create new (non-blocking) UDP-socket for communication with LxS-device.
  */
  bool Device::internal_CreateSocket(void)
  {
    bool ret = false;
    if (WSAStartup(MAKEWORD(2,2), &(wsa)) == NO_ERROR)  // startup WSA ...
    {
      sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);  // create UDP-Socket
      if (sock != INVALID_SOCKET)
      {
        if (bind(sock, ((ENDPOINT)&(dll_end_point)), sizeof(dll_end_point)) != SOCKET_ERROR)  // bind to pc-port
        {
          u_long iMode = 1; // non-blocking
          ret = (ioctlsocket(sock, FIONBIO, &iMode) == NO_ERROR); // set non-blocking-Propertie for Socket
          int iState = LXS_IN_BUFFER_LEN;
          (void)setsockopt(sock, SOL_SOCKET, SO_SNDBUF, (char *)&iState, 4);
          (void)setsockopt(sock, SOL_SOCKET, SO_RCVBUF, (char *)&iState, 4);
          iState = 0;
          (void)setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, (char *)&iState, 4);
          (void)setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (char *)&iState, 4);
        };
      };
    };
    if (!ret) { (void)WSACleanup(); }; // cleanup, if fails ...
    return (ret);
  }

  //-------------------------------------------------------------------------------------------------------------------
  /**
  @fn       bool Device::internal_StartThread(void)
  @brief    Start new thread
  @return   Operation-state (thread was started)
  @details
  Start new thread for UDP-communication with LxS-device.
  */
  bool Device::internal_StartThread(void)
  {
    thread = _beginthread(Device::LxS_Thread, 0, this);
    return (thread != NULL);
  }

  //-------------------------------------------------------------------------------------------------------------------
  /**
  @fn       uint8_t * Device::internal_CreatePackage(LxS::eCmd cmd, uint8_t * msg, int len)
  @brief    Create new package
  @param    cmd - Command
  @param    msg - Additional message
  @param    len - Length of additional message
  @return   New LxS-package
  @details
  Create new package from given command and additional message.
  */
  uint8_t * Device::internal_CreatePackage(LxS::eCmd cmd, uint8_t * msg, int len)
  {
    // pre-calculation ...
    int l16 = (len >> 1);
    uint8_t cmd_hi = ((uint8_t)(((uint16_t)(cmd)) >> 8));
    uint8_t cmd_lo = ((uint8_t)((uint16_t)(cmd)));
    uint8_t pkg_hi = ((uint8_t)(((uint16_t)(pkg_num)) >> 8));
    uint8_t pkg_lo = ((uint8_t)((uint16_t)(pkg_num)));
    uint8_t usr_hi = ((uint8_t)(((uint16_t)(l16)) >> 8));
    uint8_t usr_lo = ((uint8_t)((uint16_t)(l16)));

    int all_len = (30 + len);
    int offset = 0;
    uint8_t * data = new uint8_t[all_len]; // unmanaged : create package
    memset(data, 0, 30); // fill package witch zeros ...

    // Start sequence
    data[0] = data[1] = data[2] = data[3] = 0xFF;
    offset += 6;

    // Command number
    data[offset + 1] = cmd_hi;
    data[offset + 0] = cmd_lo;
    offset += 4;

    // Packet number
    data[offset + 1] = pkg_hi;
    data[offset + 0] = pkg_lo;
    offset += 4;
    pkg_num++;

    // Transaction number (same as command number)
    data[offset + 1] = data[offset - 7];
    data[offset + 0] = data[offset - 8];
    offset += 12;

    // Type (Fixed 0x0010)
    data[offset + 1] = 0x00;
    data[offset + 0] = 0x10;
    offset += 2;

    // Number of user data
    data[offset + 1] = usr_hi;
    data[offset + 0] = usr_lo;
    offset += 2;

    // Add additional Message
    if ((msg != NULL) && (len > 0))
    {
      memcpy(&(data[30]), msg, len);
    };

    //! @remark : user should free ressources with : delete[] data; after using
    return (data); //! @remark : !!! UNMANAGED RETURN !!!
  }

  //-------------------------------------------------------------------------------------------------------------------
  /**
  @fn       void Device::internal_IncomingPackage(int len)
  @brief    Handling for incoming package
  @param    len - Length of package
  @return   New LxS-package
  @details
  Handle incoming package, and execute next steps depends from package type.
  */
  void Device::internal_IncomingPackage(int len)
  {
    if (len >= 30) // LxS-device can send only packages >= 30 uint8_ts !
    {
      uint16_t * ptr = ((uint16_t *)(in_buffer));
      if ((ptr[0] == 0xFFFF) && (ptr[1] == 0xFFFF) && (ptr[2] == 0x0000)) // header ok ?
      {
        async_sync = false;

        // pre-calculation ...
        uint16_t cmdnum = ptr[3];
        uint16_t pkgnum = ptr[5];
        uint16_t trsnum = ptr[7];
        uint16_t status = ptr[8];
        uint16_t hi_w = ptr[9];
        uint16_t lo_w = ptr[10];
        uint16_t scnnum = ptr[12];
        uint16_t dtypes = ptr[13];
        uint16_t duint8_ts = ptr[14];

        // not needed anymore ...
        /* if (duint8_ts > 0) { uint8_t * usrbts = internal_Getuint8_ts(in_buffer, 30, (len - 30)); }; */

        // Cache neu Values for Application-Usage
        last_mode = ((LxS::eMode)(status & 0x00F0));
        last_state = status;
        last_scan = scnnum;
        last_encoder = ((((uint32_t)hi_w) << 16) | (lo_w));
        last_activation = ((status & 0x0100) == 0x0100);
        last_ethernet = ((status & 0x0001) == 0x0001);
        last_warning = ((status & 0x0200) == 0x0200);
        last_error = ((status & 0x2000) == 0x2000);

        // Analyse Package ...
        if (last_mode != LxS::LXS_MODE_ERROR)
        {
          // 1. Analysis ...
          LxS::eData dt = LxS::LXS_DATA_UNKNOWN;
          bool send_data = false;
          bool ack_resp = false;
          if (trsnum == 0x0000) // measuring-mode
          {
            dt = LxS::LXS_DATA_UNKNOWN;
            if(cmdnum == LXS_RESP_DATA_Z)
            {
              dt = LxS::LXS_DATA_Z;
            }
            if(cmdnum == LXS_RESP_DATA_X)
            {
              dt = LxS::LXS_DATA_X;
            }
            if(cmdnum == LXS_RESP_XZ_HIGH_DATA)
            {
              dt = LxS::LXS_DATA_ZX;
            }
            if(cmdnum == LXS_RESP_DATA_FIELD)
            {
              dt = LxS::LXS_DATA_FIELD;
            }
            send_data = true; // send package to app ...
          }
          else
          {
            // response for Trigger-Handling ?
            if ((trsnum == (uint16_t)LxS::LXS_CMD_GET_Z_COORDINATES) && (cmdnum == LxS::LXS_RESP_Z_COORDINATES))
            {
              dt = LxS::LXS_DATA_Z;
              send_data = true; // send package to app ...
            }
            if ((trsnum == (uint16_t)LxS::LXS_CMD_GET_X_COORDINATES) && (cmdnum == LxS::LXS_RESP_X_COORDINATES))
            {
              dt = LxS::LXS_DATA_X;
              send_data = true; // send package to app ...
            }
            if (trsnum == (uint16_t)LxS::LXS_CMD_GET_ZX_COORDINATES && (cmdnum == LxS::LXS_RESP_ZX_COORDINATES)) 
            {
              dt = LxS::LXS_DATA_ZX;
              send_data = true; // send package to app ...
            }
            if (trsnum == (uint16_t)LxS::LXS_CMD_GET_STATUS_TELEGRAM && (cmdnum == LxS::LXS_RESP_DATA_FIELD))
            {
              dt = LxS::LXS_DATA_FIELD;
              send_data = true; // send package to app ...
            }
            ack_resp = true; // send ACK to waiting client ...
          };

          // 2. Handling ...
          if (ack_resp) // we should ACK to waiting client ?
          {
            if (resp_cmd == ((LxS::eCmd)(trsnum)))
            {
              //! @remark : trick ! we free package before, an hold new package until next command for user available ...
              delete[] resp_pkg; // unmanaged : free response-data
              resp_cmd = ((LxS::eCmd)(cmdnum)); // response
              resp_len = (len - 30); // length
              resp_pkg = internal_Getuint8_ts(in_buffer, 30, resp_len); // unmanaged : copy response-data
              has_resp = true; // set response-flag for waiting client ...
            };
          };
          if ((send_data) && (cb_func != NULL)) // we should send-data ?
          {                                           // ... and we have DataHandler set ...
            uint8_t * out_bts = internal_Getuint8_ts(in_buffer, 30, (len - 30)); // unmanaged : copy data-package
            cb_func(dt, out_bts, (len - 30)); // execute DataHandler ...
            //! @remark : user should free ressources with : delete[] out_bts; after using
            //delete[] out_bts; // unmanaged : free data-package
          };
        };
        // else (Error-Mode ! => Bad-Device, we can nothing do ...)
        //delete[] usrbts;
      };
      // else ... Bad Header => drop
    };
    // else ... Drop others
  }

  //-------------------------------------------------------------------------------------------------------------------
  /**
  @fn       uint8_t * Device::internal_Getuint8_ts(uint8_t * bts, int offset, int len)
  @brief    Create copy from given array.
  @param    bts - original pointer
  @param    offset - start from
  @param    len - length
  @return   New uint8_t-Array
  @details
  Create a copy from given array.
  */
  uint8_t * Device::internal_Getuint8_ts(uint8_t * bts, int offset, int len)
  {
    uint8_t * ret = NULL;
    if (len > 0)
    {
      ret = new uint8_t[len]; // unmanaged : create buffer
      memcpy(ret, &(bts[offset]), len); // copy original data to buffer
      // delete[] ret; // unmanaged : free buffer
    };
    //! @remark : user should free ressources with : delete[] ret; after using
    return (ret); //! @remark : !!! UNMANAGED RETURN !!!
  }

  //-------------------------------------------------------------------------------------------------------------------
  /**
  @fn       LxS::ParamSet * Device::internal_GetParamSet(int id, int dtype)
  @brief    Request parameter set from device
  @param    id - Parameter set ID
  @param    dtype - Parameter set Type
  @return   Parameter set (or in error case = NULL)
  @details
  Request parameter set (see class LxS::ParamSet) from device with given ID and type.
  */
  LxS::ParamSet * Device::internal_GetParamSet(int id, int dtype)
  {
    LxS::ParamSet * ret = NULL;
    if ((is_created) && (is_connected)) // only, if connected ...
    {
      uint8_t * msg = new uint8_t[2]; // unmanaged : create buffer
      msg[0] = ((uint8_t)(dtype)); msg[1] = ((uint8_t)(dtype >> 8)); // set dtype ...
      if (ExecuteCommand(LxS::LXS_CMD_GET_SUPPORTED_PARAMETERS, msg, 2)) // get structure
      {
        if (resp_len > 0) // has response ?
        {
          int f_len = resp_len;
          uint8_t * field = internal_Getuint8_ts(resp_pkg, 0, f_len); // unmanaged : copy structure
          msg[0] = ((uint8_t)(id)); msg[1] = ((uint8_t)(id >> 8)); // set id ...
          if (ExecuteCommand(LxS::LXS_CMD_GET_PARAMETER_SET, msg, 2)) // get data
          {
            if (resp_len > 0) // has response ?
            {
              ret = new LxS::ParamSet(id, dtype, field, f_len, resp_pkg, resp_len);
            };
          };
          delete[] field; // unmanaged : free structure
        };
      };
      // else : (initial) ret->general = NULL;
      delete[] msg; // unmanaged : free buffer
    };
    //! @remark : user should free ressources with : delete[] ret; after using
    return (ret); //! @remark : !!! UNMANAGED RETURN !!!
  }

  //-------------------------------------------------------------------------------------------------------------------
  /**
  @fn       bool Device::internal_SetParamSet(int id, int dtype, LxS::ParamSet * prm, bool permanent)
  @brief    Send parameter set to Device
  @param    id - Parameter set-ID
  @param    dtype - Parameter set-Type
  @param    prm - Pointer to parameter set
  @param    permanent - FALSE = volantile, TRUE = permanent
  @return   Operation-state (parameter set was set)
  @details
  Send parameter set to device with given ID, Type and parameter set (see class LxS::ParamSet).
  */
  bool Device::internal_SetParamSet(int id, int dtype, LxS::ParamSet * prm, bool permanent)
  {
    bool ret = false;
    if ((is_created) && (is_connected) && (prm != NULL)) // only, if connected ...
    {                                                                // ... and has Parameter ...
      prm->id = id; // auto-correction for ID, if it was copied from another task ...
      uint8_t * msg = new uint8_t[2]; // unmanaged : create buffer
      msg[0] = ((uint8_t)(dtype)); msg[1] = ((uint8_t)(dtype >> 8)); // set dtype ...
      if (ExecuteCommand(LxS::LXS_CMD_GET_SUPPORTED_PARAMETERS, msg, 2)) // execute ...
      {
        if (resp_len > 0) // has response ?
        {
          // Create new Parameter-Set, to compare to given one ...
          LxS::ParamSet * test_prm = new LxS::ParamSet(id, dtype, resp_pkg, resp_len, NULL, 0); // unmanaged : test_prm
          if (test_prm->CheckStructure(prm)) // compare ...
          {
            // get package with values from given parameter-set
            int pkg_len = 0;
            uint8_t * pkg_data = prm->GenerateParameterPackage(&pkg_len); // unmanaged : create pkg_data
            if ((pkg_data != NULL) && (pkg_len > 0)) // generated ?
            {
              pkg_data[2] = ((permanent) ? (0x01) : (0x00)); // set permanent ...
              ret = (ExecuteCommand(LxS::LXS_CMD_SET_PARAMETER_SET, pkg_data, pkg_len)); // execute ...
            };
            delete[] pkg_data; // unmanaged : free pkg_data
          };
          test_prm->~ParamSet(); // unmanaged : free test_prm
        };
      };
      delete[] msg; // unmanaged : free buffer
    };
    return (ret);
  }
}
