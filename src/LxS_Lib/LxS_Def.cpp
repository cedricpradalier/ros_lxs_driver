
#include "LxS_Lib/LxS_Def.h"

using namespace LxS;
    //---------------------------------------------------------------------------------------------------------------------
    /**
      @enum       eData
      @brief      different possibilities for data-packages
      */
const char * LxS::eDataString(uint16_t data) {
    return eDataString((eData)data);
}

const char * LxS::eDataString(eData data) {
    switch (data) {
        case LXS_DATA_UNKNOWN: return "DATA_UNKNOWN"; 
        case LXS_DATA_Z: return "DATA_Z";       
        case LXS_DATA_X: return "DATA_X";       
        case LXS_DATA_ZX: return "DATA_ZX";      
        case LXS_DATA_FIELD: return "DATA_FIELD";   
        default: return "DATA_INVALID";
    }
}

eCmd LxS::eCmdResponse(eCmd cmd) {
    LxS::eCmd ret = LxS::LXS_RESP_ACK_SUCCESS;
    switch(cmd) {
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
        case LxS::LXS_CMD_GET_INSPECTION_TASK:
        case LxS::LXS_CMD_GET_PARAMETER_BY_ID:
        case LxS::LXS_CMD_GET_ERROR_TEXT:
        case LxS::LXS_CMD_GET_USER_ACCESS_LEVEL:
        case LxS::LXS_CMD_GET_SINGLE_INSPECTION_TASK_PARAMETER:
        case LxS::LXS_CMD_EXECUTE_AREA_SCAN_BASIC_TEACH:
        case LxS::LXS_CMD_EXECUTE_TRACK_SCAN_TEACH:
        case LxS::LXS_CMD_EXECUTE_AREA_SCAN_ADVAN_TEACH:
            {
                ret = (LxS::eCmd)(cmd + 1);
                break;
            }
        case LxS::LXS_CMD_GET_USER_PARAMETER:
            {
                ret = LxS::LXS_RESP_USER_PARAMETER;
                break;
            }
        case LxS::LXS_CMD_GET_ZX_COORDINATES:
            {
                ret = LxS::LXS_RESP_ZX_COORDINATES;
                break;
            }
        case LxS::LXS_CMD_GET_PARAMETER_SET:
            {
                ret = LxS::LXS_RESP_GET_PARAMETER_SET;
                break;
            }
        case LxS::LXS_CMD_GET_SUPPORTED_PARAMETERS:
            {
                ret = LxS::LXS_RESP_GET_SUPPORTED_PARAMETERS;
                break;
            }
        case LxS::LXS_CMD_GET_X_COORDINATES:
            {
                ret = LxS::LXS_RESP_X_COORDINATES;
                break;
            }
        case LxS::LXS_CMD_GET_Z_COORDINATES:
            {
                ret = LxS::LXS_RESP_Z_COORDINATES;
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

const char * LxS::eCmdString(uint16_t cmd) {
    return eCmdString((eCmd)cmd);
}

const char * LxS::eCmdString(eCmd cmd) {
    switch (cmd) {
        case LXS_CMD_SET_LASER_GATE: return "CMD_SET_LASER_GATE";                       
        case LXS_CMD_TRIGGER_SINGLE_MEASUREMENT: return "CMD_TRIGGER_SINGLE_MEASUREMENT";           
        case LXS_CMD_GET_X_COORDINATES: return "CMD_GET_X_COORDINATES";                    
        case LXS_RESP_X_COORDINATES: return "RESP_X_COORDINATES";                       
        case LXS_CMD_GET_Z_COORDINATES: return "CMD_GET_Z_COORDINATES";                    
        case LXS_RESP_Z_COORDINATES: return "RESP_Z_COORDINATES";                       
        case LXS_CMD_SET_ENCODER_VALUE: return "CMD_SET_ENCODER_VALUE";                    
        case LXS_CMD_GET_SUPPORTED_PARAMETERS: return "CMD_GET_SUPPORTED_PARAMETERS";             
        case LXS_CMD_GET_PARAMETER_SET: return "CMD_GET_PARAMETER_SET";                    
        case LXS_CMD_SET_PARAMETER_SET: return "CMD_SET_PARAMETER_SET";                    
        case LXS_CMD_GET_INSPECTION_TASK: return "CMD_GET_INSPECTION_TASK";                  
        case LXS_CMD_SET_INSPECTION_TASK: return "CMD_SET_INSPECTION_TASK";                  
        case LXS_CMD_SET_DEFAULT_CONFIGURATION: return "CMD_SET_DEFAULT_CONFIGURATION";            
        case LXS_CMD_GET_PARAMETER_BY_ID: return "CMD_GET_PARAMETER_BY_ID";                  
        case LXS_CMD_SET_SCAN_NUMBER: return "CMD_SET_SCAN_NUMBER";                      
        case LXS_CMD_GET_STATUS_TELEGRAM: return "CMD_GET_STATUS_TELEGRAM";                  
        case LXS_CMD_SET_USER_PARAMETER: return "CMD_SET_USER_PARAMETER";                   
        case LXS_CMD_GET_USER_PARAMETER: return "CMD_GET_USER_PARAMETER";                   
        case LXS_RESP_USER_PARAMETER: return "RESP_USER_PARAMETER";                      
        case LXS_CMD_GET_ZX_COORDINATES: return "CMD_GET_ZX_COORDINATES";					 
        case LXS_RESP_ZX_COORDINATES: return "RESP_ZX_COORDINATES";						 
        case LXS_CMD_GET_ERROR_TEXT: return "CMD_GET_ERROR_TEXT";                       
        case LXS_CMD_GET_USER_ACCESS_LEVEL: return "CMD_GET_USER_ACCESS_LEVEL";                
        case LXS_CMD_SET_SINGLE_INSPECTION_TASK_PARAMETER: return "CMD_SET_SINGLE_INSPECTION_TASK_PARAMETER"; 
        case LXS_CMD_GET_SINGLE_INSPECTION_TASK_PARAMETER: return "CMD_GET_SINGLE_INSPECTION_TASK_PARAMETER"; 
        case LXS_RESP_TASK_PARAMETER: return "RESP_TASK_PARAMETER";                      
        case LXS_CMD_EXECUTE_AREA_SCAN_BASIC_TEACH: return "CMD_EXECUTE_AREA_SCAN_BASIC_TEACH";        
        case LXS_CMD_EXECUTE_TRACK_SCAN_TEACH: return "CMD_EXECUTE_TRACK_SCAN_TEACH";             
        case LXS_CMD_EXECUTE_AREA_SCAN_ADVAN_TEACH: return "CMD_EXECUTE_AREA_SCAN_ADVAN_TEACH";        
        case LXS_CMD_ENTER_COMMAND_MODE: return "CMD_ENTER_COMMAND_MODE";                   
        case LXS_CMD_EXIT_COMMAND_MODE: return "CMD_EXIT_COMMAND_MODE";                    
        case LXS_RESP_ACK_SUCCESS: return "RESP_ACK_SUCCESS";                         
        case LXS_RESP_ACK_FAILURE: return "RESP_ACK_FAILURE";                         
        case LXS_CMD_CONNECT_TO_SENSOR: return "CMD_CONNECT_TO_SENSOR";                    
        case LXS_CMD_DISCONNECT_FROM_SENSOR: return "CMD_DISCONNECT_FROM_SENSOR";               
        case LXS_CMD_ETHERNET_ACTIVATION: return "CMD_ETHERNET_ACTIVATION";                  
        case LXS_CMD_ETHERNET_TRIGGER: return "CMD_ETHERNET_TRIGGER";                     
        case LXS_RESP_DATA_FIELD: return "RESP_DATA_FIELD";                          
        case LXS_RESP_DATA_X: return "RESP_DATA_X";                              
        case LXS_RESP_XZ_HIGH_DATA: return "RESP_XZ_HIGH_DATA";                        
        case LXS_RESP_DATA_Z: return "RESP_DATA_Z";                              
        case LXS_RESP_GET_PARAMETER_SET: return "RESP_GET_PARAMETER_SET";                              
        case LXS_CMD_UNSET: return "";                              
        default: return "CMD_INVALID";
    } 
}

const char * LxS::eModeString(uint16_t mode) {
    return eModeString((eMode)mode);
}

const char * LxS::eModeString(eMode mode) {
    switch (mode) {
        case LXS_MODE_INVALID: return "MODE_INVALID"; 
        case LXS_MODE_MEASURE: return "MODE_MEASURE"; 
        case LXS_MODE_MENU: return "MODE_MENU";    
        case LXS_MODE_COMMAND: return "MODE_COMMAND"; 
        case LXS_MODE_ERROR: return "MODE_ERROR";   
        default: return "MODE_INVALID";
    }
}

const char * LxS::eTypeString(uint16_t type) {
    return eTypeString((eType)type);
}

const char * LxS::eTypeString(eType type) {
    switch (type) {
        case LXS_TYPE_GENERAL: return "TYPE_GENERAL";       
        case LXS_TYPE_LPS_36_EN: return "TYPE_LPS_36_EN";     
        case LXS_TYPE_LPS_36: return "TYPE_LPS_36";        
        case LXS_TYPE_LES_36_VC: return "TYPE_LES_36_VC";     
        case LXS_TYPE_LES_36_PB: return "TYPE_LES_36_PB";     
        case LXS_TYPE_LES_36_EN_PB: return "TYPE_LES_36_EN_PB";  
        case LXS_TYPE_LES_36_HI_VC6: return "TYPE_LES_36_HI_VC6"; 
        case LXS_TYPE_LRS_36_6: return "TYPE_LRS_36_6";      
        case LXS_TYPE_LES_36_HI_PB: return "TYPE_LES_36_HI_PB";  
        case LXS_TYPE_LRS_36_PB: return "TYPE_LRS_36_PB";     
        case LXS_TYPE_LES_36_VC6: return "TYPE_LES_36_VC6";    
        case LXS_TYPE_LPS_36_HI_EN: return "TYPE_LPS_36_HI_EN";  
        case LXS_TYPE_LPS_36_HI: return "TYPE_LPS_36_HI";     
        case LXS_TYPE_LRS_36_EN_PB: return "TYPE_LRS_36_EN_PB";  
        case LXS_TYPE_LPS_36_30: return "TYPE_LPS_36_30";     
        case LXS_TYPE_LRS_36_6_10: return "TYPE_LRS_36_6_10";   
        default: return "TYPE_INVALID";
    }
}

