
#include "LxS_Lib/LxS_Datagram.h"

using namespace LxS;

DatagramPtr LxS::convertToInheritedDatagram(const Datagram & dg) {
    switch (dg.cmd) {
        case LXS_RESP_ACK_SUCCESS:
            return DatagramPtr(new RespAckSuccess(dg));
        case LXS_RESP_ACK_FAILURE:
            return DatagramPtr(new RespAckFailure(dg));
        case LXS_RESP_DATA_FIELD:
            return DatagramPtr(new RespDataField(dg));
        case LXS_RESP_DATA_X:
            return DatagramPtr(new RespDataX(dg));
        case LXS_RESP_DATA_Z:
            return DatagramPtr(new RespDataZ(dg));
        case LXS_RESP_XZ_HIGH_DATA:
            return DatagramPtr(new RespXZHighData(dg));
        case LXS_RESP_X_COORDINATES:
            return DatagramPtr(new RespXCoordinates(dg));
        case LXS_RESP_Z_COORDINATES:
            return DatagramPtr(new RespZCoordinates(dg));
        case LXS_CMD_ETHERNET_ACTIVATION:
            return DatagramPtr(new CmdEthernetActivation(dg));
        case LXS_CMD_ETHERNET_TRIGGER:
            return DatagramPtr(new CmdEthernetTrigger(dg));
        case LXS_RESP_USER_PARAMETER:
            return DatagramPtr(new RespUserParameter(dg));
        case LXS_RESP_GET_SUPPORTED_PARAMETERS:
            return DatagramPtr(new RespSupportedParameters(dg));
        case LXS_RESP_GET_PARAMETER_SET:
            return DatagramPtr(new RespParameterSet(dg));
        default:
            {
                bool RespDecodingIsReady = false;
                assert(RespDecodingIsReady);
                return DatagramPtr();
            }
    }
}
