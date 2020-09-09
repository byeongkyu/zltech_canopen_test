#include <k3_mobile_hw/can_ixxat_usb_compact_if.h>

CANIxxatUSBCompactInterface::CANIxxatUSBCompactInterface()
{
    dwCtrlHandle_ = ECI_INVALID_HANDLE;
}

CANIxxatUSBCompactInterface::~CANIxxatUSBCompactInterface()
{
    ECI109_Release();
}

bool CANIxxatUSBCompactInterface::init(uint16_t bitrate, uint16_t timeout)
{
    ECI_RESULT hResult = ECI_OK;
    ECI_HW_PARA astcHwPara[4] = {{0}};
    ECI_HW_INFO stcHwInfo = {0};
    DWORD dwIndex = 0;
    DWORD dwHwIndex = 0;
    DWORD dwCtrlIndex = 0;

    for(dwIndex=0; dwIndex < _countof(astcHwPara); dwIndex++)
    {
        astcHwPara[dwIndex].wHardwareClass = ECI_HW_USB;
        #ifdef ECIDEMO_HWUSEPOLLINGMODE
            astcHwPara[dwIndex].dwFlags = ECI_SETTINGS_FLAG_POLLING_MODE;
        #endif //ECIDEMO_HWUSEPOLLINGMODE
    }

    hResult = ECI109_Initialize(_countof(astcHwPara), astcHwPara);
    if(ECI_OK == hResult)
    {
        hResult = ECI109_GetInfo(dwHwIndex, &stcHwInfo);
        if(ECI_OK == hResult)
        {
            ROS_INFO("[CAN] Found IXXAT USB-CAN compact [%s] Interface...", stcHwInfo.u.V1.szHwSerial);
        }
    }

    ECI_CTRL_CAPABILITIES stcCtrlCaps = {0};
    DWORD dwCtrlFeatures = 0;
    ECI_CTRL_CONFIG stcCtrlConfig = {0};

    //*** Use Basic settings to open controller
    stcCtrlConfig.wCtrlClass                 = ECI_CTRL_CAN;
    stcCtrlConfig.u.sCanConfig.dwVer         = ECI_STRUCT_VERSION_V0;
    stcCtrlConfig.u.sCanConfig.u.V0.bBtReg0  = ECI_CAN_BT0_1000KB;
    stcCtrlConfig.u.sCanConfig.u.V0.bBtReg1  = ECI_CAN_BT1_1000KB;
    stcCtrlConfig.u.sCanConfig.u.V0.bOpMode  = ECI_CAN_OPMODE_STANDARD;

    //*** Open and Initialize given Controller of given board
    assert(ECI109_CtrlOpen(&dwCtrlHandle_, dwHwIndex, dwCtrlIndex, &stcCtrlConfig) == ECI_OK);

    stcCtrlCaps.wCtrlClass = ECI_CTRL_CAN;
    stcCtrlCaps.u.sCanCaps.dwVer = ECI_STRUCT_VERSION_V1;
    assert(ECI109_CtrlGetCapabilities(dwCtrlHandle_, &stcCtrlCaps) == ECI_OK);

    if(ECI_CTRL_CAN == stcCtrlCaps.wCtrlClass)
    {
        if(ECI_STRUCT_VERSION_V0 == stcCtrlCaps.u.sCanCaps.dwVer)
            dwCtrlFeatures = stcCtrlCaps.u.sCanCaps.u.V0.dwFeatures;
        else if(ECI_STRUCT_VERSION_V1 == stcCtrlCaps.u.sCanCaps.dwVer)
            dwCtrlFeatures = stcCtrlCaps.u.sCanCaps.u.V1.dwFeatures;
    }

    stcCtrlConfig.wCtrlClass = ECI_CTRL_CAN;
    stcCtrlConfig.u.sCanConfig.dwVer = ECI_STRUCT_VERSION_V0;

    switch(bitrate)
    {
        case 1000:
            stcCtrlConfig.u.sCanConfig.u.V0.bBtReg0  = ECI_CAN_BT0_1000KB;
            stcCtrlConfig.u.sCanConfig.u.V0.bBtReg1  = ECI_CAN_BT1_1000KB;
            break;
        case 500:
            stcCtrlConfig.u.sCanConfig.u.V0.bBtReg0  = ECI_CAN_BT0_500KB;
            stcCtrlConfig.u.sCanConfig.u.V0.bBtReg1  = ECI_CAN_BT1_500KB;
            break;
        case 250:
            stcCtrlConfig.u.sCanConfig.u.V0.bBtReg0  = ECI_CAN_BT0_250KB;
            stcCtrlConfig.u.sCanConfig.u.V0.bBtReg1  = ECI_CAN_BT1_250KB;
            break;
        case 100:
            stcCtrlConfig.u.sCanConfig.u.V0.bBtReg0  = ECI_CAN_BT0_100KB;
            stcCtrlConfig.u.sCanConfig.u.V0.bBtReg1  = ECI_CAN_BT1_100KB;
            break;
        default:
            ROS_WARN("Can't support %d bitrate...", bitrate);
            return false;
    }

    stcCtrlConfig.u.sCanConfig.u.V0.bOpMode  = ECI_CAN_OPMODE_STANDARD | ECI_CAN_OPMODE_EXTENDED;
    stcCtrlConfig.u.sCanConfig.u.V0.bOpMode |= (dwCtrlFeatures & ECI_CAN_FEATURE_ERRFRAME) ? ECI_CAN_OPMODE_ERRFRAME : 0;

    //*** Re-configure given controller of given board
    assert(ECI109_CtrlOpen(&dwCtrlHandle_, dwHwIndex, dwCtrlIndex, &stcCtrlConfig) == ECI_OK);
    assert(ECI109_CtrlStart(dwCtrlHandle_) == ECI_OK);

    timeout_ = timeout;

    ROS_INFO("[CAN] initialized...");
    return true;
}

bool CANIxxatUSBCompactInterface::send_message(CANMsg msg)
{
    ECI_RESULT hResult = ECI_OK;
    ECI_CTRL_MESSAGE stcCtrlMsg = {0};

    stcCtrlMsg.wCtrlClass = ECI_CTRL_CAN;
    stcCtrlMsg.u.sCanMessage.dwVer = ECI_STRUCT_VERSION_V1;
    stcCtrlMsg.u.sCanMessage.u.V0.dwMsgId = (msg.get_ID() % (ECI_CAN_MAX_11BIT_ID + 1));
    stcCtrlMsg.u.sCanMessage.u.V0.uMsgInfo.Bits.dlc = (msg.get_length() % (_countof(stcCtrlMsg.u.sCanMessage.u.V1.abData) + 1));

    for(uint8_t i = 0; i < msg.get_length(); i++)
    {
        stcCtrlMsg.u.sCanMessage.u.V0.abData[i] = msg.get_at(i);
    }

    hResult = ECI109_CtrlSend(dwCtrlHandle_, &stcCtrlMsg, timeout_);
    if(hResult != ECI_OK)
    {
        ROS_WARN("Error: send_message to [%d]", msg.get_ID());
        return false;
    }

    return true;
}

bool CANIxxatUSBCompactInterface::recv_message(CANMsg *msg)
{
    ECI_RESULT hResult = ECI_OK;
    ECI_CTRL_MESSAGE stcCtrlMsg = {0};
    DWORD dwCount = 1;

    hResult = ECI109_CtrlReceive(dwCtrlHandle_, &dwCount, &stcCtrlMsg, timeout_);
    if(ECI_ERR_NO_MORE_DATA == hResult)
        return false;

    if(hResult != ECI_OK)
    {
        ROS_WARN("Error: recv_message.");
        return false;
    }


    msg->set_ID(stcCtrlMsg.u.sCanMessage.u.V0.dwMsgId);
    msg->set_length(stcCtrlMsg.u.sCanMessage.u.V0.uMsgInfo.Bits.dlc);
    msg->set_extended(stcCtrlMsg.u.sCanMessage.u.V0.uMsgInfo.Bits.ext);

    if(ECI_CAN_MSGTYPE_ERROR == stcCtrlMsg.u.sCanMessage.u.V0.uMsgInfo.Bits.type)
        return true;

    // ROS_INFO("Type: %d", stcCtrlMsg.u.sCanMessage.u.V0.uMsgInfo.Bits.type);

    int length = stcCtrlMsg.u.sCanMessage.u.V0.uMsgInfo.Bits.dlc;
    for(int i = 0; i < length; i++)
    {
        msg->set_at(i, stcCtrlMsg.u.sCanMessage.u.V0.abData[i]);
    }
    return true;
}
