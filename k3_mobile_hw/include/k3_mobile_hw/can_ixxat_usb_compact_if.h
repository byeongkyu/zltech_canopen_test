#ifndef CAN_IXXAT_USB_COMPACT_IF_H_
#define CAN_IXXAT_USB_COMPACT_IF_H_

#include <ros/ros.h>

#include <ECI109.h>
#include <ECI_pshpack1.h>
#include <ECI_poppack.h>


#ifdef __cplusplus
extern "C"
{
#endif

void EciPrintHwInfo(const ECI_HW_INFO* pstcHwInfo);
void EciPrintCtrlCapabilities(const ECI_CTRL_CAPABILITIES* pstcCtrlCaps);
void EciPrintCtrlMessage(const ECI_CTRL_MESSAGE* pstcCtrlMsg);

ECI_RESULT EciGetNthCtrlOfClass(const ECI_HW_INFO* stcHwInfo,
                                    e_CTRLCLASS eCtrlClass,
                                    DWORD dwRelCtrlIndex,
                                    DWORD* pdwCtrIndex);
#ifdef __cplusplus
};
#endif


class CANMsg
{
    public:
        CANMsg():
            id_(0), extended_(false), len_(0)
        {
            memset(data_, 0, sizeof(uint8_t) * 8);
        }
        ~CANMsg() {}

    public:
        uint16_t get_ID()
        {
            return id_;
        }

        void set_ID(uint16_t id)
        {
            id_ = id;
        }

        bool get_extended()
        {
            return extended_;
        }

        bool set_extended(bool extended)
        {
            extended_ = extended;
        }

        uint8_t get_length()
        {
            return len_;
        }

        void set_length(uint8_t len)
        {
            len_ = len;
        }

        void get(uint8_t *data0, uint8_t *data1, uint8_t *data2, uint8_t *data3, uint8_t *data4, uint8_t *data5, uint8_t *data6, uint8_t *data7)
        {
            *data0 = data_[0];
            *data1 = data_[1];
            *data2 = data_[2];
            *data3 = data_[3];
            *data4 = data_[4];
            *data5 = data_[5];
            *data6 = data_[6];
            *data7 = data_[7];
        }

        uint8_t get_at(uint8_t index)
        {
            return data_[index];
        }

        void set(uint8_t data0=0, uint8_t data1=0, uint8_t data2=0, uint8_t data3=0, uint8_t data4=0, uint8_t data5=0, uint8_t data6=0, uint8_t data7=0)
        {
            memset(data_, 0, sizeof(uint8_t) * 8);

            data_[0] = data0;
            data_[1] = data1;
            data_[2] = data2;
            data_[3] = data3;
            data_[4] = data4;
            data_[5] = data5;
            data_[6] = data6;
            data_[7] = data7;
        }

        void set_at(uint8_t index, uint8_t data)
        {
            data_[index] = data;
        }

        void clear()
        {
            id_ = 0;
            extended_ = false;
            len_ = 0;
            memset(data_, 0, sizeof(uint8_t) * 8);
        }

        void printMessage()
        {
            ROS_INFO("[CAN_MSG]");
            ROS_INFO("========");
            ROS_INFO("  ID: 0x%X", id_);
            ROS_INFO("  Extended: %d", extended_);
            ROS_INFO("  Length: %d", len_);
            ROS_INFO("  Data: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x", data_[0], data_[1], data_[2], data_[3], data_[4], data_[5], data_[6], data_[7]);
            ROS_INFO("========");
        }


    private:
        uint16_t id_;
        bool extended_;
        uint8_t len_;
        uint8_t data_[8];
};

class CANIxxatUSBCompactInterface
{
    public:
        CANIxxatUSBCompactInterface();
        ~CANIxxatUSBCompactInterface();

    public:
        bool init(uint16_t bitrate, uint16_t timeout=5);
        bool send_message(CANMsg msg);
        bool recv_message(CANMsg *msg);

    private:
        ECI_CTRL_HDL dwCtrlHandle_;
        uint16_t timeout_;
};


#endif //CAN_IXXAT_USB_COMPACT_IF_H_