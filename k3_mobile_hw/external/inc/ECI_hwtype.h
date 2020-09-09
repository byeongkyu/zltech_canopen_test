///////////////////////////////////////////////////////////////////////////////
// (C) 2008-2012 IXXAT Automation GmbH, all rights reserved
///////////////////////////////////////////////////////////////////////////////
/**
  Definition of types, structs and unions for hardware initialization and 
  communication.

  @file ECI_hwtype.h
*/

#ifndef __ECI_HWTYPE_H__
#define __ECI_HWTYPE_H__


//////////////////////////////////////////////////////////////////////////
// include files
#include "ECI_cantype.h"
#include "ECI_lintype.h"
#include "ECI_thread.h"

#include <ECI_pshpack1.h>

//////////////////////////////////////////////////////////////////////////
// constants and macros


/** Timeout definition for polling mode. */
#define ECI_NO_WAIT          (0)
/** Timeout definition for blocking mode. */
#define ECI_WAIT_FOREVER     (0xFFFFFFFF)

/** Definition of an invalid handle. */
#define ECI_INVALID_HANDLE   0

/** Maximum supported boards by the ECI API. @ingroup HwTypes */
#define ECI_MAXBOARDCOUNT    32
/** Maximum supported controllers per board. @ingroup CtrlTypes */
#define ECI_MAXCTRLCOUNT     8


//////////////////////////////////////////////////////////////////////////
// data types

/** 
  ECI controller handle 

  @ingroup CtrlTypes
*/
typedef DWORD ECI_CTRL_HDL;

/**
  ECI structure versions
*/
typedef enum
{
  ECI_STRUCT_VERSION_V0 = 0,      ///< Version 0
  ECI_STRUCT_VERSION_V1,          ///< Version 1
  ECI_STRUCT_VERSION_V2,          ///< Version 2
  ECI_STRUCT_VERSION_V3           ///< Version 3
} e_ECI_STRUCT_VERSION;


/**
  Library states

  @ingroup HwTypes
*/
typedef enum
{
  ECI_UNINITIALIZED    = 0x00,     ///< Library is uninitialized
  ECI_INITIALIZED      = 0x01,     ///< Library is initialized
  ECI_CONFIGURED       = 0x02      ///< Library in the configured state
} e_LIBSTATES;


/**
  Controller states

  @ingroup CtrlTypes
*/
typedef enum
{
  ECI_CTRL_UNCONFIGURED = 0x00,    ///< Controller is unconfigured
  ECI_CTRL_INITIALIZED  = 0x01,    ///< Controller is initialized
  ECI_CTRL_RUNNING      = 0x02     ///< Controller is running
} e_CTRLSTATES;


/**
  Hardware classes

  @ingroup HwTypes
*/
typedef enum
{
  ECI_HW_UNDEFINED  = 0x00,        ///< undefined
  ECI_HW_PCI        = 0x01,        ///< PCI hardware like iPCI-165/PCI
  ECI_HW_ISA        = 0x02,        ///< ISA hardware like iPCI-320/ISA
  ECI_HW_USB        = 0x03,        ///< USB hardware like USB-to-CAN compact
  ECI_HW_IP         = 0x04         ///< IP hardware like CAN\@net NT
} e_HWCLASS;


/**
  Controller classes

  @ingroup CtrlTypes
*/
typedef enum
{
  ECI_CTRL_UNDEFINED = 0x00,       ///< undefined
  ECI_CTRL_CAN       = 0x01,       ///< CAN controller
  ECI_CTRL_LIN       = 0x02,       ///< LIN controller
  ECI_CTRL_FLX       = 0x03,       ///< FlexRay controller
  ECI_CTRL_KLI       = 0x04        ///< K-Line controller
} e_CTRLCLASS;


/**
  Controller settings flags
  Bit coded information, resulting value can be any combination of values below.

  @ingroup CtrlTypes
*/
typedef enum
{
  /** No flags used */
  ECI_SETTINGS_FLAG_NONE            = 0x00000000, 

  /** If set the hardware is initialized in polling mode, that means no interrupt 
      is assigned and the interrupt line is not used. To use the recommended 
      interrupt driven mode set this flag resp. bit to zero.
      Please also refer to \ref api_InterruptVsPolling. */
  ECI_SETTINGS_FLAG_POLLING_MODE    = 0x00000001,

  /** If set the the board is opened without verifying the ECI firmware after 
      downloading it. This switch can be used if start time of the board shall
      by reduced. */
  ECI_SETTINGS_FLAG_NO_FW_VERIFY    = 0x40000000,

  /** If set the the board is opened without downloading the ECI firmware. This
      flag is only used by maintenance programs and must not be set by any ECI
      application */
  ECI_SETTINGS_FLAG_NO_FW_DOWNLOAD  = 0x80000000
} e_SETTINGS_FLAGS;


/**
  Controller stop flags

  @ingroup CtrlTypes
*/
typedef enum
{
  /** No flags used */
  ECI_STOP_FLAG_NONE            = 0x00000000, 

  /** If set an ongoing transmission is aborted and the controller configuration is reset. (RESET) @n
      If not set the controller is stopped after an ongoing transmission, the configuration is kept (STOP). */
  ECI_STOP_FLAG_RESET_CTRL      = 0x00000001, 
                                              
  /** If set all entries from TX FIFO are cleared @n
      If not set all entries are kept in TX FIFO. */
  ECI_STOP_FLAG_CLEAR_TX_FIFO   = 0x00000002, 

  /** If set all entries from RX FIFO are cleared. @n
      If not set all entries are kept in RX FIFO. @n
      @note 
      Please note if RX FIFO clearing is not enabled while re-starting the controller. The timestamps
      are also not reset upon re-start of the controller.
      */
  ECI_STOP_FLAG_CLEAR_RX_FIFO   = 0x00000004
} e_STOP_FLAGS;


/**
  Definition of the PCI hardware configuration parameters.
  
  @ingroup HwTypes
*/
typedef struct
{
  DWORD dwVer;                 ///< Version of valid union struct

  union
  {
    struct
    {
      DWORD dwReserved;        ///< Reserved for future use
    } __PACKED__ V0;                      ///< Version 0
	struct
    {
      ECI_SCHEDULER_SETTINGS sSchedSettings;  ///< Scheduler settings
    } __PACKED__ V1;                          ///< Version 1
  } __PACKED__ u;              ///< Version controlled structs container
} __PACKED__ ECI_PCI_SETTINGS;


/**
  Definition of the ISA hardware configuration parameters.
  
  @ingroup HwTypes
*/
typedef struct
{
  DWORD dwVer;                 ///< Version of valid union struct

  union
  {
    struct
    {
      DWORD dwAddress;         ///< Address of the ISA hardware
      BYTE  bInterrupt;        ///< Interrupt of the ISA hardware
    } __PACKED__ V0;           ///< Version 0
	struct
    {
	  DWORD dwAddress;                        ///< Address of the ISA hardware
      BYTE  bInterrupt;                       ///< Interrupt of the ISA hardware
      ECI_SCHEDULER_SETTINGS sSchedSettings;  ///< Scheduler settings
    } __PACKED__ V1;                          ///< Version 1
  } __PACKED__ u;                             ///< Version controlled structs container
} __PACKED__ ECI_ISA_SETTINGS;


/**
  Definition of the USB hardware configuration parameters.
  
  @ingroup HwTypes
*/
typedef struct
{
  DWORD dwVer;                 ///< Version of valid union struct

  union
  {
    struct
    {
      DWORD dwReserved;        ///< Reserved for future use
    } __PACKED__ V0;           ///< Version 0
	struct
    {
      ECI_SCHEDULER_SETTINGS sRxSchedSettings;///< Scheduler settings for rx threads
      ECI_SCHEDULER_SETTINGS sTxSchedSettings;///< Scheduler settings for tx threads
    } __PACKED__ V1;                          ///< Version 1
  } __PACKED__ u;                             ///< Version controlled structs container
} __PACKED__ ECI_USB_SETTINGS;


/**
  Definition of the IP hardware configuration parameters.
  
  @ingroup HwTypes
*/
typedef struct
{
  DWORD dwVer;                  ///< Version of valid union struct

  union
  {
    struct
    {
      DWORD dwProtocol;         ///< Type of Protocol to use
      char  szIpAddress[0x100]; ///< IP Address, either IPv4, IPv6 oder DNS Name
      char  szPassword[0x100];  ///< Readable or encrypted password
      DWORD adwPort[16];        ///< Array of Port Numbers
    } __PACKED__ V0;                       ///< Version 0
	struct
    {
      DWORD dwProtocol;                       ///< Type of Protocol to use
      char  szIpAddress[0x100];               ///< IP Address, either IPv4, IPv6 oder DNS Name
      char  szPassword[0x100];                ///< Readable or encrypted password
      DWORD adwPort[16];                      ///< Array of Port Numbers
	  ECI_SCHEDULER_SETTINGS sSchedSettings;  ///< Scheduler settings
    } __PACKED__ V1;                          ///< Version 0
  } __PACKED__ u;                             ///< Version controlled structs container
} __PACKED__ ECI_IP_SETTINGS;


/**
  Definition of the hardware configuration parameters.
  
  @ingroup HwTypes
*/
typedef struct
{
  WORD  wHardwareClass;             ///< Hardware class @see e_HWCLASS
  DWORD dwFlags;                    ///< Flag field for special purpose @see e_SETTINGS_FLAGS

  union
  {
    ECI_PCI_SETTINGS sPciSettings;  ///< PCI hardware settings @see ECI_PCI_SETTINGS
    ECI_ISA_SETTINGS sIsaSettings;  ///< ISA hardware settings @see ECI_ISA_SETTINGS
    ECI_USB_SETTINGS sUsbSettings;  ///< USB hardware settings @see ECI_USB_SETTINGS
    ECI_IP_SETTINGS  sIpSettings;   ///< IP hardware settings @see ECI_IP_SETTINGS
  } __PACKED__ u;                              ///< Hardware configuration container
} __PACKED__ ECI_HW_PARA;


/**
  Controller information.
  
  @ingroup CtrlTypes
*/
typedef struct
{
  WORD wCtrlClass;         ///< Controller class @see e_CTRLCLASS
  WORD wCtrlState;         ///< Current controller state @see e_CTRLSTATES
} __PACKED__ ECI_CTRL_INFO;


/**
  Board information.
  
  @ingroup HwTypes
*/
typedef struct
{
  DWORD dwVer;                                    ///< Version of valid union struct

  union
  {
    struct
    {
      char  szHwBoardType[32];                    ///< Hardware board type string
      char  szHwSerial[16];                       ///< Hardware board serial number string
      BYTE  abHwVersion[4];                       ///< Hardware version (Branch, Major, Minor, Build)
      BYTE  abBmVersion[4];                       ///< Bootmanager version (Branch, Major, Minor, Build)
      char  szFwIdentification[64];               ///< Firmware identification string
      BYTE  abFwVersion[4];                       ///< Firmware version (Branch, Major, Minor, Build)
      DWORD adwApiVersion[4];                     ///< ECI API version (Branch, Major, Minor, Build)
      DWORD dwCtrlCount;                          ///< Number of supported controllers
      ECI_CTRL_INFO sCtrlInfo[ECI_MAXCTRLCOUNT];  ///< Controller info
    } __PACKED__ V0;                                         ///< Version 0
    struct
    {
      char  szHwBoardType[32];                    ///< Hardware board type string
      char  szHwSerial[16];                       ///< Hardware board serial number string
      BYTE  abHwVersion[4];                       ///< Hardware version (Branch, Major, Minor, Build)
      BYTE  abBmVersion[4];                       ///< Bootmanager version (Branch, Major, Minor, Build)
      char  szFwIdentification[64];               ///< Firmware identification string
      BYTE  abFwVersion[4];                       ///< Firmware version (Branch, Major, Minor, Build)
      DWORD adwApiVersion[4];                     ///< ECI API version (Branch, Major, Minor, Build)
      DWORD dwCtrlCount;                          ///< Number of supported controllers
      ECI_CTRL_INFO sCtrlInfo[ECI_MAXCTRLCOUNT];  ///< Controller info
      DWORD dwDevId;                              ///< Device ID
    } __PACKED__ V1;                                         ///< Version 0
  } __PACKED__ u;                                            ///< Version controlled structs container
} __PACKED__ ECI_HW_INFO;


/**
  Controller capabilities.
  
  @ingroup CtrlTypes
*/
typedef struct
{
  WORD wCtrlClass;                  ///< Controller class @see e_CTRLCLASS

  union
  {
    ECI_CANCAPABILITIES sCanCaps;   ///< CAN controller capability structure @see ECI_CANCAPABILITIES
    ECI_LINCAPABILITIES sLinCaps;   ///< LIN controller capability structure @see ECI_LINCAPABILITIES
  } __PACKED__ u;                   ///< Controller capability container 
} __PACKED__ ECI_CTRL_CAPABILITIES;


/**
  Current controller status.
  
  @ingroup CtrlTypes
*/
typedef struct
{
  WORD wCtrlClass;              ///< Controller class @see e_CTRLCLASS

  union
  {
    ECI_CANSTATUS sCanStatus;   ///< CAN controller capability structure @see ECI_CANSTATUS
    ECI_LINSTATUS sLinStatus;   ///< LIN controller capability structure @see ECI_LINSTATUS
  } __PACKED__ u;               ///< Controller status container 
} __PACKED__ ECI_CTRL_STATUS;


/**
  Controller configuration settings.
  
  @ingroup CtrlTypes
*/
typedef struct
{
  WORD wCtrlClass;              ///< Controller class @see e_CTRLCLASS

  union
  {
    ECI_CANINITLINE sCanConfig; ///< CAN controller configuration structure @see ECI_CANINITLINE
    ECI_LININITLINE sLinConfig; ///< LIN controller configuration structure @see ECI_LININITLINE
  } __PACKED__ u;                          ///< Controller configuration container
} __PACKED__ ECI_CTRL_CONFIG;


/**
  Controller message definitions
  
  @ingroup CtrlTypes
*/
typedef struct
{
  WORD wCtrlClass;              ///< Controller class @see e_CTRLCLASS

  union
  {
    ECI_CANMESSAGE sCanMessage; ///< CAN controller message structure @see ECI_CANMESSAGE
    ECI_LINMESSAGE sLinMessage; ///< LIN controller message structure @see ECI_LINMESSAGE
  } __PACKED__ u;                          ///< Controller message container
} __PACKED__ ECI_CTRL_MESSAGE;


/**
  Controller command request definitions.
  
  @ingroup CtrlTypes
*/
typedef struct
{
  WORD wCtrlClass;                    ///< Controller class @see e_CTRLCLASS

  union
  {
    ECI_CANCMDREQUEST sCanCmdRequest; ///< CAN controller command request structure @see ECI_CANCMDREQUEST
    ECI_LINCMDREQUEST sLinCmdRequest; ///< LIN controller command request structure @see ECI_LINCMDREQUEST
  } __PACKED__ u;                                ///< Controller command request container
} __PACKED__ ECI_CTRL_CMDREQUEST;


/**
  Controller command response definitions.
  
  @ingroup CtrlTypes
*/
typedef struct
{
  WORD wCtrlClass;                      ///< Controller class @see e_CTRLCLASS

  union
  {
    ECI_CANCMDRESPONSE sCanCmdResponse; ///< CAN controller command response structure @see ECI_CANCMDRESPONSE
    ECI_LINCMDRESPONSE sLinCmdResponse; ///< LIN controller command response structure @see ECI_LINCMDRESPONSE
  } __PACKED__ u;                                  ///< Controller command response container
} __PACKED__ ECI_CTRL_CMDRESPONSE;


/**
  Structure for filter definition.
  
  @ingroup CtrlTypes
*/

typedef struct
{
  WORD wCtrlClass;            ///< Controller class @see e_CTRLCLASS
  WORD wPadding;              ///< padding

  union
  {
    ECI_CANFILTER sCanFilter; ///< CAN controller message filter structure @see ECI_CANFILTER
    ECI_LINFILTER sLinFilter; ///< LIN controller message filter structure @see ECI_LINFILTER
  } __PACKED__ u;                        ///< Controller message filter container
} __PACKED__ ECI_CTRL_FILTER;


#include <ECI_poppack.h>

#endif //__ECI_HWTYPE_H__
