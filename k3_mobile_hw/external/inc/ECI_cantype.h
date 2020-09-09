///////////////////////////////////////////////////////////////////////////////
// (C) 2008-2011 IXXAT Automation GmbH, all rights reserved
///////////////////////////////////////////////////////////////////////////////
/**
  Definition of types, structs and unions for CAN initialization and
  communication.

  @file ECI_cantype.h
*/


#ifndef __ECI_CANTYPE_H__
#define __ECI_CANTYPE_H__


//////////////////////////////////////////////////////////////////////////
// include files
#include <OsEci.h>

#include <ECI_pshpack1.h>

//////////////////////////////////////////////////////////////////////////
// constants and macros

/*****************************************************************************
* CAN baud rates (bt0/bt1 for SJA1000 CAN controller with 16Mhz)
* (used in ECI_STRUCT_VERSION_V0 of ECI_CANINITLINE)
****************************************************************************/
#define ECI_CAN_BT0_5KB            0x3F   ///<  BT0    5 KB @ingroup CanTypes
#define ECI_CAN_BT1_5KB            0x7F   ///<  BT1    5 KB @ingroup CanTypes
#define ECI_CAN_BT0_10KB           0x31   ///<  BT0   10 KB @ingroup CanTypes
#define ECI_CAN_BT1_10KB           0x1C   ///<  BT1   10 KB @ingroup CanTypes
#define ECI_CAN_BT0_20KB           0x18   ///<  BT0   20 KB @ingroup CanTypes
#define ECI_CAN_BT1_20KB           0x1C   ///<  BT1   20 KB @ingroup CanTypes
#define ECI_CAN_BT0_50KB           0x09   ///<  BT0   50 KB @ingroup CanTypes
#define ECI_CAN_BT1_50KB           0x1C   ///<  BT1   50 KB @ingroup CanTypes
#define ECI_CAN_BT0_100KB          0x04   ///<  BT0  100 KB @ingroup CanTypes
#define ECI_CAN_BT1_100KB          0x1C   ///<  BT1  100 KB @ingroup CanTypes
#define ECI_CAN_BT0_125KB          0x03   ///<  BT0  125 KB @ingroup CanTypes
#define ECI_CAN_BT1_125KB          0x1C   ///<  BT1  125 KB @ingroup CanTypes
#define ECI_CAN_BT0_250KB          0x01   ///<  BT0  250 KB @ingroup CanTypes
#define ECI_CAN_BT1_250KB          0x1C   ///<  BT1  250 KB @ingroup CanTypes
#define ECI_CAN_BT0_500KB          0x00   ///<  BT0  500 KB @ingroup CanTypes
#define ECI_CAN_BT1_500KB          0x1C   ///<  BT1  500 KB @ingroup CanTypes
#define ECI_CAN_BT0_800KB          0x00   ///<  BT0  800 KB @ingroup CanTypes
#define ECI_CAN_BT1_800KB          0x16   ///<  BT1  800 KB @ingroup CanTypes
#define ECI_CAN_BT0_1000KB         0x00   ///<  BT0 1000 KB @ingroup CanTypes
#define ECI_CAN_BT1_1000KB         0x14   ///<  BT1 1000 KB @ingroup CanTypes

#define ECI_CAN_BT01_5KB           0x3F,0x7F   ///<    5 KB @ingroup CanTypes
#define ECI_CAN_BT01_10KB          0x31,0x1C   ///<   10 KB @ingroup CanTypes
#define ECI_CAN_BT01_20KB          0x18,0x1C   ///<   20 KB @ingroup CanTypes
#define ECI_CAN_BT01_50KB          0x09,0x1C   ///<   50 KB @ingroup CanTypes
#define ECI_CAN_BT01_100KB         0x04,0x1C   ///<  100 KB @ingroup CanTypes
#define ECI_CAN_BT01_125KB         0x03,0x1C   ///<  125 KB @ingroup CanTypes
#define ECI_CAN_BT01_250KB         0x01,0x1C   ///<  250 KB @ingroup CanTypes
#define ECI_CAN_BT01_500KB         0x00,0x1C   ///<  500 KB @ingroup CanTypes
#define ECI_CAN_BT01_800KB         0x00,0x16   ///<  800 KB @ingroup CanTypes
#define ECI_CAN_BT01_1000KB        0x00,0x14   ///< 1000 KB @ingroup CanTypes

/**
  retrieve value of baud rate prescaler (BRP)

  @ingroup CanTypes
*/
#define ECI_CAN_BT0_GET_BRP(bt0) ((bt0 & 0x3F) + 1)

/**
  retrieve value of synchronization jump width (SJW)

  @ingroup CanTypes
*/
#define ECI_CAN_BT0_GET_SJW(bt0) (((bt0 & 0xC0) >> 6) + 1)

/**
  retrieve value of time segment 1 (TSEG1)

  @ingroup CanTypes
*/
#define ECI_CAN_BT1_GET_TS1(bt1) ((bt1 & 0x0F) + 1)

/**
  retrieve value of time segment2 (TSEG2)

  @ingroup CanTypes
*/
#define ECI_CAN_BT1_GET_TS2(bt1) (((bt1 & 0x70) >> 4) + 1)

/**
  retrieve value of sampling mode (SAM)

  @ingroup CanTypes
*/
#define ECI_CAN_BT1_GET_SAM(bt1) ((bt1 & 0x80) >> 8)


/**
  determine time quanta per bit
  
  @ingroup CanTypes
*/
#define ECI_CAN_BT1_GET_TQS(bt1) \
 ( 1 + ECI_CAN_BT1_GET_TS1(bt1) + ECI_CAN_BT1_GET_TS2(bt1) )

/**
  determine bit rate
  
  @ingroup CanTypes
*/
#define ECI_CAN_BT01_GET_BITRATE(bt0, bt1) \
  ( 16000000 / (2 * ECI_CAN_BT1_GET_TQS(bt1) * ECI_CAN_BT0_GET_BRP(bt0)) )

/*****************************************************************************
* CAN baud rates
* (without SYNC segment as used in ECI_STRUCT_VERSION_V1 of ECI_CANINITLINE)
****************************************************************************/
#define V1_ECI_CAN_BTP_5KB         {0,     5000, 16,  8,  1,  0}   ///<    10 KB, SP 68,0% @ingroup CanTypes
#define V1_ECI_CAN_BTP_10KB        {0,    10000, 13,  2,  1,  0}   ///<    10 KB, SP 87,5% @ingroup CanTypes
#define V1_ECI_CAN_BTP_20KB        {0,    20000, 13,  2,  1,  0}   ///<    20 KB, SP 87,5% @ingroup CanTypes
#define V1_ECI_CAN_BTP_50KB        {0,    50000, 13,  2,  1,  0}   ///<    50 KB, SP 87,5% @ingroup CanTypes
#define V1_ECI_CAN_BTP_100KB       {0,   100000, 13,  2,  1,  0}   ///<   100 KB, SP 87,5% @ingroup CanTypes
#define V1_ECI_CAN_BTP_125KB       {0,   125000, 13,  2,  1,  0}   ///<   125 KB, SP 87,5% @ingroup CanTypes
#define V1_ECI_CAN_BTP_250KB       {0,   250000, 13,  2,  1,  0}   ///<   250 KB, SP 87,5% @ingroup CanTypes
#define V1_ECI_CAN_BTP_500KB       {0,   500000, 13,  2,  1,  0}   ///<   500 KB, SP 87,5% @ingroup CanTypes
#define V1_ECI_CAN_BTP_800KB       {0,   800000,  7,  2,  1,  0}   ///<   800 KB, SP 80,0% @ingroup CanTypes
#define V1_ECI_CAN_BTP_1000KB      {0,  1000000,  5,  2,  1,  0}   ///<  1000 KB, SP 75,0% @ingroup CanTypes

/*****************************************************************************
* CAN FD baud rates, used for arbitration and fast data rate
* (without SYNC segment as used in ECI_STRUCT_VERSION_V1 of ECI_CANINITLINE)
****************************************************************************/
#define V1_ECI_CAN_SDR_BTP_500KB   {0,   500000, 63, 16,  16, 0}   ///<   500 KB, SP 80,0% @ingroup CanTypes
#define V1_ECI_CAN_SDR_BTP_1000KB  {0,  1000000, 31,  8,   8, 0}   ///<  1000 KB, SP 80,0% @ingroup CanTypes

#define V1_ECI_CAN_FDR_BTP_2000KB  {0,  2000000, 15,  4,  4, 32}   ///<  2000 KB, SP 80,0% @ingroup CanTypes
#define V1_ECI_CAN_FDR_BTP_4000KB  {0,  4000000,  7,  2,  2, 16}   ///<  4000 KB, SP 80,0% @ingroup CanTypes
#define V1_ECI_CAN_FDR_BTP_5000KB  {0,  5000000,  5,  2,  2, 12}   ///<  5000 KB, SP 75,0% @ingroup CanTypes
#define V1_ECI_CAN_FDR_BTP_6666KB  {0,  6666666,  3,  2,  2,  8}   ///<  6666 KB, SP 66,7% @ingroup CanTypes
#define V1_ECI_CAN_FDR_BTP_8000KB  {0,  8000000,  3,  1,  1,  5}   ///<  8000 KB, SP 80,0% @ingroup CanTypes
#define V1_ECI_CAN_FDR_BTP_10000KB {0, 10000000,  2,  1,  1,  4}   ///< 10000 KB, SP 75,0% @ingroup CanTypes

/*****************************************************************************
* CAN baud rates
* (with SYNC segment as used in ECI_STRUCT_VERSION_V2 of ECI_CANINITLINE)
****************************************************************************/

//------------------------------------------------------------------------
// predefined CiA bit rates
//------------------------------------------------------------------------

//
// controller indepenent bit timing parameters
//
// -----------------+-----------+-------+-------+-------+-------+--------
// Bitrate [kBit/s] | BPS [1/s] |  TS1  |  TS2  |  SJW  |  TDO  | SP [%]
// -----------------+-----------+-------+-------+-------+-------+--------
//      5                 5000      17      8       1       0     68,0
//     10                10000      14      2       1       0     87,5
//     20                20000      14      2       1       0     87,5
//     50                50000      14      2       1       0     87,5
//    100               100000      14      2       1       0     87,5
//    125               125000      14      2       1       0     87,5
//    250               250000      14      2       1       0     87,5
//    500               500000      14      2       1       0     87,5
//    800               800000       8      2       1       0     80,0
//   1000              1000000       6      2       1       0     75,0
//
#define V2_ECI_CAN_BTP_EMPTY  {0,      0, 0,0,0,0}  ///< Empty definition
#define V2_ECI_CAN_BTP_5KB    {0,   5000,17,8,1,0}  ///<    5 KB, SP 68,0% @ingroup CanTypes
#define V2_ECI_CAN_BTP_10KB   {0,  10000,14,2,1,0}  ///<   10 KB, SP 87,5% @ingroup CanTypes
#define V2_ECI_CAN_BTP_20KB   {0,  20000,14,2,1,0}  ///<   20 KB, SP 87,5% @ingroup CanTypes
#define V2_ECI_CAN_BTP_50KB   {0,  50000,14,2,1,0}  ///<   50 KB, SP 87,5% @ingroup CanTypes
#define V2_ECI_CAN_BTP_100KB  {0, 100000,14,2,1,0}  ///<  100 KB, SP 87,5% @ingroup CanTypes
#define V2_ECI_CAN_BTP_125KB  {0, 125000,14,2,1,0}  ///<  125 KB, SP 87,5% @ingroup CanTypes
#define V2_ECI_CAN_BTP_250KB  {0, 250000,14,2,1,0}  ///<  250 KB, SP 87,5% @ingroup CanTypes
#define V2_ECI_CAN_BTP_500KB  {0, 500000,14,2,1,0}  ///<  500 KB, SP 87,5% @ingroup CanTypes
#define V2_ECI_CAN_BTP_800KB  {0, 800000, 8,2,1,0}  ///<  800 KB, SP 80,0% @ingroup CanTypes
#define V2_ECI_CAN_BTP_1000KB {0,1000000, 6,2,1,0}  ///< 1000 KB, SP 75,0% @ingroup CanTypes

//------------------------------------------------------------------------
// predefined controller independent CAN FD bit timing parameters
// for short lines
//------------------------------------------------------------------------

//
// Arbitration bitrate
//
// -----------------+-----------+-------+-------+-------+-------+--------
// Bitrate [kBit/s] | BPS [1/s] |  TS1  |  TS2  |  SJW  |  TDO  | SP [%]
// -----------------+-----------+-------+-------+-------+-------+--------
//    500               500000    6400    1600    1600      0     80,0
//   1000              1000000    6400    1600    1600      0     80,0
//

#define ECI_CAN_BTP_ABR_SL_500KB  {0, 500000, 6400, 1600, 1600, 0} ///<  500 KB, SP 80,0% @ingroup CanTypes
#define ECI_CAN_BTP_ABR_SL_1000KB {0,1000000, 6400, 1600, 1600, 0} ///< 1000 KB, SP 80,0% @ingroup CanTypes

//
// Data bitrates
//
// -----------------+-----------+-------+-------+-------+-------+--------
// Bitrate [kBit/s] | BPS [1/s] |  TS1  |  TS2  |  SJW  |  TDO  | SP [%]
// -----------------+-----------+-------+-------+-------+-------+--------
//   1000              1000000    1600     400     400    1600    80,0
//   2000              2000000    1600     400     400    1600    80,0
//   4000              4000000     800     200     200     800    80,0
//   5000              5000000     600     200     200     600    75,0
//   6667              6666666     400     200     200     402    66,7
//   8000              8000000     400     100     100     250    80,0
//  10000             10000000     300     100     100     200    75,0
//

#define ECI_CAN_BTP_DBR_SL_1000KB  {0, 1000000, 1600, 400, 400, 1600} ///<  1000 KB, SP 80,0% @ingroup CanTypes
#define ECI_CAN_BTP_DBR_SL_2000KB  {0, 2000000, 1600, 400, 400, 1600} ///<  2000 KB, SP 80,0% @ingroup CanTypes
#define ECI_CAN_BTP_DBR_SL_4000KB  {0, 4000000,  800, 200, 200,  800} ///<  4000 KB, SP 80,0% @ingroup CanTypes
#define ECI_CAN_BTP_DBR_SL_5000KB  {0, 5000000,  600, 200, 200,  600} ///<  5000 KB, SP 75,0% @ingroup CanTypes
#define ECI_CAN_BTP_DBR_SL_6667KB  {0, 6666666,  400, 200, 200,  402} ///<  6667 KB, SP 66,0% @ingroup CanTypes
#define ECI_CAN_BTP_DBR_SL_8000KB  {0, 8000000,  400, 100, 100,  250} ///<  8000 KB, SP 80,0% @ingroup CanTypes
#define ECI_CAN_BTP_DBR_SL_10000KB {0,10000000,  300, 100, 100,  200} ///< 10000 KB, SP 75,0% @ingroup CanTypes

//-------------------------------------------------------------------
// predefined controller independent CAN FD bit timing parameters
// for long lines
//-------------------------------------------------------------------

//
// Arbitration bitrate:
//
// -----------------+-----------+-------+-------+-------+-------+--------
// Bitrate [kBit/s] | BPS [1/s] |  TS1  |  TS2  |  SJW  |  TDO  | SP [%]
// -----------------+-----------+-------+-------+-------+-------+--------
//    250               250000    6400    1600    1600      0     80,0
//

#define ECI_CAN_BTP_ABR_LL_250KB {0, 250000, 6400, 1600, 1600, 0} ///<  250 KB, SP 80,0% @ingroup CanTypes

//
// Data bitrates (see table below):
//
// -----------------+-----------+-------+-------+-------+-------+--------
// Bitrate [kBit/s] | BPS [1/s] |  TS1  |  TS2  |  SJW  |  TDO  | SP [%]
// -----------------+-----------+-------+-------+-------+-------+--------
//    500               500000    6400    1600    1600    6400    80,0
//    833               833333    1600     400     400    1620    80,0
//   1000              1000000    1600     400     400    1600    80,0
//   1538              1538461    1000     300     300    1040    76,9
//   2000              2000000     800     200     200     770    80,0
//   4000              4000000     800     200     200     800    80,0
//

#define ECI_CAN_BTP_DBR_LL_500KB  {0,  500000, 6400, 1600, 1600, 6400} ///<  500 KB, SP 80,0% @ingroup CanTypes
#define ECI_CAN_BTP_DBR_LL_833KB  {0,  833333, 1600,  400,  400, 1620} ///<  833 KB, SP 80,0% @ingroup CanTypes
#define ECI_CAN_BTP_DBR_LL_1000KB {0, 1000000, 1600,  400,  400, 1600} ///< 1000 KB, SP 80,0% @ingroup CanTypes
#define ECI_CAN_BTP_DBR_LL_1538KB {0, 1538461, 1000,  300,  300, 1040} ///< 1538 KB, SP 76,0% @ingroup CanTypes
#define ECI_CAN_BTP_DBR_LL_2000KB {0, 2000000,  800,  200,  200,  770} ///< 2000 KB, SP 80,0% @ingroup CanTypes
#define ECI_CAN_BTP_DBR_LL_4000KB {0, 4000000,  800,  200,  200,  800} ///< 4000 KB, SP 80,0% @ingroup CanTypes

//
// set ECI_USE_V1_CAN_FD_RATES if you want to use the 
// bitrate constants compatible to ECI_STRUCT_VERSION_V1 of ECI_CANINITLINE.
// When using them be sure to set dwVer attribute of ECI_CANINITLINE to ECI_STRUCT_VERSION_V1.
//
#ifdef ECI_USE_V1_CAN_FD_RATES

#define ECI_CAN_BTP_5KB           V1_ECI_CAN_BTP_5KB  
#define ECI_CAN_BTP_10KB          V1_ECI_CAN_BTP_10KB  
#define ECI_CAN_BTP_20KB          V1_ECI_CAN_BTP_20KB  
#define ECI_CAN_BTP_50KB          V1_ECI_CAN_BTP_50KB  
#define ECI_CAN_BTP_100KB         V1_ECI_CAN_BTP_100KB 
#define ECI_CAN_BTP_125KB         V1_ECI_CAN_BTP_125KB 
#define ECI_CAN_BTP_250KB         V1_ECI_CAN_BTP_250KB 
#define ECI_CAN_BTP_500KB         V1_ECI_CAN_BTP_500KB 
#define ECI_CAN_BTP_800KB         V1_ECI_CAN_BTP_800KB 
#define ECI_CAN_BTP_1000KB        V1_ECI_CAN_BTP_1000KB

/*****************************************************************************
* CAN FD baud rates, used for arbitration and fast data rate
* (without SYNC segment as used in ECI_STRUCT_VERSION_V1 of ECI_CANINITLINE)
****************************************************************************/
#define ECI_CAN_SDR_BTP_500KB     V1_ECI_CAN_SDR_BTP_500KB  
#define ECI_CAN_SDR_BTP_1000KB    V1_ECI_CAN_SDR_BTP_1000KB 

#define ECI_CAN_FDR_BTP_2000KB    V1_ECI_CAN_FDR_BTP_2000KB 
#define ECI_CAN_FDR_BTP_4000KB    V1_ECI_CAN_FDR_BTP_4000KB 
#define ECI_CAN_FDR_BTP_5000KB    V1_ECI_CAN_FDR_BTP_5000KB 
#define ECI_CAN_FDR_BTP_6666KB    V1_ECI_CAN_FDR_BTP_6666KB 
#define ECI_CAN_FDR_BTP_8000KB    V1_ECI_CAN_FDR_BTP_8000KB 
#define ECI_CAN_FDR_BTP_10000KB   V1_ECI_CAN_FDR_BTP_10000KB

#else

//
// Although the following defines would be valid placeholders,
// we do not define them to fail the source lines in applications which
// uses the V1 struct of ECI_CANINITLINE.
// Use either 
// #define ECI_USE_V1_CAN_FD_RATES
// before including ECI.h or switch to the V2 struct.
//
/*

#define ECI_CAN_BTP_5KB           V2_ECI_CAN_BTP_5KB  
#define ECI_CAN_BTP_10KB          V2_ECI_CAN_BTP_10KB  
#define ECI_CAN_BTP_20KB          V2_ECI_CAN_BTP_20KB  
#define ECI_CAN_BTP_50KB          V2_ECI_CAN_BTP_50KB  
#define ECI_CAN_BTP_100KB         V2_ECI_CAN_BTP_100KB 
#define ECI_CAN_BTP_125KB         V2_ECI_CAN_BTP_125KB 
#define ECI_CAN_BTP_250KB         V2_ECI_CAN_BTP_250KB 
#define ECI_CAN_BTP_500KB         V2_ECI_CAN_BTP_500KB 
#define ECI_CAN_BTP_800KB         V2_ECI_CAN_BTP_800KB 
#define ECI_CAN_BTP_1000KB        V2_ECI_CAN_BTP_1000KB

#define ECI_CAN_SDR_BTP_500KB     ECI_CAN_BTP_ABR_SL_500KB
#define ECI_CAN_SDR_BTP_1000KB    ECI_CAN_BTP_ABR_SL_1000KB

#define ECI_CAN_FDR_BTP_2000KB    ECI_CAN_BTP_DBR_SL_2000KB
#define ECI_CAN_FDR_BTP_4000KB    ECI_CAN_BTP_DBR_SL_4000KB
#define ECI_CAN_FDR_BTP_5000KB    ECI_CAN_BTP_DBR_SL_5000KB
#define ECI_CAN_FDR_BTP_6666KB    ECI_CAN_BTP_DBR_SL_6667KB
#define ECI_CAN_FDR_BTP_8000KB    ECI_CAN_BTP_DBR_SL_8000KB
#define ECI_CAN_FDR_BTP_10000KB   ECI_CAN_BTP_DBR_SL_10000KB

*/

#endif



/** Maximum possible 11bit CAN ID @ingroup CanTypes */
#define ECI_CAN_MAX_11BIT_ID 0x7FF

/** Maximum possible 29bit CAN ID @ingroup CanTypes */
#define ECI_CAN_MAX_29BIT_ID 0x1FFFFFFF

/**
  CAN controller types

  @ingroup CanTypes
*/
typedef enum
{
  ECI_CAN_CTRL_UNKNOWN = 0x00,         ///< unknown
  ECI_CAN_CTRL_82527   = 0x01,         ///< Intel 82527
  ECI_CAN_CTRL_82C200  = 0x02,         ///< Intel 82C200
  ECI_CAN_CTRL_81C90   = 0x03,         ///< Intel 81C90
  ECI_CAN_CTRL_81C92   = 0x04,         ///< Intel 81C92
  ECI_CAN_CTRL_SJA1000 = 0x05,         ///< Philips SJA 1000
  ECI_CAN_CTRL_82C900  = 0x06,         ///< Infineon 82C900 (TwinCAN)
  ECI_CAN_CTRL_TOUCAN  = 0x07,         ///< Motorola TOUCAN
  ECI_CAN_CTRL_MSCAN   = 0x08,         ///< Freescale Star12 MSCAN
  ECI_CAN_CTRL_FLEXCAN = 0x09,         ///< Freescale Coldfire FLEXCAN
  ECI_CAN_CTRL_IFI     = 0x0A,         ///< IFI CAN (ALTERA FPGA CAN)
  ECI_CAN_CTRL_CCAN    = 0x0B,         ///< CCAN (Bosch C_CAN)
  ECI_CAN_CTRL_BXCAN   = 0x0C,         ///< BXCAN (ST BX_CAN)
  ECI_CAN_CTRL_IFIFD   = 0x0D,         ///< IFI CAN FD (ALTERA FPGA CAN FD)
  ECI_CAN_CTRL_MAXVAL  = 0xFF          ///< Maximum value for controller type
} e_CANCTRLCLASS;


/**
  CAN controller supported features.
  Bit coded information, resulting value can be any combination of values below.

  @ingroup CanTypes
*/
typedef enum
{
  ECI_CAN_FEATURE_UNDEFINED = 0x00000,  ///< undefined
  ECI_CAN_FEATURE_STDOREXT  = 0x00001,  ///< 11 OR 29 bit (exclusive)
  ECI_CAN_FEATURE_STDANDEXT = 0x00002,  ///< 11 AND 29 bit (simultaneous)
  ECI_CAN_FEATURE_RMTFRAME  = 0x00004,  ///< Reception of remote frames
  ECI_CAN_FEATURE_ERRFRAME  = 0x00008,  ///< Reception of error frames
  ECI_CAN_FEATURE_BUSLOAD   = 0x00010,  ///< Bus load measurement
  ECI_CAN_FEATURE_IDFILTER  = 0x00020,  ///< Exact message filter
  ECI_CAN_FEATURE_LISTONLY  = 0x00040,  ///< Listen only mode
  ECI_CAN_FEATURE_SCHEDULER = 0x00080,  ///< Cyclic message scheduler
  ECI_CAN_FEATURE_GENERRFRM = 0x00100,  ///< Error frame generation
  ECI_CAN_FEATURE_DELAYEDTX = 0x00200,  ///< Delayed message transmission
  ECI_CAN_FEATURE_SSM       = 0x00400,  ///< Single shot mode
  ECI_CAN_FEATURE_HI_PRIO   = 0x00800,  ///< High priority message
  ECI_CAN_FEATURE_EXTDATA   = 0x01000,  ///< Extended data length respectively
                                        ///< improved frame format (CAN FD - EDL/IFF)
  ECI_CAN_FEATURE_FASTDATA  = 0x02000,  ///< Fast data rate (CAN FD - BRS)
  ECI_CAN_FEATURE_STT       = 0x04000,  ///< single transmission try messages with acknowledge error
  ECI_CAN_FEATURE_ISOFD     = 0x08000,  ///< ISO CAN FD
  ECI_CAN_FEATURE_NONISOFD  = 0x10000,  ///< non-ISO CAN FD
} e_CANCTRLFEATURE;


/**
  CAN controller bus coupling types.
  Bit coded information, resulting value can be any combination of values below.

  @ingroup CanTypes
*/
typedef enum
{
  ECI_CAN_BUSC_UNDEFINED = 0x0000,     ///< undefined
  ECI_CAN_BUSC_LOWSPEED  = 0x0001,     ///< Low speed coupling
  ECI_CAN_BUSC_HIGHSPEED = 0x0002      ///< High speed coupling
} e_CANBUSC ;


/**
  CAN controller operating modes.
  Bit coded information, resulting value can be any combination of values below.

  @ingroup CanTypes
*/
typedef enum
{
  ECI_CAN_OPMODE_UNDEFINED     = 0x00,     ///< undefined
  ECI_CAN_OPMODE_STANDARD      = 0x01,     ///< Reception of 11-bit id messages
  ECI_CAN_OPMODE_EXTENDED      = 0x02,     ///< Reception of 29-bit id messages
  ECI_CAN_OPMODE_ERRFRAME      = 0x04,     ///< Enable reception of error frames
  ECI_CAN_OPMODE_LISTONLY      = 0x08,     ///< Listen only mode (TX passive)
  ECI_CAN_OPMODE_LOWSPEED      = 0x10,     ///< Use low speed bus interface
  ECI_CAN_OPMODE_STT           = 0x20,     ///< @deprecated enables single transmission try
                                           ///< messages with acknowledge error
  ECI_CAN_OPMODE_CACHED_STATUS = 0x40,     ///< The status will be cached within the driver
                                           ///< which leads to very fast ECIDRV_CtrlGetStatus
                                           ///< calls. However you may need to set a status
                                           ///< update rate with ECIDRV_CtrlSetStatusUpdateRate.
} e_CANOPMODE;


/**
  CAN controller extended operating modes.
  Bit coded information, resulting value can be any combination of values below.

  @ingroup CanTypes
*/
typedef enum
{
  ECI_CAN_EXMODE_DISABLED  = 0x00,     ///< No extended operation
  ECI_CAN_EXMODE_EXTDATA   = 0x01,     ///< Enable extended data length respectively
                                       ///< improved frame format (CAN FD - EDL/IFF)
  ECI_CAN_EXMODE_FASTDATA  = 0x02,     ///< Enable fast data bit rate (CAN FD - BRS)
  ECI_CAN_EXMODE_ISOFD     = 0x04      ///< Enable ISO CAN FD mode otherwise non-ISO CAN FD is enabled
} e_CANEXMODE;


/**
  CAN controller bit-timing modes modes.
  Bit coded information, resulting value can be any combination of values below.

  @ingroup CanTypes
*/
typedef enum
{
  ECI_CAN_BTMODE_UNDEFINED = 0x00000000,     ///< undefined
  ECI_CAN_BTMODE_NATIVE    = 0x00000001,     ///< Enable native mode setting.
  ECI_CAN_BTMODE_TRISAMPLE = 0x00000002      ///< Enable triple sampling mode.
} e_CANBTMODE;


/**
  CAN message types (used by <CANMSGINFO.Bytes.bType>)

  @ingroup CanTypes
*/
typedef enum
{
  ECI_CAN_MSGTYPE_DATA    = 0,         ///< Data frame
  ECI_CAN_MSGTYPE_INFO    = 1,         ///< Info frame
  ECI_CAN_MSGTYPE_ERROR   = 2,         ///< Error frame
  ECI_CAN_MSGTYPE_STATUS  = 3,         ///< Status frame
  ECI_CAN_MSGTYPE_WAKEUP  = 4,         ///< Wakeup frame
  ECI_CAN_MSGTYPE_TIMEOVR = 5,         ///< Timer overrun
  ECI_CAN_MSGTYPE_TIMERST = 6          ///< Timer reset
} e_CANMSGTYPE;


/**
  CAN message information flags (used by <CANMSGINFO.Bytes.bFlags>).
  Bit coded information, resulting value can be any combination of values below.

  @ingroup CanTypes
*/
typedef enum
{
  ECI_CAN_MSGFLAGS_DLC = 0x0F,         ///< Data length code
  ECI_CAN_MSGFLAGS_OVR = 0x10,         ///< Data overrun flag
  ECI_CAN_MSGFLAGS_SRR = 0x20,         ///< Self reception request
  ECI_CAN_MSGFLAGS_RTR = 0x40,         ///< Remote transmission request
  ECI_CAN_MSGFLAGS_EXT = 0x80          ///< Frame format (0=11-bit, 1=29-bit)
} e_CANMSGFLAGS;


/**
  CAN message information flags2 (used by <CANMSGINFO.Bytes.bFlags2>).
  Bit coded information, resulting value can be any combination of values below.

  @ingroup CanTypes
*/
typedef enum
{
  ECI_CAN_MSGFLAGS_SSM = 0x01,         ///< Single shot message. @copydetails ECI_CANMSGINFO::ssm
  ECI_CAN_MSGFLAGS_HPM = 0x06,         ///< High priority message channel 0-3. @copydetails ECI_CANMSGINFO::hpm
  ECI_CAN_MSGFLAGS_EDL = 0x08,         ///< Extended data length respectively improved frame format (CAN FD - EDL/IFF).
                                       ///< @copydetails ECI_CANMSGINFO::edl
  ECI_CAN_MSGFLAGS_BRS = 0x10,         ///< Fast data rate, bit rate switch (CAN FD - BRS). @copydetails ECI_CANMSGINFO::brs
  ECI_CAN_MSGFLAGS_ESI = 0x20          ///< Error state indicator (CAN FD - ESI). @copydetails ECI_CANMSGINFO::esi
} e_CANMSGFLAGS2;


/**
  Information supplied in the abData[0] field of info frames
  (CANMSGINFO.Bytes.bType = CAN_MSGTYPE_INFO).

  @ingroup CanTypes
*/
typedef enum
{
  ECI_CAN_INFO_START = 1,              ///< Start of CAN controller
  ECI_CAN_INFO_STOP  = 2,              ///< Stop of CAN controller
  ECI_CAN_INFO_RESET = 3               ///< Reset of CAN controller
} e_CANINFO;


/**
  Error information supplied in the abData[0] field of error frames
  (CANMSGINFO.Bytes.bType = CAN_MSGTYPE_ERROR).

  @ingroup CanTypes
*/
typedef enum
{
  ECI_CAN_ERROR_UNDEFINED = 0,             ///< Unknown or no error
  ECI_CAN_ERROR_STUFF     = 1,             ///< Stuff error
  ECI_CAN_ERROR_FORM      = 2,             ///< Form error
  ECI_CAN_ERROR_ACK       = 3,             ///< Acknowledgment error
  ECI_CAN_ERROR_BIT       = 4,             ///< Bit error
  ECI_CAN_ERROR_FAST_DATA = 5,             ///< Fast data bit error (CAN FD)
  ECI_CAN_ERROR_CRC       = 6,             ///< CRC error
  ECI_CAN_ERROR_OTHER     = 7              ///< Other (unspecified) error
} e_CANERROR;


/**
  Status information supplied in the abData[0] field of status frames
  (CANMSGINFO.Bytes.bType = ECI_CAN_MSGTYPE_STATUS) and in
  ECI_CANSTATUS::u::V0::dwStatus.
  Bit coded information, resulting value can be any combination of values below.

  @ingroup CanTypes
*/
typedef enum
{
  ECI_CAN_STATUS_TXPEND  = 0x01,        ///< Transmission pending
  ECI_CAN_STATUS_OVRRUN  = 0x02,        ///< Data overrun occurred
  ECI_CAN_STATUS_ERRLIM  = 0x04,        ///< Error warning limit exceeded
  ECI_CAN_STATUS_BUSOFF  = 0x08,        ///< Bus off status
  ECI_CAN_STATUS_ININIT  = 0x10,        ///< Init mode active
  ECI_CAN_STATUS_BUSCERR = 0x20,        ///< Bus coupling error
  ECI_CAN_STATUS_ACKERR  = 0x40         ///< acknowledge error
} e_CANSTATUS;


/**
  Information on which tx fifo should be cleared

  @ingroup CanClearTxFifo
*/
typedef enum
{
  ECI_CAN_CLRDEV_REMPEND_BIT   = 0x01, ///< Remove the actual pending message
  ECI_CAN_CLRDEV_CTRL_BIT      = 0x02, ///< Clear the controller fifo
  ECI_CAN_CLRDEV_FIFO_BIT      = 0x04, ///< Clear the hardware fifo
  ECI_CAN_CLRDEV_COMM_BIT      = 0x08, ///< Clear the communication buffers
  ECI_CAN_CLRDRV_FIFO_BIT      = 0x10, ///< Clear the software fifo
  ECI_CAN_CLRSRV_FIFO_BIT      = 0x20, ///< Clear the software fifo
  ECI_CAN_CLRALL               = 0xFF  ///< Clear all
} e_CANCLEARTX;

/**
  Filter selection whether you want to set a software filter where
  all traffic is handled by the driver or, if supported, you want to 
  set the hardware filter directly on the interface.

  @ingroup CanTypes
*/
typedef enum
{
  ECI_CAN_SW_FILTER_STD = 0x00,         ///< select standard software filter (11-bit)
  ECI_CAN_SW_FILTER_EXT = 0x01,         ///< select extended software filter (29-bit)
  ECI_CAN_HW_FILTER_STD = 0x02,         ///< select standard hardware filter (11-bit)
  ECI_CAN_HW_FILTER_EXT = 0x03          ///< select extended hardware filter (29-bit)
} e_CANFILTER;

#define CAN_FILTER_ACC_CODE_ALL  0x00000000     ///< filter id to receive all messages
#define CAN_FILTER_ACC_MASK_ALL  0x00000000     ///< filter mask to receive all messages

#define CAN_FILTER_ACC_CODE_NONE 0xFFFFFFFF     ///< filter id to filter all messages
#define CAN_FILTER_ACC_MASK_NONE 0x80000000     ///< filter mask to filter all messages

#define ECI_CAN_MAX_11BIT_FILTERID ((ECI_CAN_MAX_11BIT_ID << 1) | 0x1) ///< maximal 11-bit filter id
#define ECI_CAN_MAX_29BIT_FILTERID ((ECI_CAN_MAX_29BIT_ID << 1) | 0x1) ///< maximal 29-bit filter id


//////////////////////////////////////////////////////////////////////////
// data types

/**
  CAN controller bit timing parameter

   ECI_STRUCT_VERSION_V1:
     ECI specific API. Similar API as in VCI4 but with different handling of the SYNC
     segment for non native bitrates (ECI_CAN_BTMODE_NATIVE not set).
     TSEG1 does *not* include the SYNC segment which is always 1 TQ.

    @code
      Computation of nominal bit time Tbit
     
      |<------------ Tbit ------------>|
      +------+------------+------------+
      | SYNC | Time Seg 1 | Time Seg 2 |
      +------+------------+------------+
                          |
                          +-> Sample Point
     
      Fs    := Frequency of the CAN system clock see also: ECI_CANCAPABILITIES
      Ts    := duration of a CAN system clock tick
      Tq    := duration of a time quanta
      Fbit  := bit rate in bits per second
      Tbit  := duration of a bit
      Tsync := duration of Sync Segment (always 1)
      Tseg1 := duration of Time Segment 1
      Tseg2 := duration of Time Segment 2
      Qbit  := number of time quanta for one bit
      Qsync := number of time quanta for Sync Segment (always 1)
      Qseg1 := number of time quanta for Time Segment 1
      Qseg2 := number of time quanta for Time Segment 2
      BRP   := baud rate pre-scaler
     
      Ts = 1 / Fs
      Tq = Ts * BRP = BRP / Fs
      Tq = Tbit / Qbit
     
      Tbit = 1 / Fbit
     
      Qbit = Qsync + Qseg1 + Qseg2
     
      Tsync = Tq * 1
      Tseg1 = Tq * Qseg1
      Tseg2 = Tq * Qseg2
     
      Tbit = Tsync + Tseg1 + Tseg2
      Tbit = Tq * (1 + Qseg1 + Qseg2)
      Tbit = Ts * BRP * (1 + Qseg1 + Qseg2)

    @endcode

   ECI_STRUCT_VERSION_V2:
     Same API as in VCI4
     TSEG1 does include SYNC segment.
     Advantages:
      The position of the sample point can be determined without knowing the controller clock rate.
      Compatibility to VCI4.

    @code
      
      *** normal mode *******************************
      
       <------- Tbit --------->
       +------+-------+-------+
       | SYNC | TSEG1 | TSEG2 |
       +------+---- --+-------+
       |     wTS1     | wTS2  |
       +--------------+-------+
                      |
                      +-> Sample Point
      
       computation of sample point
      
                           wTS1
       SP [%] = 100 * ---------------
                       (wTS1 + wTS2)
      
      *** raw mode **********************************
      
       <-------- Tbit -------->
       +------+-------+-------+
       | SYNC | TSEG1 | TSEG2 |
       +------+-------+-------+
       |  1   |  wTS1 |  wTS2 |
       +------+-------+-------+
                      |
                      +-> Sample Point
      
       computation of sample point
      
                          (1 + wTS1)
       SP [%] = 100 * -------------------
                       (1 + wTS1 + wTS2)
      
       SYNC  := Re-Synchronization Segment
       TSEG1 := Time Segment 1
       TSEG2 := Time Segment 2

    @endcode

  @ingroup CanTypes
*/
typedef struct
{
  DWORD dwMode;         ///< Timing mode  @see e_CANBTMODE constants
  DWORD dwBPS;          ///< Bits per second or prescaler @see e_CANBTMODE.
                        ///< In native mode this parameter contains the controller specific
                        ///< value for the baud rate pre-scaler.
  WORD  wTS1;           ///< Duration of time segment 1 in quantas
  WORD  wTS2;           ///< Duration of time segment 2 in quantas
  WORD  wSJW;           ///< Re-synchronisation jump width in quantas
  WORD  wTDO;           ///< Transceiver delay compensation offset in quantas
                        ///< Used to specify the sample point
                        ///< during bit monitoring in fast data bit rate.
                        ///< (0 = disabled, 0xFFFF = simplified SSP positioning)
                        ///< Using 0xFFFF the TDO value will be calculated according to
                        ///< the simplified SSP positioning (see chapter 7.5.2 of
                        ///< the CiA 601 Standard Part 3 System Design Recommendation)
} __PACKED__ ECI_CANBTP;


/**
  CAN controller configuration.

   Bitrate configuration options has evolved and finally have been
   adjusted to be compatible to VCI4. Evolution in three steps:
   ECI_STRUCT_VERSION_V0:
     Virtual SJA1000 registers (bt0/bt1) simulating an SJA1000 at 16MHz.
     Same API as in VCI1, VCI2 and VCI3
   ECI_STRUCT_VERSION_V1:
     ECI specific API. Similar API as in VCI4 but with different handling of the SYNC
     segment for non native bitrates (ECI_CAN_BTMODE_NATIVE not set).
     TSEG1 does *not* include the SYNC segment which is always 1 TQ.
   ECI_STRUCT_VERSION_V2:
     Same API as in VCI4
     TSEG1 does include SYNC segment.
     Advantages:
      The position of the sample point can be determined without knowing the controller clock rate.
      Compatibility to VCI4.

  @ingroup CanTypes
*/
typedef struct
{
  DWORD dwVer;                ///< Version of valid union struct

  union
  {
    struct
    {
      BYTE       bOpMode;     ///< CAN operating mode @see e_CANOPMODE
      BYTE       bReserved;   ///< Reserved set to 0
      BYTE       bBtReg0;     ///< Bus timing register 0 according to SJA1000/16MHz
      BYTE       bBtReg1;     ///< Bus timing register 1 according to SJA1000/16MHz
    } __PACKED__ V0;                     ///< Version 0
    struct
    {
      BYTE       bOpMode;     ///< CAN operating mode @see e_CANOPMODE
      BYTE       bExMode;     ///< Extended operation mode @see e_CANEXMODE
      ECI_CANBTP sBtpSdr;     ///< Standard / arbitration bit rate timing
      ECI_CANBTP sBtpFdr;     ///< Fast data bit rate timing
    } __PACKED__ V1;                     ///< Version 1
    struct
    {
      BYTE       bOpMode;     ///< CAN operating mode @see e_CANOPMODE
      BYTE       bExMode;     ///< Extended operation mode @see e_CANEXMODE
      ECI_CANBTP sBtpSdr;     ///< Standard / arbitration bit rate timing
      ECI_CANBTP sBtpFdr;     ///< Fast data bit rate timing
    } __PACKED__ V2;                     ///< Version 2
  } __PACKED__ u;                        ///< Version controlled structs container
} __PACKED__ ECI_CANINITLINE;


/**
  CAN controller capabilities.

  @ingroup CanTypes
*/
typedef struct
{
  DWORD dwVer;                      ///< Version of valid union struct

  union
  {
    struct
    {
      WORD        wCanType;         ///< Type of CAN controller @see e_CANCTRLCLASS
      WORD        wBusCoupling;     ///< Type of Bus coupling @see e_CANBUSC
      DWORD       dwFeatures;       ///< Supported features @see e_CANCTRLFEATURE
      DWORD       dwClockFreq;      ///< Clock frequency of the primary counter in Hz
      DWORD       dwTscDivisor;     ///< Divisor for the message time stamp counter
      DWORD       dwDtxDivisor;     ///< Divisor for the delayed message transmitter
      DWORD       dwDtxMaxTicks;    ///< Maximum tick count value of the delayed message transmitter
      DWORD       dwNoOfPrioQueues; ///< Number of priority TX queues
    } __PACKED__ V0;                           ///< Version 0
    struct
    {
      WORD        wCanType;         ///< Type of CAN controller @see e_CANCTRLCLASS
      WORD        wBusCoupling;     ///< Type of Bus coupling @see e_CANBUSC
      DWORD       dwFeatures;       ///< Supported features @see e_CANCTRLFEATURE
      DWORD       dwPriClkFreq;     ///< Clock frequency of the primary counter in Hz
      DWORD       dwTscDivisor;     ///< Divisor for the message time stamp counter
      DWORD       dwDtxDivisor;     ///< Divisor for the delayed message transmitter
      DWORD       dwDtxMaxTicks;    ///< Maximum tick count value of the delayed message transmitter
      DWORD       dwNoOfPrioQueues; ///< Number of priority TX queues
      DWORD       dwCanClkFreq;     ///< CAN clock frequency in Hz (16/2 MHz for SJA1000)
      ECI_CANBTP  sSdrRangeMin;     ///< Minimum bit timing values for standard bit rate
      ECI_CANBTP  sSdrRangeMax;     ///< Maximum bit timing values for standard bit rate
      ECI_CANBTP  sFdrRangeMin;     ///< Minimum bit timing values for fast data bit rate
      ECI_CANBTP  sFdrRangeMax;     ///< Maximum bit timing values for fast data bit rate
    } __PACKED__ V1;                           ///< Version 1
  } __PACKED__ u;                              ///< Version controlled structs container
} __PACKED__ ECI_CANCAPABILITIES;


/**
  CAN controller status.

  @ingroup CanTypes
*/
typedef struct
{
  DWORD dwVer;                      ///< Version of valid union struct

  union
  {
    struct
    {
      BYTE        bOpMode;          ///< Current CAN operating mode @see e_CANOPMODE
      BYTE        bBusLoad;         ///< Average bus load in percent (0..100)
      BYTE        bBtReg0;          ///< Current bus timing register 0 value according to SJA1000/16MHz
      BYTE        bBtReg1;          ///< Current bus timing register 1 value according to SJA1000/16MHz
      DWORD       dwStatus;         ///< Status of the CAN controller @see e_CANSTATUS
    } __PACKED__ V0;                           ///< Version 0
    struct
    {
      BYTE        bOpMode;          ///< Current CAN operating mode @see e_CANOPMODE
      BYTE        bBusLoad;         ///< Average bus load in percent (0..100)
      BYTE        bExMode;          ///< Extended operation mode @see e_CANEXMODE
      BYTE        bReserved;        ///< Reserved set to 0
      DWORD       dwStatus;         ///< Status of the CAN controller @see e_CANSTATUS
      ECI_CANBTP  sBtpSdr;          ///< Standard / arbitration bit rate timing
      ECI_CANBTP  sBtpFdr;          ///< Fast data bit rate timing
    } __PACKED__ V1;                           ///< Version 1
    struct
    {
      BYTE        bOpMode;          ///< Current CAN operating mode @see e_CANOPMODE
      BYTE        bBusLoad;         ///< Average bus load in percent (0..100)
      BYTE        bExMode;          ///< Extended operation mode @see e_CANEXMODE
      BYTE        bReserved;        ///< Reserved set to 0
      DWORD       dwStatus;         ///< Status of the CAN controller @see e_CANSTATUS
      ECI_CANBTP  sBtpSdr;          ///< Standard / arbitration bit rate timing (VCI4 compatible)
      ECI_CANBTP  sBtpFdr;          ///< Fast data bit rate timing (VCI4 compatible)
    } __PACKED__ V2;                           ///< Version 1
  } __PACKED__ u;                              ///< Version controlled structs container
} __PACKED__ ECI_CANSTATUS;


/**
  CAN message information.

  @ingroup CanTypes
*/
typedef union
{
  struct
  {
    BYTE  bType;          ///< Message type @see e_CANMSGTYPE
    BYTE  bFlags2;        ///< Flags @see e_CANMSGFLAGS2
    BYTE  bFlags;         ///< Flags @see e_CANMSGFLAGS
    BYTE  bAccept;        ///< Acceptance filter code
  } __PACKED__  Bytes;                ///< CAN Message information in byte format

  struct
  {
    DWORD type: 8;        ///< Message type @see e_CANMSGTYPE
    DWORD ssm : 1;        ///< Single shot message (TX direction only).
                          ///< If set it is tried to send the message only once. If
                          ///< transmission or bus arbitration fails the message is discarded.
    DWORD hpm : 2;        ///< High priority message channel 0-3 (TX direction only).
                          ///< Selects the priority of the message where 0 is the lowest
                          ///< and "normal" message channel and 3 is the highest priority
                          ///< message channel. The number of supported priority queues
                          ///< is available in @ref ECI_CANCAPABILITIES::dwNoOfPrioQueues.
    DWORD edl : 1;        ///< Extended data length respectively improved frame format (CAN FD - EDL/IFF)
                          ///< If set the dlc field is interpreted according to CAN FD
                          ///< specification. This means up to 64 data bytes are supported.
    DWORD brs : 1;        ///< Fast data rate, bit rate switch (CAN FD - BRS)
                          ///< If set the data field is send or was received with a higher
                          ///< data baud rate. Therefore the CAN controller has to be configured
                          ///< with two independant baud rates.
    DWORD esi : 1;        ///< Error state indicator (CAN FD - ESI) (RX direction only).
                          ///< Indicates if a sending CAN FD node is in error passive level.
    DWORD res : 2;        ///< Reserved for future use
    DWORD dlc : 4;        ///< Data length code
    DWORD ovr : 1;        ///< Possible data overrun
    DWORD srr : 1;        ///< Self reception request
    DWORD rtr : 1;        ///< Remote transmission request. This flag may not be combined
                          ///< with edl
    DWORD ext : 1;        ///< Extended frame format (0=standard, 1=extended)
    DWORD afc : 8;        ///< Acceptance filter code
  } __PACKED__  Bits;                 ///< CAN Message information in bit format
} __PACKED__  ECI_CANMSGINFO;


/**
  CAN message structure.

  @ingroup CanTypes
*/
typedef struct
{
  DWORD dwVer;                    ///< Version of valid union struct

  union
  {
    struct
    {
      DWORD           dwTime;     ///< Time stamp for receive message in [us]
      DWORD           dwMsgId;    ///< CAN message identifier (INTEL format)
      ECI_CANMSGINFO  uMsgInfo;   ///< CAN message information (bit field)
      BYTE            abData[8];  ///< Message data
    } __PACKED__ V0;                         ///< Version 0
    struct
    {
      DWORD           dwTime;     ///< Time stamp for receive message in [us]
      DWORD           dwMsgId;    ///< CAN message identifier (INTEL format)
      ECI_CANMSGINFO  uMsgInfo;   ///< CAN message information (bit field)
      BYTE            abData[64]; ///< Message data
    } __PACKED__ V1;                         ///< Version 1
  } __PACKED__ u;                            ///< Version controlled structs container
} __PACKED__ ECI_CANMESSAGE;


/**
  CAN command structure.

  @ingroup CanTypes
*/
typedef struct
{
  WORD  wCode;                    ///< Command request code
} __PACKED__  ECI_CAN_CMD_REQ_HD;


/**
  CAN Command structure.

  @ingroup CanTypes
*/
typedef struct
{
  WORD  wResult;                  ///< Command result code
} __PACKED__  ECI_CAN_CMD_RES_HD;


/**
  CAN command structure.

  @ingroup CanTypes
*/
typedef struct
{
  DWORD                   dwVer;      ///< Version of valid union struct

  union
  {
    struct
    {
      ECI_CAN_CMD_REQ_HD  sCmdHeader; ///< CAN command header @see ECI_CAN_CMD_REQ_HD
      DWORD               dwReserved; ///< reserved for future use
    } __PACKED__  V0;                 ///< Version 0
  } __PACKED__  u;                    ///< Version controlled structs container
} __PACKED__  ECI_CANCMDREQUEST;


/**
  CAN command structure.

  @ingroup CanTypes
*/
typedef struct
{
  DWORD                   dwVer;      ///< Version of valid union struct

  union
  {
    struct
    {
      ECI_CAN_CMD_RES_HD  sCmdHeader; ///< CAN command header @see ECI_CAN_CMD_RES_HD
      DWORD               dwReserved; ///< reserved for future use
    } __PACKED__  V0;                 ///< Version 0
  } __PACKED__  u;                    ///< Version controlled structs container
} __PACKED__  ECI_CANCMDRESPONSE;


/**
  CAN filter structure.

  @ingroup CanTypes
*/
typedef struct
{
  DWORD dwVer;                    ///< Version of valid union struct

  union
  {
    struct
    {
      DWORD dwIsExtended;         ///< specifies whether to choose extended or standard id
      DWORD dwCode;               ///< id code for filter settings
      DWORD dwMask;               ///< mask code for filter settings
    } __PACKED__  V0;             ///< Version 0
  } __PACKED__  u;                ///< Version controlled structs container
} __PACKED__  ECI_CANFILTER ;

#include <ECI_poppack.h>

#endif
