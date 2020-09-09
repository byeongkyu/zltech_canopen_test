///////////////////////////////////////////////////////////////////////////////
// (C) 2008-2011 IXXAT Automation GmbH, all rights reserved
///////////////////////////////////////////////////////////////////////////////
/**
  Definition of error codes used by the ECI.

  @file ECI_error.h
*/

#ifndef __ECI_ERROR_H__
#define __ECI_ERROR_H__

//////////////////////////////////////////////////////////////////////////
// include files
#include <OsEci.h>

//////////////////////////////////////////////////////////////////////////
// constants and macros

/** Maximal length of an error string */
#define TEST_MAX_ERRSTR       256

/** Facility code for the ECI error codes */
#define FACILITY_IXXAT_ECI    0x00FE0000

/** Severity code success */
#define SEV_SUCCESS           0x00000000
/** Severity code info */
#define SEV_INFO              0x40000000
/** Severity code warning */
#define SEV_WARN              0x80000000
/** Severity code error */
#define SEV_ERROR             0xC0000000

/** Customer flag */
#define CUSTOMER_FLAG         0x20000000
/** Reserved flag */
#define RESERVED_FLAG         0x10000000

/** IXXAT defined ECI info code */
#define SEV_ECI_INFO          (SEV_INFO  | CUSTOMER_FLAG | FACILITY_IXXAT_ECI)
/** IXXAT defined ECI warning code */
#define SEV_ECI_WARN          (SEV_WARN  | CUSTOMER_FLAG | FACILITY_IXXAT_ECI)
/** IXXAT defined ECI error code */
#define SEV_ECI_ERROR         (SEV_ERROR | CUSTOMER_FLAG | FACILITY_IXXAT_ECI)

/** Mask to determine the facility code */
#define FACILITY_MASK         0x0FFF0000
/** Mask to determine the status resp. error code */
#define STATUS_MASK           0x0000FFFF

/** Data type for ECI_RESULT */
#define ECI_RESULT              HRESULT

//////////////////////////////////////////////////////////////////////////
// data types

/**
  List of ECI error codes.
*/
typedef enum
{
  ECI_OK = (0),                                                   ///< Operation finished successfully.
  ECI_ERR_FIRST                     = (SEV_ECI_ERROR | 0x0000),
  ECI_ERR_INVALIDARG                = (SEV_ECI_ERROR | 0x0001),   ///< One or more arguments are invalid.
  ECI_ERR_FAILED                    = (SEV_ECI_ERROR | 0x0002),   ///< Failed with unknown error.
  ECI_ERR_NOTIMPL                   = (SEV_ECI_ERROR | 0x0003),   ///< Function is not implemented.
  ECI_ERR_NOT_SUPPORTED             = (SEV_ECI_ERROR | 0x0004),   ///< Type or parameter is not supported.
  ECI_ERR_ACCESS_DENIED             = (SEV_ECI_ERROR | 0x0005),   ///< Access denied.
  ECI_ERR_RESOURCE_BUSY             = (SEV_ECI_ERROR | 0x0006),   ///< The device or resource is busy.
  ECI_ERROR_OUTOFMEMORY             = (SEV_ECI_ERROR | 0x0007),   ///< Ran out of memory
  ECI_ERR_INVALID_HANDLE            = (SEV_ECI_ERROR | 0x0008),   ///< The handle is invalid.
  ECI_ERR_INVALID_POINTER           = (SEV_ECI_ERROR | 0x0009),   ///< The pointer is invalid.
  ECI_ERR_INVALID_DATATYPE          = (SEV_ECI_ERROR | 0x000A),   ///< The data type is invalid.
  ECI_ERR_INVALID_DATA              = (SEV_ECI_ERROR | 0x000B),   ///< The data is invalid.
  ECI_ERR_TIMEOUT                   = (SEV_ECI_ERROR | 0x000C),   ///< This operation returned because the timeout period expired.
  ECI_ERR_INSUFFICIENT_RESOURCES    = (SEV_ECI_ERROR | 0x000D),   ///< The available resources are insufficient to perform this request.
  ECI_ERR_RESOURCE_NOT_FOUND        = (SEV_ECI_ERROR | 0x000E),   ///< The device or resource could not be found.
  ECI_ERR_REQUEST_FAILED            = (SEV_ECI_ERROR | 0x000F),   ///< The request failed.
  ECI_ERR_INVALID_RESPONSE          = (SEV_ECI_ERROR | 0x0010),   ///< The received response is invalid.
  ECI_ERR_UNKNOWN_RESPONSE          = (SEV_ECI_ERROR | 0x0011),   ///< The received response is unknown.
  ECI_ERR_BAD_COMMAND               = (SEV_ECI_ERROR | 0x0012),   ///< The device does not recognize the command.
  ECI_ERR_NO_MORE_DATA              = (SEV_ECI_ERROR | 0x0013),   ///< No more data is available.
  ECI_ERR_NO_MORE_SPACE_LEFT        = (SEV_ECI_ERROR | 0x0014),   ///< No more space left to perform this request.
  ECI_ERR_UNSUPPORTED_VERSION       = (SEV_ECI_ERROR | 0x0015),   ///< The available or requested version is unsupported.
  //ECI specific
  ECI_ERR_INVALID_CTRLHANDLE        = (SEV_ECI_ERROR | 0x0016),   ///< The specified ECI board or controller handle is invalid.
  ECI_ERR_WRONG_STATE               = (SEV_ECI_ERROR | 0x0017),   ///< ECI is in wrong state to perform this request.
  ECI_ERR_CREATE_FAILED             = (SEV_ECI_ERROR | 0x0018),   ///< ECI interface could not be instantiated correctly.
  ECI_ERR_IRQTEST_FAILED            = (SEV_ECI_ERROR | 0x0019),   ///< IRQ handler could not be installed correctly.
  ECI_ERR_FWDOWNLOAD                = (SEV_ECI_ERROR | 0x001A),   ///< An error occurred while downloading or verifying firmware
  ECI_ERR_DPRAM_LOCK_VIOLATION      = (SEV_ECI_ERROR | 0x001B),   ///< A lock violation occurred while communication via DPRAM interface.
  ECI_ERR_DRPAM_IO_ERROR            = (SEV_ECI_ERROR | 0x001C),   ///< An I/O error occurred while reading from or writing to DRPAM interface
  ECI_ERR_UNSUPPORTED_FWVERSION     = (SEV_ECI_ERROR | 0x001D),   ///< The boot manager or firmware version is unsupported.
  ECI_ERR_USB_INCOMPLETE_DESCRIPTOR = (SEV_ECI_ERROR | 0x001E),   ///< The USB device descriptor is invalid.
  ECI_ERR_FLASH_WRITE               = (SEV_ECI_ERROR | 0x001F),   ///< The system cannot write to the specified flash memory.
  ECI_ERR_FLASH_READ                = (SEV_ECI_ERROR | 0x0020),   ///< The system cannot read from the specified flash memory.
  ECI_ERR_FLASH_ERASE               = (SEV_ECI_ERROR | 0x0021),   ///< The requested delete operation could not be performed on the flash memory.
  ECI_ERR_FLASH_VERIFY              = (SEV_ECI_ERROR | 0x0022),   ///< The requested verify operation failed.
  ECI_ERROR_SUCCESS_REBOOT_REQUIRED = (SEV_ECI_ERROR | 0x0023),   ///< The requested operation finishes successfully. Changes will not be effective until the system is rebooted.
  ECI_ERR_LAST
} e_ECIERROR;

#endif
