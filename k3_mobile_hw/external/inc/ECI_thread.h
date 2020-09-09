///////////////////////////////////////////////////////////////////////////////
// (C) 2008-2011 IXXAT Automation GmbH, all rights reserved
///////////////////////////////////////////////////////////////////////////////
/**
  Definition of types, structs and unions for thread initialization.

  @file ECI_thread.h
*/


#ifndef __ECI_THREAD_H__
#define __ECI_THREAD_H__

#include <OsEci.h>
#include <ECI_pshpack1.h>

#define ECI_CPU_ALL    0xFFFFFFFF            ///< Allow all CPUs
#define ECI_MAX_CPU    (sizeof(DWORD) * 8)   ///< Max CPU number
#define ECI_CPU(x)     (1 << x)              ///< Set CPU number

/**
  Scheduler types

  @ingroup SchedulerTypes
*/
typedef enum
{
  ECI_SCHED_NOCHANGE    = 0x00,       ///< keep actual scheduler
  ECI_SCHED_FIFO        = 0x01,       ///< fifo scheduling
  ECI_SCHED_RR          = 0x02,       ///< round robin scheduling
  ECI_SCHED_OTHER       = 0x03        ///< other scheduling
} e_SCHEDPOLICY;

/**
  Scheduler setting

  @ingroup SchedulerTypes
*/
typedef struct
{
    DWORD dwSchedulerPolicy;       ///< Scheduler policy class @see e_SCHEDPOLICY
    DWORD dwSchedulerPriority;     ///< Scheduler thread priority
    DWORD dwCpuAffinity;           ///< CPU affinity of the thread @see e_CPUAFFINITY
}__PACKED__ ECI_SCHEDULER_SETTINGS,*PECI_SCHEDULER_SETTINGS;
/** @typedef PECI_SCHEDULER_SETTINGS
  @brief @copybrief ECI_SCHEDULER_SETTINGS @n <b> See structure:</b> @ref ECI_SCHEDULER_SETTINGS */

#include <ECI_poppack.h>

#endif // __ECI_THREAD_H__
