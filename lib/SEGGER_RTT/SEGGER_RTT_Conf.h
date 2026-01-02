/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*       Solutions for real time microcontroller applications         *
**********************************************************************
*                                                                    *
*            (c) 1995 - 2023 SEGGER Microcontroller GmbH             *
*                                                                    *
*       Internet: segger.com  Support: support_embos@segger.com      *
*                                                                    *
**********************************************************************
*                                                                    *
*       embOS * Real time operating system                           *
*                                                                    *
*       Please note: Knowledge of this file may under no             *
*       circumstances be used to write a similar product.            *
*       Thank you for your fairness !                                *
*                                                                    *
**********************************************************************
*                                                                    *
*       embOS version: V5.18.2.0                                     *
*                                                                    *
**********************************************************************

----------------------------------------------------------------------
File    : SEGGER_RTT_Conf.h
Purpose : Implementation of SEGGER real-time transfer (RTT) which
          allows real-time communication on targets which support
          debugger memory accesses while the CPU is running.
Revision: $Rev: 24316 $
----------------------------------------------------------------------
*/

#ifndef SEGGER_RTT_CONF_H
#define SEGGER_RTT_CONF_H

#ifdef __IAR_SYSTEMS_ICC__
  #include <intrinsics.h>
#endif

/*********************************************************************
*
*       Defines, configurable
*
**********************************************************************
*/

//
// Take in and set to correct values for Alarm system (system/project wide)
//
#ifndef   SEGGER_RTT_MAX_NUM_UP_BUFFERS
  #define SEGGER_RTT_MAX_NUM_UP_BUFFERS             (3)     // Max. number of up-buffers (T->H) available on this target    (Default: 3)
#endif

#ifndef   SEGGER_RTT_MAX_NUM_DOWN_BUFFERS
  #define SEGGER_RTT_MAX_NUM_DOWN_BUFFERS           (3)     // Max. number of down-buffers (H->T) available on this target  (Default: 3)
#endif

#ifndef   BUFFER_SIZE_UP
  #define BUFFER_SIZE_UP                            (1024)  // Size of the buffer for terminal output of target, up to host (Default: 1k)
#endif

#ifndef   BUFFER_SIZE_DOWN
  #define BUFFER_SIZE_DOWN                          (16)    // Size of the buffer for terminal input to target from host (Default: 16)
#endif

#ifndef   SEGGER_RTT_PRINTF_BUFFER_SIZE
  #define SEGGER_RTT_PRINTF_BUFFER_SIZE             (64u)   // Size of buffer for RTT printf to bulk-send chars via RTT     (Default: 64)
#endif

#ifndef   SEGGER_RTT_MODE_DEFAULT
  #define SEGGER_RTT_MODE_DEFAULT                   SEGGER_RTT_MODE_NO_BLOCK_SKIP // Mode for pre-initialized terminal channel (buffer 0)
#endif

/*********************************************************************
*
*       RTT memcpy configuration
*
*       memcpy() is good for large amounts of data,
*       but the RTT implementation is faster for small amounts.
*/
#ifndef   SEGGER_RTT_MEMCPY_USE_BYTELOOP
  #define SEGGER_RTT_MEMCPY_USE_BYTELOOP            0       // 0: Use memcpy/SEGGER_RTT_MEMCPY, 1: Use simple byte-loop
#endif

#ifndef   SEGGER_RTT_MEMCPY
  #include <string.h>
  #define SEGGER_RTT_MEMCPY(pDest, pSrc, NumBytes)  memcpy((pDest), (pSrc), (NumBytes))
#endif

/*********************************************************************
*
*       RTT lock configuration
*/
#ifndef   SEGGER_RTT_LOCK
  #define SEGGER_RTT_LOCK()
#endif

#ifndef   SEGGER_RTT_UNLOCK
  #define SEGGER_RTT_UNLOCK()
#endif

/*********************************************************************
*
*       RTT control block configuration
*/
#ifndef   SEGGER_RTT_ALIGNMENT
  #define SEGGER_RTT_ALIGNMENT                      0
#endif

#ifndef   SEGGER_RTT_BUFFER_ALIGNMENT
  #define SEGGER_RTT_BUFFER_ALIGNMENT               0
#endif

#ifndef   SEGGER_RTT_IN_RAM
  #define SEGGER_RTT_IN_RAM                         0
#endif

#ifndef   SEGGER_RTT_SECTION
  #if (defined __GNUC__)
    #define SEGGER_RTT_PUT_BUFFER_SECTION(Decl)     Decl __attribute__ ((section (".rtt_buffer")))
    #define SEGGER_RTT_PUT_CB_SECTION(Decl)         Decl __attribute__ ((section (".rtt_cb")))
  #else
    #define SEGGER_RTT_PUT_BUFFER_SECTION(Decl)     Decl
    #define SEGGER_RTT_PUT_CB_SECTION(Decl)         Decl
  #endif
#endif

#endif
