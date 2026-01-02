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

----------------------------------------------------------------------
File    : SEGGER_RTT.c
Purpose : Implementation of SEGGER real-time transfer (RTT) which
          allows real-time communication on targets which support
          debugger memory accesses while the CPU is running.
Revision: $Rev: 25842 $
----------------------------------------------------------------------
*/

#include "SEGGER_RTT.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

/*********************************************************************
*
*       Defines, fixed
*
**********************************************************************
*/
#define MIN(a, b)         (((a) < (b)) ? (a) : (b))
#define MAX(a, b)         (((a) > (b)) ? (a) : (b))

#define SEGGER_RTT_MODE_MASK                  (3u)

/*********************************************************************
*
*       Static data
*
**********************************************************************
*/

//
// RTT Control Block and target buffers
//
static char _acUpBuffer  [BUFFER_SIZE_UP];
static char _acDownBuffer[BUFFER_SIZE_DOWN];

//
// RTT Control Block placed in specified section
//
SEGGER_RTT_PUT_CB_SECTION(SEGGER_RTT_CB _SEGGER_RTT);

static unsigned char _IsInitialized = 0;

/*********************************************************************
*
*       Static functions
*
**********************************************************************
*/

/*********************************************************************
*
*       _DoInit()
*
*  Function description
*    Initializes the RTT Control Block
*/
static void _DoInit(void) {
  SEGGER_RTT_CB* p;

  p = &_SEGGER_RTT;
  memset(p, 0, sizeof(*p));
  p->MaxNumUpBuffers = SEGGER_RTT_MAX_NUM_UP_BUFFERS;
  p->MaxNumDownBuffers = SEGGER_RTT_MAX_NUM_DOWN_BUFFERS;
  //
  // Initialize up buffer 0
  //
  p->aUp[0].sName = "Terminal";
  p->aUp[0].pBuffer = _acUpBuffer;
  p->aUp[0].SizeOfBuffer = sizeof(_acUpBuffer);
  p->aUp[0].RdOff = 0u;
  p->aUp[0].WrOff = 0u;
  p->aUp[0].Flags = SEGGER_RTT_MODE_DEFAULT;
  //
  // Initialize down buffer 0
  //
  p->aDown[0].sName = "Terminal";
  p->aDown[0].pBuffer = _acDownBuffer;
  p->aDown[0].SizeOfBuffer = sizeof(_acDownBuffer);
  p->aDown[0].RdOff = 0u;
  p->aDown[0].WrOff = 0u;
  p->aDown[0].Flags = SEGGER_RTT_MODE_DEFAULT;
  //
  // Finish initialization of the RTT Control Block.
  // ID is set last to indicate valid block.
  //
  strcpy(p->acID, "SEGGER RTT");
  _IsInitialized = 1;
}

/*********************************************************************
*
*       _Init()
*
*  Function description
*    Initializes RTT if not already done.
*/
static void _Init(void) {
  if (_IsInitialized == 0) {
    _DoInit();
  }
}

/*********************************************************************
*
*       _GetAvailableSpace()
*
*  Function description
*    Returns the available space in the ring buffer.
*/
static unsigned _GetAvailableSpace(SEGGER_RTT_BUFFER_UP* pRing) {
  unsigned RdOff;
  unsigned WrOff;
  unsigned r;

  RdOff = pRing->RdOff;
  WrOff = pRing->WrOff;
  if (RdOff <= WrOff) {
    r = pRing->SizeOfBuffer - 1u - WrOff + RdOff;
  } else {
    r = RdOff - WrOff - 1u;
  }
  return r;
}

/*********************************************************************
*
*       Public functions
*
**********************************************************************
*/

/*********************************************************************
*
*       SEGGER_RTT_Init()
*
*  Function description
*    Initializes the RTT Control Block.
*/
void SEGGER_RTT_Init(void) {
  _DoInit();
}

/*********************************************************************
*
*       SEGGER_RTT_Write()
*
*  Function description
*    Stores data in RTT buffer. Handles wrap-around.
*
*  Return value
*    Number of bytes stored.
*/
unsigned SEGGER_RTT_Write(unsigned BufferIndex, const void* pBuffer, unsigned NumBytes) {
  unsigned NumBytesToWrite;
  unsigned NumBytesWritten;
  unsigned RdOff;
  unsigned WrOff;
  unsigned char* pDst;
  SEGGER_RTT_BUFFER_UP* pRing;
  volatile SEGGER_RTT_CB* pRTTCB;

  _Init();
  pRTTCB = &_SEGGER_RTT;
  //
  // Get handle to ring buffer
  //
  pRing = (SEGGER_RTT_BUFFER_UP*)((char*)&pRTTCB->aUp[BufferIndex]);
  //
  // Get write position and calculate available space
  //
  RdOff = pRing->RdOff;
  WrOff = pRing->WrOff;
  NumBytesToWrite = _GetAvailableSpace(pRing);
  //
  // Limit write length to available space
  //
  if (NumBytesToWrite < NumBytes) {
    if ((pRing->Flags & SEGGER_RTT_MODE_MASK) == SEGGER_RTT_MODE_NO_BLOCK_SKIP) {
      NumBytes = 0;  // Skip entire write
    } else {
      NumBytes = NumBytesToWrite;  // Trim to available space
    }
  }
  //
  // Write data to ring buffer
  //
  pDst = (unsigned char*)pRing->pBuffer + WrOff;
  NumBytesWritten = NumBytes;
  if (WrOff + NumBytes > pRing->SizeOfBuffer) {
    //
    // Write wraps around - write in two parts
    //
    unsigned NumBytesBeforeWrap = pRing->SizeOfBuffer - WrOff;
    memcpy(pDst, pBuffer, NumBytesBeforeWrap);
    memcpy(pRing->pBuffer, (const unsigned char*)pBuffer + NumBytesBeforeWrap, NumBytes - NumBytesBeforeWrap);
  } else {
    memcpy(pDst, pBuffer, NumBytes);
  }
  //
  // Update write offset
  //
  WrOff += NumBytes;
  if (WrOff >= pRing->SizeOfBuffer) {
    WrOff -= pRing->SizeOfBuffer;
  }
  pRing->WrOff = WrOff;

  return NumBytesWritten;
}

/*********************************************************************
*
*       SEGGER_RTT_WriteNoLock()
*/
unsigned SEGGER_RTT_WriteNoLock(unsigned BufferIndex, const void* pBuffer, unsigned NumBytes) {
  return SEGGER_RTT_Write(BufferIndex, pBuffer, NumBytes);
}

/*********************************************************************
*
*       SEGGER_RTT_WriteSkipNoLock()
*/
unsigned SEGGER_RTT_WriteSkipNoLock(unsigned BufferIndex, const void* pBuffer, unsigned NumBytes) {
  return SEGGER_RTT_Write(BufferIndex, pBuffer, NumBytes);
}

/*********************************************************************
*
*       SEGGER_RTT_WriteString()
*
*  Function description
*    Stores a null-terminated string in RTT buffer.
*
*  Return value
*    Number of bytes stored (excluding terminating null).
*/
unsigned SEGGER_RTT_WriteString(unsigned BufferIndex, const char* s) {
  return SEGGER_RTT_Write(BufferIndex, s, strlen(s));
}

/*********************************************************************
*
*       SEGGER_RTT_Read()
*
*  Function description
*    Reads characters from RTT buffer.
*
*  Return value
*    Number of characters read.
*/
unsigned SEGGER_RTT_Read(unsigned BufferIndex, void* pData, unsigned BufferSize) {
  unsigned NumBytesRem;
  unsigned NumBytesRead;
  unsigned RdOff;
  unsigned WrOff;
  unsigned char* pBuffer;
  SEGGER_RTT_BUFFER_DOWN* pRing;
  volatile SEGGER_RTT_CB* pRTTCB;

  _Init();
  pRTTCB = &_SEGGER_RTT;
  pRing = (SEGGER_RTT_BUFFER_DOWN*)((char*)&pRTTCB->aDown[BufferIndex]);
  pBuffer = (unsigned char*)pData;
  RdOff = pRing->RdOff;
  WrOff = pRing->WrOff;
  NumBytesRead = 0u;
  //
  // Calculate number of bytes available
  //
  if (RdOff > WrOff) {
    NumBytesRem = pRing->SizeOfBuffer - RdOff;
    NumBytesRem = MIN(NumBytesRem, BufferSize);
    memcpy(pBuffer, pRing->pBuffer + RdOff, NumBytesRem);
    NumBytesRead += NumBytesRem;
    pBuffer += NumBytesRem;
    BufferSize -= NumBytesRem;
    RdOff += NumBytesRem;
    if (RdOff >= pRing->SizeOfBuffer) {
      RdOff = 0u;
    }
  }
  //
  // Read remaining bytes
  //
  NumBytesRem = WrOff - RdOff;
  NumBytesRem = MIN(NumBytesRem, BufferSize);
  if (NumBytesRem > 0u) {
    memcpy(pBuffer, pRing->pBuffer + RdOff, NumBytesRem);
    NumBytesRead += NumBytesRem;
    RdOff += NumBytesRem;
  }
  pRing->RdOff = RdOff;

  return NumBytesRead;
}

/*********************************************************************
*
*       SEGGER_RTT_ReadNoLock()
*/
unsigned SEGGER_RTT_ReadNoLock(unsigned BufferIndex, void* pData, unsigned BufferSize) {
  return SEGGER_RTT_Read(BufferIndex, pData, BufferSize);
}

/*********************************************************************
*
*       SEGGER_RTT_HasData()
*
*  Function description
*    Returns the number of bytes available in the down buffer.
*/
unsigned SEGGER_RTT_HasData(unsigned BufferIndex) {
  unsigned RdOff;
  unsigned WrOff;
  unsigned NumBytes;
  volatile SEGGER_RTT_CB* pRTTCB;
  SEGGER_RTT_BUFFER_DOWN* pRing;

  _Init();
  pRTTCB = &_SEGGER_RTT;
  pRing = (SEGGER_RTT_BUFFER_DOWN*)((char*)&pRTTCB->aDown[BufferIndex]);
  RdOff = pRing->RdOff;
  WrOff = pRing->WrOff;
  if (WrOff >= RdOff) {
    NumBytes = WrOff - RdOff;
  } else {
    NumBytes = pRing->SizeOfBuffer - RdOff + WrOff;
  }
  return NumBytes;
}

/*********************************************************************
*
*       SEGGER_RTT_HasDataUp()
*/
unsigned SEGGER_RTT_HasDataUp(unsigned BufferIndex) {
  unsigned RdOff;
  unsigned WrOff;
  unsigned NumBytes;
  volatile SEGGER_RTT_CB* pRTTCB;
  SEGGER_RTT_BUFFER_UP* pRing;

  _Init();
  pRTTCB = &_SEGGER_RTT;
  pRing = (SEGGER_RTT_BUFFER_UP*)((char*)&pRTTCB->aUp[BufferIndex]);
  RdOff = pRing->RdOff;
  WrOff = pRing->WrOff;
  if (WrOff >= RdOff) {
    NumBytes = WrOff - RdOff;
  } else {
    NumBytes = pRing->SizeOfBuffer - RdOff + WrOff;
  }
  return NumBytes;
}

/*********************************************************************
*
*       SEGGER_RTT_HasKey()
*
*  Function description
*    Returns true if at least one character is available in buffer 0.
*/
int SEGGER_RTT_HasKey(void) {
  return (SEGGER_RTT_HasData(0) > 0) ? 1 : 0;
}

/*********************************************************************
*
*       SEGGER_RTT_GetKey()
*
*  Function description
*    Reads one character from buffer 0.
*
*  Return value
*    Character read, or -1 if no character available.
*/
int SEGGER_RTT_GetKey(void) {
  char c;
  int r;

  r = (int)SEGGER_RTT_Read(0u, &c, 1u);
  if (r == 1) {
    r = (int)(unsigned char)c;
  } else {
    r = -1;
  }
  return r;
}

/*********************************************************************
*
*       SEGGER_RTT_WaitKey()
*
*  Function description
*    Waits until at least one character is available, then reads it.
*/
int SEGGER_RTT_WaitKey(void) {
  int r;

  do {
    r = SEGGER_RTT_GetKey();
  } while (r < 0);
  return r;
}

/*********************************************************************
*
*       SEGGER_RTT_printf()
*
*  Function description
*    Stores a formatted string in the specified RTT buffer.
*/
int SEGGER_RTT_printf(unsigned BufferIndex, const char* sFormat, ...) {
  char acBuffer[SEGGER_RTT_PRINTF_BUFFER_SIZE];
  va_list args;
  int n;

  va_start(args, sFormat);
  n = vsnprintf(acBuffer, sizeof(acBuffer), sFormat, args);
  va_end(args);

  if (n > 0) {
    SEGGER_RTT_Write(BufferIndex, acBuffer, (unsigned)n);
  }
  return n;
}

/*********************************************************************
*
*       SEGGER_RTT_ConfigUpBuffer()
*/
int SEGGER_RTT_ConfigUpBuffer(unsigned BufferIndex, const char* sName, void* pBuffer, unsigned BufferSize, unsigned Flags) {
  volatile SEGGER_RTT_CB* pRTTCB;

  _Init();
  pRTTCB = &_SEGGER_RTT;
  if (BufferIndex < SEGGER_RTT_MAX_NUM_UP_BUFFERS) {
    pRTTCB->aUp[BufferIndex].sName = sName;
    pRTTCB->aUp[BufferIndex].pBuffer = (char*)pBuffer;
    pRTTCB->aUp[BufferIndex].SizeOfBuffer = BufferSize;
    pRTTCB->aUp[BufferIndex].WrOff = 0u;
    pRTTCB->aUp[BufferIndex].RdOff = 0u;
    pRTTCB->aUp[BufferIndex].Flags = Flags;
    return 0;
  }
  return -1;
}

/*********************************************************************
*
*       SEGGER_RTT_ConfigDownBuffer()
*/
int SEGGER_RTT_ConfigDownBuffer(unsigned BufferIndex, const char* sName, void* pBuffer, unsigned BufferSize, unsigned Flags) {
  volatile SEGGER_RTT_CB* pRTTCB;

  _Init();
  pRTTCB = &_SEGGER_RTT;
  if (BufferIndex < SEGGER_RTT_MAX_NUM_DOWN_BUFFERS) {
    pRTTCB->aDown[BufferIndex].sName = sName;
    pRTTCB->aDown[BufferIndex].pBuffer = (char*)pBuffer;
    pRTTCB->aDown[BufferIndex].SizeOfBuffer = BufferSize;
    pRTTCB->aDown[BufferIndex].WrOff = 0u;
    pRTTCB->aDown[BufferIndex].RdOff = 0u;
    pRTTCB->aDown[BufferIndex].Flags = Flags;
    return 0;
  }
  return -1;
}

/*********************************************************************
*
*       SEGGER_RTT_AllocUpBuffer()
*/
int SEGGER_RTT_AllocUpBuffer(const char* sName, void* pBuffer, unsigned BufferSize, unsigned Flags) {
  int i;
  volatile SEGGER_RTT_CB* pRTTCB;

  _Init();
  pRTTCB = &_SEGGER_RTT;
  for (i = 1; i < SEGGER_RTT_MAX_NUM_UP_BUFFERS; i++) {
    if (pRTTCB->aUp[i].pBuffer == NULL) {
      SEGGER_RTT_ConfigUpBuffer((unsigned)i, sName, pBuffer, BufferSize, Flags);
      return i;
    }
  }
  return -1;
}

/*********************************************************************
*
*       SEGGER_RTT_AllocDownBuffer()
*/
int SEGGER_RTT_AllocDownBuffer(const char* sName, void* pBuffer, unsigned BufferSize, unsigned Flags) {
  int i;
  volatile SEGGER_RTT_CB* pRTTCB;

  _Init();
  pRTTCB = &_SEGGER_RTT;
  for (i = 1; i < SEGGER_RTT_MAX_NUM_DOWN_BUFFERS; i++) {
    if (pRTTCB->aDown[i].pBuffer == NULL) {
      SEGGER_RTT_ConfigDownBuffer((unsigned)i, sName, pBuffer, BufferSize, Flags);
      return i;
    }
  }
  return -1;
}

/*********************************************************************
*
*       SEGGER_RTT_SetNameUpBuffer()
*/
int SEGGER_RTT_SetNameUpBuffer(unsigned BufferIndex, const char* sName) {
  volatile SEGGER_RTT_CB* pRTTCB;

  _Init();
  pRTTCB = &_SEGGER_RTT;
  if (BufferIndex < SEGGER_RTT_MAX_NUM_UP_BUFFERS) {
    pRTTCB->aUp[BufferIndex].sName = sName;
    return 0;
  }
  return -1;
}

/*********************************************************************
*
*       SEGGER_RTT_SetNameDownBuffer()
*/
int SEGGER_RTT_SetNameDownBuffer(unsigned BufferIndex, const char* sName) {
  volatile SEGGER_RTT_CB* pRTTCB;

  _Init();
  pRTTCB = &_SEGGER_RTT;
  if (BufferIndex < SEGGER_RTT_MAX_NUM_DOWN_BUFFERS) {
    pRTTCB->aDown[BufferIndex].sName = sName;
    return 0;
  }
  return -1;
}

/*********************************************************************
*
*       SEGGER_RTT_SetFlagsUpBuffer()
*/
int SEGGER_RTT_SetFlagsUpBuffer(unsigned BufferIndex, unsigned Flags) {
  volatile SEGGER_RTT_CB* pRTTCB;

  _Init();
  pRTTCB = &_SEGGER_RTT;
  if (BufferIndex < SEGGER_RTT_MAX_NUM_UP_BUFFERS) {
    pRTTCB->aUp[BufferIndex].Flags = Flags;
    return 0;
  }
  return -1;
}

/*********************************************************************
*
*       SEGGER_RTT_SetFlagsDownBuffer()
*/
int SEGGER_RTT_SetFlagsDownBuffer(unsigned BufferIndex, unsigned Flags) {
  volatile SEGGER_RTT_CB* pRTTCB;

  _Init();
  pRTTCB = &_SEGGER_RTT;
  if (BufferIndex < SEGGER_RTT_MAX_NUM_DOWN_BUFFERS) {
    pRTTCB->aDown[BufferIndex].Flags = Flags;
    return 0;
  }
  return -1;
}

/*********************************************************************
*
*       SEGGER_RTT_WriteWithOverwriteNoLock()
*/
void SEGGER_RTT_WriteWithOverwriteNoLock(unsigned BufferIndex, const void* pBuffer, unsigned NumBytes) {
  (void)SEGGER_RTT_Write(BufferIndex, pBuffer, NumBytes);
}
