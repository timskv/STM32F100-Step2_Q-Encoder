/****************************************************************************
 *
 * MODULE:             Utils
 *
 * COMPONENT:          $RCSfile: Utils.h,v $
 *
 * VERSION:            $Name:  $
 *
 * REVISION:           $Revision: 1.3 $
 *
 * DATED:              $Date: 2008-10-15 09:30:44 $
 *
 * STATUS:             $State: Exp $
 *
 * AUTHOR:             GP
 *
 * DESCRIPTION:
 *
 *
 * LAST MODIFIED BY:   $Author: ja $
 *                     $Modtime: $
 *
 ****************************************************************************
 *
 * This software is owned by Jennic and/or its supplier and is protected
 * under applicable copyright laws. All rights are reserved. We grant You,
 * and any third parties, a license to use this software solely and
 * exclusively on Jennic products. You, and any third parties must reproduce
 * the copyright and warranty notice and any other legend of ownership on each
 * copy or partial copy of the software.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS". JENNIC MAKES NO WARRANTIES, WHETHER
 * EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * ACCURACY OR LACK OF NEGLIGENCE. JENNIC SHALL NOT, IN ANY CIRCUMSTANCES,
 * BE LIABLE FOR ANY DAMAGES, INCLUDING, BUT NOT LIMITED TO, SPECIAL,
 * INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON WHATSOEVER.
 *
 * Copyright Jennic Ltd 2008. All rights reserved
 *
 ****************************************************************************/
#ifndef UTILS_H
#define UTILS_H
#ifdef __cplusplus
extern "C" {
#endif  //__cplusplus
/*****************************************************************************************
**  INCLUDE FILES
*****************************************************************************************/
/*****************************************************************************************
**  MACROS
*****************************************************************************************/
/*****************************************************************************************
**  CONSTANTS
*****************************************************************************************/
/*****************************************************************************************
**  TYPEDEFS
*****************************************************************************************/
/*****************************************************************************************
**  EXTERNAL VARIABLES
*****************************************************************************************/
/*****************************************************************************************
**  GLOBAL VARIABLES
*****************************************************************************************/
/*****************************************************************************************
**  LOCAL VARIABLES
*****************************************************************************************/
/*****************************************************************************************
**  EXPORTED FUNCTIONS
*****************************************************************************************/
#include "stm32f10x_usart.h"
#define UTILS_UART USART1


void vUtils_Init(void);

void vUtils_DisplayHex(uint32_t u32Data, int iSize);

void vUtils_DisplayHexNoZero(uint32_t u32Data, int iSize);

void vUtils_DisplayDec(uint8_t u8Data);
void vUtils_Debug(char *pcMessage);

void vUtils_DebugNoZero(char *pcMessage);

void vUtils_DisplayMsg(char *pcMessage, uint32_t u32Data);

void vUtils_TrueDisplayMsg(uint32_t u32Data);

void vUtils_String(char *pcMessage);
void vUtils_Char(uint8_t u8char);
void vUtils_ValToHex(char *pcOutString, uint32_t u32Data, int iSize);

void vUtils_ValToHexNoZero(char *pcOutString, uint32_t u32Data, int iSize);

void vUtils_ValToDec(char *pcOutString, uint8_t u8Value);
void vUtils_DisplayBytes(uint8_t *pcOutString, uint8_t u8Num);
void vUtils_WriteChar(char u8chr);


/*****************************************************************************************
**  LOCAL FUNCTIONS DESC
*****************************************************************************************/
/*****************************************************************************************
**  LOCAL FUNCTIONS
*****************************************************************************************/
#ifdef __cplusplus
}
#endif  //__cplusplus
#endif  // UTILS_H
/*****************************************************************************************
**  END OF FILE
*****************************************************************************************/



/****************************************************************************
 *
 * NAME: vUtils_DisplayMsg
 *
 * DESCRIPTION:
 * Used to display text plus number
 *
 * PARAMETERS:      Name            RW  Usage
 *                  pcMessage       R   Message to display
 *                  u32Data         R   Data to display
 *
 * RETURNS:
 * void, never returns
 *
 ****************************************************************************/
void vUtils_DisplayMsg(char *pcMessage, uint32_t u32Data)
{
    vUtils_Debug(pcMessage);
    vUtils_DisplayHex(u32Data, 8);
}

 void vUtils_TrueDisplayMsg(uint32_t u32Data)
{
    vUtils_DisplayHexNoZero(u32Data, 2);
    vUtils_DebugNoZero("-");
}
/****************************************************************************
 *
 * NAME: vUtils_Debug
 *
 * DESCRIPTION:
 * Sends a string to UART0 using the hardware API with CR
 *
 * PARAMETERS:      Name            RW  Usage
 *                  pcMessage       R   Null-terminated message to send
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
void vUtils_Debug(char *pcMessage)
{
    vUtils_String(pcMessage);
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
	{
	}
	USART_SendData(UTILS_UART, '\r');
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
	{
	}
	USART_SendData(UTILS_UART, '\n');
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
	{
	}
}

void vUtils_DebugNoZero(char *pcMessage)
{
    vUtils_String(pcMessage);
}


/****************************************************************************
 *
 * NAME: vUtils_String
 *
 * DESCRIPTION:
 * Sends a string to UART0 using the hardware API without CR
 *
 * PARAMETERS:      Name            RW  Usage
 *                  pcMessage       R   Null-terminated message to send
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
void vUtils_String(char *pcMessage)
{
    while (*pcMessage)
    {
    	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
    	{
    	}
    	USART_SendData(UTILS_UART, *pcMessage);
    	pcMessage++;
    }
}
/****************************************************************************
 *
 * NAME: vUtils_String
 *
 * DESCRIPTION:
 * Sends a string to UART0 using the hardware API without CR
 *
 * PARAMETERS:      Name            RW  Usage
 *                  pcMessage       R   Null-terminated message to send
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
void vUtils_Char(uint8_t u8char)
{
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
	{
	}
	USART_SendData(UTILS_UART, u8char);
}
/****************************************************************************
 *
 * NAME: vUtils_DisplayHex
 *
 * DESCRIPTION:
 *
 * PARAMETERS:      Name            RW  Usage
 *                  u32Data         R   Value to send
 *                  iSize           R   Length of value
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
void vUtils_DisplayHex(uint32_t u32Data, int iSize)
{
    char acValue[9];
    vUtils_ValToHex(acValue, u32Data, 8);
    vUtils_Debug(acValue);
}

void vUtils_DisplayHexNoZero(uint32_t u32Data, int iSize)
{
    char acValue[9];
    vUtils_ValToHexNoZero(acValue, u32Data, 2);
    vUtils_DebugNoZero(acValue);
}
/****************************************************************************
 *
 * NAME: vUtils_DisplayDec
 *
 * DESCRIPTION:
 *
 * PARAMETERS:      Name            RW  Usage
 *                  u32Data         R   Value to send
 *                  iSize           R   Length of value
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
void vUtils_DisplayDec(uint8_t u8Data)
{
    char acValue[3];
    vUtils_ValToDec(acValue, u8Data);
    vUtils_Debug(acValue);
}
/****************************************************************************
 *
 * NAME: vUtils_DisplayHex
 *
 * DESCRIPTION:
 *
 * PARAMETERS:      Name            RW  Usage
 *                  u32Data         R   Value to send
 *                  iSize           R   Length of value
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
void vUtils_ValToHex(char *pcString, uint32_t u32Data, int iSize)
{
    uint8_t u8Nybble;
    int i, j;
    j = 0;
    for (i = (iSize << 2) - 4; i >= 0; i -= 4)
    {
        u8Nybble = (uint8_t)((u32Data >> i) & 0x0f);
        u8Nybble += 0x30;
        if (u8Nybble > 0x39)
            u8Nybble += 7;
        *pcString = u8Nybble;
        pcString++;
    }
    *pcString = '\0';
}

void vUtils_ValToHexNoZero(char *pcString, uint32_t u32Data, int iSize)
{
    uint8_t u8Nybble;
    int i, j;
    j = 0;
    for (i = (iSize << 2) - 4; i >= 0; i -= 4)
    {
        u8Nybble = (uint8_t)((u32Data >> i) & 0x0f);
        u8Nybble += 0x30;
        if (u8Nybble > 0x39)
            u8Nybble += 7;
        *pcString = u8Nybble;
        pcString++;
    }
}
/****************************************************************************
 *
 * NAME: vUtils_ValToDec
 *
 * DESCRIPTION:
 * Converts an 8-bit value to a string of the textual decimal representation.
 *
 * PARAMETERS:      Name            RW  Usage
 *                  pcOutString     R   Location for new string
 *                  u8Value         R   Value to convert
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
void vUtils_ValToDec(char *pcOutString, uint8_t u8Value)
{
    static const uint8_t au8Digits[3] = {100, 10, 1};
    uint8_t u8Digit;
    uint8_t u8DigitIndex;
    uint8_t u8Count;
    uint8_t boPreviousDigitPrinted = 1;// FALSE;
    for (u8DigitIndex = 0; u8DigitIndex < 3; u8DigitIndex++)
    {
        u8Count = 0;
        u8Digit = au8Digits[u8DigitIndex];
        while (u8Value >= u8Digit)
        {
            u8Value -= u8Digit;
            u8Count++;
        }
        if ((u8Count != 0) || (boPreviousDigitPrinted == 1)
            || (u8DigitIndex == 2))
        {
            *pcOutString = '0' + u8Count;
            boPreviousDigitPrinted = 1;
            pcOutString++;
        }
    }
    *pcOutString = '\0';
}
void vUtils_DisplayBytes(uint8_t *pcOutString, uint8_t u8Num)
{
uint8_t chr;
    while(u8Num--)
    {
        chr=(*pcOutString >> 4)+0x30;
        if(chr > 0x39)
        {
            chr+=0x07;
        }

        USART_SendData(UTILS_UART, chr);
    	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
    	{
    	}


        chr=(*pcOutString & 0x0f)+0x30;
        if(chr > 0x39)
        {
            chr+=0x07;
        }


        USART_SendData(UTILS_UART, chr);
    	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
    	{
    	}

        USART_SendData(UTILS_UART, ' ');
    	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
    	{
    	}

        pcOutString++;
    }
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
	{
	}
	USART_SendData(UTILS_UART, '\r');
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
	{
	}
	USART_SendData(UTILS_UART, '\n');
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
	{
	}
}
