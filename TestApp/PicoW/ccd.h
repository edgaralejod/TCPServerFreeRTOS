/*
 * Translation of the Arduino CCD library for the PicoW
 * Written in C instead of C++.
 * 
 * Copyright (C) 2022, Edgar Duarte
 * 
 * Original library available at: https://github.com/laszlodaniel/CCDLibrary
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef CCD_LIBRARY_H
#define CCD_LIBRARY_H
#include "FreeRTOS.h"
#include "semphr.h"
#include "timers.h"
#include "task.h"
#include "stdint.h"
#include "stdio.h"
#include "pico/stdlib.h"

static uint8_t messageAirbagOk[3] = {0x50, 0x00, 0x00};
static uint8_t messageAirbagBad[3] = {0x51, 0x00, 0x00};
static uint8_t messageVehicleSpeed[4] = {0x24, 0x02, 0x00, 0x00};
static uint8_t CCD_ignoreList[3] = { 0x22, 0x24, 0x51 };
#define CCD_DEFAULT_SPEED     7812 // default CCD-bus baudrate
#define UART_FRAME_ERROR      0x10 // framing error by UART
#define UART_OVERRUN_ERROR    0x08 // overrun condition by UART
#define UART_BUFFER_OVERFLOW  0x04 // receive buffer overflow
#define UART_NO_DATA          0x02 // receive buffer is empty
#define IDLE_BITS_05          5
#define IDLE_BITS_06          6
#define IDLE_BITS_07          7
#define IDLE_BITS_08          8
#define IDLE_BITS_09          9
#define IDLE_BITS_10          10   // CCD-bus idle condition
#define IDLE_BITS_11          11
#define IDLE_BITS_12          12
#define IDLE_BITS_13          13
#define IDLE_BITS_14          14
#define IDLE_BITS_15          15
#define ENABLE_RX_CHECKSUM    1    // verify received message checksum
#define DISABLE_RX_CHECKSUM   0
#define ENABLE_TX_CHECKSUM    1    // calculate outgoing message checksum
#define DISABLE_TX_CHECKSUM   0
#define CDP68HC68S1           1    // CDP68HC68S1 has two dedicated pins to signal CCD-bus condition
#define IDLE_PIN              2    // Arduino Mega: INT4 pin (CCD-bus idle interrupt)
#define CTRL_PIN              3    // Arduino Mega: INT5 pin (CCD-bus active byte (control) interrupt)
#define CUSTOM_TRANSCEIVER    0
#define TIMER1                0
#define TIMER1_TICKS          5
// Set (1), clear (0) and invert (1->0; 0->1) bit in a register or variable easily.
#define sbi(reg, bit) reg |=  (1 << bit)
#define cbi(reg, bit) reg &= ~(1 << bit)
#define ibi(reg, bit) reg ^=  (1 << bit)

extern TaskHandle_t xCCDTaskHandle;
extern TaskHandle_t xCCDRxTaskHandle;
extern TaskHandle_t xCCDSpeedTaskHandle;
extern TimerHandle_t xCCDTimer1Handle;
extern SemaphoreHandle_t xSemaphoreUart;

void pvrCCDTask( void *pvParameters );
void pvrCCDRxTask( void *pvParameters );
void pvrCCDSpeedTask( void *pvParameters );
void vCCDTimer1Callback( TimerHandle_t xTimer );
void CCD_SendMessage(uint8_t* message, uint8_t length);
void CCD_SetMPH(uint8_t mph, uint8_t *message);
//void CCD_Init(void);
//void CCD_Listen_All(void);
//void CCD_Listen(uint8_t* ids);
//void CCD_IgnoreAll(void);
//void CCD_Ignore(uint8_t* ids);
uint8_t CCD_ReceiveByte(void);
void CCD_SendByte(uint8_t byte);
//void CCD_TransmitDelayHandler(void);
//void CCD_timer1Handler(void);
//void CCD_BusIDleInterruptHandler(void);
//void CCD_ActiveByteInterruptHandler(void);
//void CCD_OnMessageReceived(void);
//void CCD_OnError(void);
void CCDINTERNAL_SerialInit(uint baudrate);
void CCDINTERNAL_ProcessMessage(uint8_t *message, uint8_t length);
//void CCDINTERNAL_TransmitDelayTimerInit(void);
//void CCDINTERNAL_TransmitDelayTimerStart(void);
//void CCDINTERNAL_TransmitDelayTimerStop(void);
//void CCDINTERNAL_ClockGeneratorInit(void);
//void CCDINTERNAL_BusIdleTimerInit(void);
//void CCDINTERNAL_BusIdleTimerStart(void);
//void CCDINTERNAL_BusIdleTimerStop(void);

#endif /*CCD_LIBRARY_H*/