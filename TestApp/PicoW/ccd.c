#include "ccd.h"
#include "hardware/uart.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "string.h"
#define UART_ID uart1
#define UART_TX_PIN 4
#define UART_RX_PIN 5
#define CLK_OUTPUT_PIN 10
#define PROBE_OUTPUT_PIN 16
uint8_t CCD_message[16] = {0};
uint8_t CCD_message_length = 0;
uint8_t CCD_serialRxBuffer[16] = {0};
uint8_t CCD_serialRxBufferIndex = 0;
volatile uint8_t CCD_serialTxBuffer[16] = {0};
volatile uint8_t CCD_serialTxBufferIndex = 0;
volatile uint8_t CCD_serialTxBufferLength = 0;
volatile char CCD_TransmitAllowed = 0;
volatile char CCD_ProcessMessage = 0;
TaskHandle_t xCCDTaskHandle = NULL;
TaskHandle_t xCCDRxTaskHandle = NULL;
TaskHandle_t xCCDSpeedTaskHandle = NULL;
TimerHandle_t xCCDTimer1Handle = NULL;
SemaphoreHandle_t xSemaphoreUart = NULL;

uint32_t print_counter = 0;
uint8_t firstFlushFlag = 0;
uint32_t tx_counter = 0;
uint32_t tx_counter_speed = 0;
uint8_t lock_receive_task = 0;
uint8_t CCD_Checksum = 1;
uint8_t CCD_BusIdle = 1;
uint32_t CCD_BusBusyCounter = 0;
//The CCD Thread
void pvrCCDTask( void *pvParameters )
{
    (void)pvParameters;
    uint8_t i = 0;
    //Set the CLK pin to output
    //gpio_set_function(CLK_OUTPUT_PIN, GPIO_FUNC_PWM);
    //Set up the probe pin
    gpio_init(PROBE_OUTPUT_PIN);
    gpio_set_dir(PROBE_OUTPUT_PIN, GPIO_OUT);
    gpio_put(PROBE_OUTPUT_PIN, 1);
    vTaskDelay(100);
    gpio_put(PROBE_OUTPUT_PIN, 0);
    //Find out which PWM slice is connected to the pin
    //uint slice_num = pwm_gpio_to_slice_num(CLK_OUTPUT_PIN);
    // Set period of 4 cycles (0 to 3 inclusive)
    //pwm_set_wrap(slice_num, 120);
    // Set channel A output high for one cycle before dropping
    //pwm_set_chan_level(slice_num, PWM_CHAN_A, 60);
    // Set initial B output high for three cycles before dropping
    //pwm_set_chan_level(slice_num, PWM_CHAN_B, 60);
    // Set the PWM running
    //pwm_set_enabled(slice_num, true);
    printf("CCD Task Started\n\r");
    CCDINTERNAL_SerialInit(CCD_DEFAULT_SPEED);
    while(1)
    {   vTaskDelay(50);
        if( CCD_BusIdle == 1 )
        {
          //printf("CCD Bus Idle\n\r");
          vTaskDelay(1);
          CCD_SendMessage(messageAirbagOk, sizeof(messageAirbagOk));
        }
        else
        {
          //printf("CCD Bus Busy\n\r");
          CCD_BusBusyCounter++;
        }
    }
}
//The CCD Speed Thread
void pvrCCDSpeedTask( void *pvParameters )
{
  (void)pvParameters;
  uint8_t i = 10;
  printf("CCD Speed Task Started\n\r");
  while(1)
  {
    vTaskDelay(500);
    if( CCD_BusIdle == 1 )
    {
      CCD_SetMPH(i, messageVehicleSpeed);
      i+=10;
      if(i > 100)
      {
        i = 10;
      }
      vTaskDelay(1);
      CCD_SendMessage(messageVehicleSpeed, sizeof(messageVehicleSpeed));
    }
  }
}
//The CCD Rx Thread
void pvrCCDRxTask( void *pvParameters )
{
  (void)pvParameters;
  uint8_t ignore_byte;
  printf("CCD Rx Task Started\n\r");
  while(1)
  {
    if( xSemaphoreTake( xSemaphoreUart, 10 ) )
    {
      if( uart_is_readable(UART_ID) )
      {
        CCD_BusIdle = 0;
        if ( !CCD_ReceiveByte() )
        {
          //Check if Timer 1 is running
          if(xTimerIsTimerActive(xCCDTimer1Handle) == pdFALSE)
          {
            //Start Timer 1
            xTimerStart(xCCDTimer1Handle, 0);
          }
          else
          {
            //Reset Timer 1
            xTimerReset(xCCDTimer1Handle, 0);
          }
        }
      }
      xSemaphoreGive( xSemaphoreUart );
    }
  }
}
void vCCDTimer1Callback( TimerHandle_t xTimer )
{ 
  //Only the first time we flush the receive buffer after a timeout
  if(firstFlushFlag == 0)
  {
    memset(CCD_serialRxBuffer, 0, sizeof(CCD_serialRxBuffer));
    CCD_serialRxBufferIndex = 0;
    firstFlushFlag = 1;
  }
  else if ( CCD_serialRxBufferIndex > 0 )
  {
    CCD_BusIdle = 1;
    gpio_put(PROBE_OUTPUT_PIN, 1);
    CCDINTERNAL_ProcessMessage(CCD_serialRxBuffer, CCD_serialRxBufferIndex);
    gpio_put(PROBE_OUTPUT_PIN, 0);
  }
}
//CCD_SetMPH
//@param mph: The speed in MPH
//@param message: A pointer to the message array
void CCD_SetMPH(uint8_t mph, uint8_t *message)
{
  message[2] = mph;
}

//CCD_SendMessage
//@param message: A pointer to the message array
//@param length: The length of the message
void CCD_SendMessage(uint8_t *message, uint8_t length)
{
  if( xSemaphoreTake( xSemaphoreUart, ( TickType_t ) 10 ) == pdTRUE )
  {
    CCD_BusIdle = 0;
    //IF the message is larger than 1 calculate the checksum
    if(length > 1 & CCD_Checksum == 1)
    {
      uint8_t checksum = 0;
      uint8_t checksumLocation = length - 1;
      for (uint8_t i = 0; i < checksumLocation ; i++) checksum += message[i];
      message[checksumLocation] = checksum;
    }
    //Send the message
    for(uint8_t i = 0; i < length; i++)
    {
      CCD_SendByte(message[i]);
    }
    xSemaphoreGive( xSemaphoreUart );
    CCD_BusIdle = 1;
  }
}
void CCD_SendByte(uint8_t byte)
{
    uart_putc(UART_ID, byte);
}
uint8_t CCD_ReceiveByte(void)
{
  uint8_t byte = uart_getc(UART_ID);
  //Check if the uart byte is in the ignore list only if index is 0
  if ( CCD_serialRxBufferIndex == 0 )
  {
    for (uint8_t i = 0; i < sizeof(CCD_ignoreList); i++)
    {
      if ( CCD_ignoreList[i] ==  byte)
      {
        return 0xFF;
      }
    }
  }
  CCD_serialRxBuffer[CCD_serialRxBufferIndex] = byte;
  CCD_serialRxBufferIndex++;
  return 0;
}
//Process the CCD Message,
//@param message: A pointer to the message array
//@param length: The length of the message
void CCDINTERNAL_ProcessMessage(uint8_t *message, uint8_t length)
{
  // printf("CCD Message Received: \n\r");
  // //Print the message
  // for(uint8_t i = 0; i < 16; i++)
  // {
  //     printf("%x ", message[i]);
  // }
  // printf("\n\r");
  //Clear the buffer
  memset(message, 0, length);
  //Reset the buffer index
  CCD_ProcessMessage = 0;
  CCD_serialRxBufferIndex = 0;
}
//Initialize the UART for the CCD bus
void CCDINTERNAL_SerialInit(uint baudrate)
{
    uart_init(UART_ID, baudrate);

    //Set the TX and RX Pins
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    //Get the actual baudrate that will be set to the uart
    uint32_t actual_baudrate = uart_set_baudrate(UART_ID, baudrate);

    //Display the actual baudrate
    printf("Actual Baudrate: %d\n\r", actual_baudrate);

    //Disable control flow for the uart
    uart_set_hw_flow(UART_ID, false, false);

    //Set the UART to 8N1
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
}