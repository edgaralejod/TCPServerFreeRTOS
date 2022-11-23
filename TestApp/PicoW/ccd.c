#include "ccd.h"
#include "hardware/uart.h"

#define UART_ID uart1
#define UART_TX_PIN 4
#define UART_RX_PIN 5

uint8_t CCD_message[16] = {0};
uint8_t CCD_message_length = 0;
volatile uint8_t CCD_serialRxBuffer[16] = {0};
volatile uint8_t CCD_serialRxBufferIndex = 0;
volatile uint8_t CCD_serialTxBuffer[16] = {0};
volatile uint8_t CCD_serialTxBufferIndex = 0;
volatile uint8_t CCD_serialTxBufferLength = 0;
volatile char CCD_busIdle = 0;
volatile char CCD_TransmitAllowed = 0;
TaskHandle_t xCCDTaskHandle = NULL;

//The CCD Thread
void pvrCCDTask( void *pvParameters )
{
    (void)pvParameters;
    uint8_t i = 0;
    printf("CCD Task Started\n\r");
    CCDINTERNAL_SerialInit(CCD_DEFAULT_SPEED);
    while(1)
    {   
        vTaskDelay(100);
        CCD_SendByte(i);
        i++;
    }
}

void CCD_SendByte(uint8_t byte)
{
    uart_putc(UART_ID, byte);
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