/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Library includes. */
#include <stdio.h>
#include "hardware/gpio.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "lwip/sockets.h"
#include "ccd.h"

/* Priorities at which the tasks are created. */
#define mainQUEUE_RECEIVE_TASK_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define	mainQUEUE_SEND_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )

/* The rate at which data is sent to the queue.  The 200ms value is converted
to ticks using the portTICK_PERIOD_MS constant. */
#define mainQUEUE_SEND_FREQUENCY_MS			( 1000 / portTICK_PERIOD_MS )
#define IPERF_SERVER_IP "192.168.2.15"
/* The number of items the queue can hold.  This is 1 as the receive task
will remove items as they are added, meaning the send task should always find
the queue empty. */
#define mainQUEUE_LENGTH					( 1 )


/* Prototypes for the standard FreeRTOS callback/hook functions implemented
within this file. */
void vApplicationMallocFailedHook( void );
void vApplicationIdleHook( void );
void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName );
void vApplicationTickHook( void );
/*Other Prototypes used in this file*/
static void pvrHeartBeatTask( void *pvParameters );
static void pvrWifiTask( void *pvParameters );
static void run_server();
static void send_message(int socket, char *msg);
static int handle_single_command(int conn_sock);
static void handle_connection(int conn_sock);

TaskHandle_t xWifiTaskHandle;
TaskHandle_t xHeartBeatHandle;
extern TaskHandle_t xCCDTaskHandle;

/* Global variables */
char isLedOn = 0;
char isWifiInit = 0;
uint8_t test = 0;
UBaseType_t uxCoreAffinityMask;
void main( void )
{
	stdio_init_all();
    printf(" Starting Program.\n");
	xTaskCreate( pvrHeartBeatTask, "Hearbeat", configMINIMAL_STACK_SIZE, NULL, mainQUEUE_SEND_TASK_PRIORITY, &xHeartBeatHandle );
	xTaskCreate( pvrWifiTask, "Wifi", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, &xWifiTaskHandle );
    xTaskCreate( pvrCCDTask, "CCD", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, &xCCDTaskHandle );
    uxCoreAffinityMask = ( ( 1 << 0 ) );
    vTaskCoreAffinitySet( xWifiTaskHandle, uxCoreAffinityMask );
    uxCoreAffinityMask = ( ( 1 << 1 ) );
    vTaskCoreAffinitySet( xHeartBeatHandle, uxCoreAffinityMask );
    vTaskCoreAffinitySet( xCCDTaskHandle, uxCoreAffinityMask );
	vTaskStartScheduler();
	/* If all is well, the scheduler will now be running, and the following
	line will never be reached.  If the following line does execute, then
	there was insufficient FreeRTOS heap memory available for the Idle and/or
	timer tasks to be created.  See the memory management section on the
	FreeRTOS web site for more details on the FreeRTOS heap
	http://www.freertos.org/a00111.html. */
	for( ;; );
}
/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
static void pvrHeartBeatTask( void *pvParameters )
{
TickType_t xNextWakeTime;
const unsigned long ulValueToSend = 100UL;

	/* Remove compiler warning about unused parameter. */
	( void ) pvParameters;


	for( ;; )
	{
		vTaskDelay( 500 );
        /* Toggle the LED. */
        if (isLedOn) {
            if ( isWifiInit )
                cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
            isLedOn = 0;
        } else {
            if ( isWifiInit )
                cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
            //printf("Turning You ON!\n\r");
            isLedOn = 1;
        }
	}
}
/*-----------------------------------------------------------*/

static void pvrWifiTask( void *pvParameters )
{
	char input = 0;
	/* Remove compiler warning about unused parameter. */
	( void ) pvParameters;
    vTaskDelay( 1000 );
    //Initialize the cyw43 wifi module and check for success
    if (cyw43_arch_init() != 0) {
        printf("Failed to initialize the CYW43 module.\r");
        vTaskDelete( NULL );
    }
    isWifiInit = 1;
    cyw43_arch_enable_sta_mode();
    printf("Station mode enabled.\r");
    //Connect to the AP and check for success
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000))  {
        printf("Failed to connect to the AP.\r");
        vTaskDelete( NULL );
    }
    //Start the TCP server
    run_server();
	for( ;; )
	{	
        vTaskDelay( 1000 );
	}
}
/*-----------------------------------------------------------*/
void vApplicationMallocFailedHook( void )
{
    /* Called if a call to pvPortMalloc() fails because there is insufficient
    free memory available in the FreeRTOS heap.  pvPortMalloc() is called
    internally by FreeRTOS API functions that create tasks, queues, software
    timers, and semaphores.  The size of the FreeRTOS heap is set by the
    configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

    /* Force an assert. */
    configASSERT( ( volatile void * ) NULL );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
    ( void ) pcTaskName;
    ( void ) pxTask;

    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected. */

    /* Force an assert. */
    configASSERT( ( volatile void * ) NULL );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
    volatile size_t xFreeHeapSpace;

    /* This is just a trivial example of an idle hook.  It is called on each
    cycle of the idle task.  It must *NOT* attempt to block.  In this case the
    idle task just queries the amount of FreeRTOS heap that remains.  See the
    memory management section on the http://www.FreeRTOS.org web site for memory
    management options.  If there is a lot of heap memory free then the
    configTOTAL_HEAP_SIZE value in FreeRTOSConfig.h can be reduced to free up
    RAM. */
    xFreeHeapSpace = xPortGetFreeHeapSize();

    /* Remove compiler warning about xFreeHeapSpace being set but never used. */
    ( void ) xFreeHeapSpace;
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
	/* This function will be called by each tick interrupt if
	configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
	added here, but the tick hook is called from an interrupt context, so
	code must not attempt to block, and only the interrupt safe FreeRTOS API
	functions can be used (those that end in FromISR()). */
}

static void send_message(int socket, char *msg)
{
    int len = strlen(msg);
    int done = 0;
    while (done < len)
    {
        int done_now = send(socket, msg + done, len - done, 0);
        if (done_now <= 0)
            return;
        done += done_now;
    }
}

static int handle_single_command(int conn_sock)
{
    char buffer[128];
    int done = 0;
    send_message(conn_sock, "Enter command: ");
 
    while (done < sizeof(buffer))
    {
        int done_now = recv(conn_sock, buffer + done, sizeof(buffer) - done, 0);
        if (done_now <= 0)
            return -1;
        done += done_now;
        char *end = strnstr(buffer, "\r", done);
        if (!end)
            continue;
        *end = 0;
 
        if (!strcmp(buffer, "on"))
        {
            cyw43_arch_gpio_put(0, true);
            send_message(conn_sock, "The LED is now on\r\n");
        }
        else if (!strcmp(buffer, "off"))
        {
            cyw43_arch_gpio_put(0, false);
            send_message(conn_sock, "The LED is now off\r\n");
        }
        else
        {
            send_message(conn_sock, "Unknown command\r\n");
        }
        break;
    }
 
    return 0;
}
 
static void handle_connection(int conn_sock)
{
    while (!handle_single_command(conn_sock))
    {
    }
 
    closesocket(conn_sock);
}
 
static void run_server()
{
    int server_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    struct sockaddr_in listen_addr =
        {
            .sin_len = sizeof(struct sockaddr_in),
            .sin_family = AF_INET,
            .sin_port = htons(1234),
            .sin_addr = 0,
        };
 
    if (server_sock < 0)
    {
        printf("Unable to create socket: error %d", errno);
        return;
    }
 
    if (bind(server_sock, (struct sockaddr *)&listen_addr, sizeof(listen_addr)) < 0)
    {
        printf("Unable to bind socket: error %d\n", errno);
        return;
    }
 
    if (listen(server_sock, 1) < 0)
    {
        printf("Unable to listen on socket: error %d\n", errno);
        return;
    }
 
    printf("Starting server at %s on port %u\n", ip4addr_ntoa(netif_ip4_addr(netif_list)), ntohs(listen_addr.sin_port));
 
    while (true)
    {
        struct sockaddr_storage remote_addr;
        socklen_t len = sizeof(remote_addr);
        int conn_sock = accept(server_sock, (struct sockaddr *)&remote_addr, &len);
        if (conn_sock < 0)
        {
            printf("Unable to accept incoming connection: error %d\n", errno);
            return;
        }
        handle_connection(conn_sock);
    }
}