cmake_minimum_required(VERSION 3.13)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)
set(PICO_BOARD pico_w)
# Pull in FreeRTOS
include(FreeRTOS_Kernel_import.cmake)

project(main C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(
        main
        main.c
        IntQueueTimer.c
        RegTest.s
        ../Common/Minimal/blocktim.c
        ../Common/Minimal/countsem.c
        ../Common/Minimal/dynamic.c
        ../Common/Minimal/recmutex.c
        ../Common/Minimal/QueueOverwrite.c
        ../Common/Minimal/EventGroupsDemo.c
        ../Common/Minimal/IntSemTest.c
        ../Common/Minimal/IntQueue.c
        ../Common/Minimal/TaskNotify.c
        ../Common/Minimal/TimerDemo.c
        ../Common/Minimal/GenQTest.c
        ../Common/Minimal/death.c
        ../Common/Minimal/semtest.c
        ../Common/Minimal/BlockQ.c
        ../Common/Minimal/flop.c
        )

target_compile_definitions(main PRIVATE
        WIFI_SSID="NETGEAR97"
        WIFI_PASSWORD="purpletrail985"
        NO_SYS=0
        LWIP_SOCKET=1
        )

target_include_directories(main PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/../Common/include)

target_compile_definitions(main PRIVATE
        PICO_STDIO_STACK_BUFFER_SIZE=64 # use a small printf on stack buffer
)
target_link_libraries(
        main 
        pico_stdlib 
        FreeRTOS-Kernel 
        FreeRTOS-Kernel-Heap4
        pico_cyw43_arch_lwip_sys_freertos
)
# Enable usb output, disable uart output
pico_enable_stdio_usb(${PROJECT_NAME} 0)
pico_enable_stdio_uart(${PROJECT_NAME} 1)

pico_add_extra_outputs(main)