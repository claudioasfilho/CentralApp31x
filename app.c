/***************************************************************************//**
 * @file
 * @brief Empty NCP-host Example Project.
 *
 * Reference implementation of an NCP (Network Co-Processor) host, which is
 * typically run on a central MCU without radio. It can connect to an NCP via
 * VCOM to access the Bluetooth stack of the NCP and to control it using BGAPI.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <errno.h>
#include "system.h"
#include "sl_bt_api.h"
#include "sl_bt_ncp_host.h"
#include "app_log.h"
#include "app_assert.h"
#include "uart.h"
#include "app.h"
#include "tcp.h"
#include <unistd.h>

#define USAGE "\n%s\n"                                                                         \
              "Usage: -u <serial port> <baud rate> [flow control: 1(on, default) or 0(off)]\n" \
              "       -t <IPv4 address (e.g. 192.168.0.0)>\n\n"
#define DEFAULT_UART_PORT             NULL
#define DEFAULT_UART_BAUD_RATE        115200
#define DEFAULT_UART_FLOW_CONTROL     1
#define DEFAULT_UART_TIMEOUT          100
#define DEFAULT_TCP_PORT              "4901"

// This characteristic handle value has to match the value in gatt_db.h of
// NCP empty example running on the connected WSTK.
#define GATTDB_SYSTEM_ID 18

SL_BT_API_DEFINE();

// The advertising set handle allocated from Bluetooth stack.
//static uint8_t advertising_set_handle = 0xff;

static void uart_tx_wrapper(uint32_t len, uint8_t *data);

static void tcp_tx_wrapper(uint32_t len, uint8_t *data);

typedef struct {
  bd_addr	address;
  uint8_t	address_type;
  uint8_t connection;
}CONN_DEVICES;

static CONN_DEVICES Connectable_Devices_Array[40];
static uint8_t Connectable_Devices_Counter = 0;
static uint8_t Connected_Devices_Counter = 0;
static uint8_t Connecting_Devices_Counter = 0;
static uint8_t ConInnProcessTimeCounter = 0;

#define MAX_NUMBER_OF_CONNECTABLE_DEVICES 40
#define MAX_NUMBER_OF_CONNECTIONS 32

#define CENTRAL_SOFTTIMER_HANDLER 0xFE

typedef enum CentralStages
{
      Disabled,
      Scanning,
      Scanning_Completed,
      Connecting_Devices,
      Connection_in_Process,
      Connections_Completed,
      Collecting_Data
}CENTRALSTAGES;

CENTRALSTAGES Central_State = Disabled;
typedef struct {
  uint16_t connection_handle;   //This is used for connection handle for connection oriented, and for sync handle for connection less mode
  bd_addr address;
  uint8_t address_type;
  uint32_t cte_service_handle;
  uint16_t cte_enable_char_handle;
  //connection_state_t connection_state;
  //aoa_libitems_t aoa_states;
} conn_properties_t;


//#define AD_FIELD_I 0x06
//#define AD_FIELD_C 0x07

#define SERVICE_UUID_LEN 2
//static const uint8_t cte_service[SERVICE_UUID_LEN] = { 0x9, 0x18 };

const char DEVICE_NAME_STRING[] = "Silabsxx:xx";

/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
void app_init(int argc, char *argv[])
{
  char *uartPort = DEFAULT_UART_PORT;
  uint32_t uartBaudRate = DEFAULT_UART_BAUD_RATE;
  uint32_t uartFlowControl = DEFAULT_UART_FLOW_CONTROL;
  int opt;

  // Get connection option from command line.
  // Allow only one connection at a time.
  opt = getopt(argc, argv, "t:T:u:U:hH");
  switch (opt) {
    // TCP connection
    case 't':
    case 'T':
      SL_BT_API_INITIALIZE_NONBLOCK(tcp_tx_wrapper, tcp_rx, tcp_rx_peek);

      // Initialise socket communication
      if (tcp_open(optarg, DEFAULT_TCP_PORT) < 0) {
        app_log("Non-blocking TCP connection init failure\n");
        exit(EXIT_FAILURE);
      }

      break;

    // UART connection
    case 'u':
    case 'U':
      SL_BT_API_INITIALIZE_NONBLOCK(uart_tx_wrapper, uartRx, uartRxPeek);

      // Handle the command-line arguments.
      switch (argc) {
        case 5:
          uartFlowControl = atoi(argv[4]);
        // Falls through on purpose.
        case 4:
          uartBaudRate = atoi(argv[3]);
        case 3:
          uartPort = argv[2];
        // Falls through on purpose.
        default:
          break;
      }
      if (!uartPort || !uartBaudRate || (uartFlowControl > 1)) {
        app_log(USAGE, argv[0]);
        app_log("\n%s: option requires an argument -- u\n", argv[0]);
        exit(EXIT_FAILURE);
      }
      // Initialise the serial port with RTS/CTS enabled.
      // Initialise serial communication as non-blocking.
      if (uartOpen((int8_t*)uartPort, uartBaudRate, uartFlowControl, DEFAULT_UART_TIMEOUT) < 0) {
        app_log("Non-blocking serial port init failure\n");
        exit(EXIT_FAILURE);
      }

      break;

    // Print help
    case 'h':
    case 'H':
    default:
      app_log(USAGE, argv[0]);
      exit(EXIT_FAILURE);
  }

  app_log("Empty NCP-host initialised\n");
  app_log("Resetting NCP...\n");
  // Reset NCP to ensure it gets into a defined state.
  // Once the chip successfully boots, boot event should be received.
  sl_bt_system_reset(0);

  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application init code here!                         //
  // This is called once during start-up.                                    //
  /////////////////////////////////////////////////////////////////////////////
}

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
void app_process_action(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application code here!                              //
  // This is called infinitely.                                              //
  // Do not call blocking functions from here!                               //
  /////////////////////////////////////////////////////////////////////////////
}



void Change_Central_State(CENTRALSTAGES Temp_Central_State)
{
  switch (Temp_Central_State)
  {
    case Disabled:
      Central_State = Disabled;
      printf("Central_State = Disabled\n\r");
    break;

    case Scanning:
      Central_State = Scanning;
      printf("Central_State = Scanning\n\r");
    break;

    case Scanning_Completed:
      Central_State = Scanning_Completed;
      printf("Central_State = Scanning_Completed\n\r");
    break;

    case Connecting_Devices:
      Central_State = Connecting_Devices;
      printf("Central_State = Connecting_Devices\n\r");
    break;

    case Connection_in_Process:
      Central_State = Connection_in_Process;
      printf("Central_State = Connection_in_Process\n\r");
    break;

    case Connections_Completed:
      Central_State = Connections_Completed;
      printf("Central_State = Connections_Completed\n\r");
    break;

    case Collecting_Data:
      Central_State = Collecting_Data;
      printf("Central_State = Collecting_Data\n\r");
    break;

  }

}

//To Connection//sl_bt_connection_open(00:0b:57:b5:f1:dd, 0, 1)

//sl_bt_gatt_set_characteristic_notification(1, 24, 2)
static int process_scan_response(struct sl_bt_evt_scanner_scan_report_s *pResp) {
  int i = 0;
  int adMatchFound = 0;
  int adLen;
  int adType;

  while (i < (pResp->data.len - 1)) {
    adLen = pResp->data.data[i];
    adType = pResp->data.data[i + 1];

    // Type 0x09 = Complete Local Name, 0x08 Shortened Name
    if (adType == 0x09) {
      // Check if device name is Throughput Tester
      if (memcmp(pResp->data.data + i + 2, DEVICE_NAME_STRING, 6) == 0) {
        adMatchFound = 1;
        //printf(" Match found \n\r");
        //printf("Connectable devices counter %d",Connectable_Devices_Counter);
      //  if (Connectable_Devices_Counter < 32)
        {
          Connectable_Devices_Array[Connectable_Devices_Counter].address = pResp->address;
          Connectable_Devices_Array[Connectable_Devices_Counter].address_type = pResp->address_type;
          //printf("Connectable devices found %d",Connectable_Devices_Counter);
          Connectable_Devices_Counter++;
        }
        break;
      }
      else printf(" NO Match \n\r");
    }
    // Jump to next AD record
    i = i + adLen + 1;
  }

  return (adMatchFound);
}

void printDeviceAddress(bd_addr address)
{
  printf("%2x:%2x:%2x:%2x:%2x:%2x \n\r",address.addr[5],address.addr[4],address.addr[3],address.addr[2],address.addr[1],address.addr[0]);
}
/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  //sl_status_t sc;
  //uint8_t conn_handle;
  //conn_properties_t* conn;



  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:
      // Print boot message.
      app_log("Bluetooth stack booted: v%d.%d.%d-b%d\n",
              evt->data.evt_system_boot.major,
              evt->data.evt_system_boot.minor,
              evt->data.evt_system_boot.patch,
              evt->data.evt_system_boot.build);

      Connectable_Devices_Counter = 0;
      Connected_Devices_Counter = 0;
      Central_State = Disabled;

      //Set a continuous Timer to drive the Central App State Machine
      sl_bt_system_set_soft_timer(1*32768, CENTRAL_SOFTTIMER_HANDLER, 0);
      sl_bt_scanner_start(1, 1);

      Change_Central_State(Scanning);


      break;

      case sl_bt_evt_scanner_scan_report_id:
        // Check if the tag is whitelisted
      {

        process_scan_response(&(evt->data.evt_scanner_scan_report));

        if (Connectable_Devices_Counter == MAX_NUMBER_OF_CONNECTABLE_DEVICES+1)
        {
          sl_bt_scanner_stop();
          printf("Final count on Connectable devices %d\n\r",Connectable_Devices_Counter+1);
          printf("List of Connectable devices\n\r");

          for (uint8_t i =0 ; i<MAX_NUMBER_OF_CONNECTABLE_DEVICES-1; i++)
          {
            printDeviceAddress(Connectable_Devices_Array[i].address);
            //printf("%2x:%2x:%2x:%2x:%2x:%2x \n\r",Connectable_Devices_Array[i].address.addr[5],Connectable_Devices_Array[i].address.addr[4],Connectable_Devices_Array[i].address.addr[3],Connectable_Devices_Array[i].address.addr[2],Connectable_Devices_Array[i].address.addr[1],Connectable_Devices_Array[i].address.addr[0]);
          }

          Change_Central_State(Scanning_Completed);
        }


      }
        break;

    case sl_bt_evt_system_soft_timer_id:

    if (evt->data.evt_system_soft_timer.handle == CENTRAL_SOFTTIMER_HANDLER)
           {
             if (Central_State == Scanning_Completed)
             {
               Change_Central_State(Connecting_Devices);
               Connected_Devices_Counter = 0;
               Connecting_Devices_Counter = 0;

             }
             //It connects to devices on the connectable devices list
             if ((Connected_Devices_Counter <= MAX_NUMBER_OF_CONNECTIONS) && (Central_State == Connecting_Devices))
             {

               sl_bt_connection_open(Connectable_Devices_Array[Connecting_Devices_Counter].address, Connectable_Devices_Array[Connecting_Devices_Counter].address_type, 1,&Connectable_Devices_Array[Connecting_Devices_Counter].connection);
               Change_Central_State(Connection_in_Process);
               ConInnProcessTimeCounter = 0;
             }

             //If Stuck trying to connect
             if ((Central_State == Connection_in_Process)&&(ConInnProcessTimeCounter++ == 4))
             {
               //uint8_t temp = Connecting_Devices_Counter + 1;
               printf("Device Skiped ");
               printDeviceAddress(Connectable_Devices_Array[Connecting_Devices_Counter].address);
               Connecting_Devices_Counter++;

               //sl_bt_connection_open(Connectable_Devices_Array[temp].address, Connectable_Devices_Array[temp].address_type, 1,&Connectable_Devices_Array[temp].connection);
               sl_bt_connection_open(Connectable_Devices_Array[Connecting_Devices_Counter].address, Connectable_Devices_Array[Connecting_Devices_Counter].address_type, 1,&Connectable_Devices_Array[Connecting_Devices_Counter].connection);
               ConInnProcessTimeCounter = 0;
             }


           }
      //sl_bt_scanner_stop();

      break;
    // -------------------------------
    // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:
      app_log("Connection opened %d\n", Connected_Devices_Counter++);

      if (Connected_Devices_Counter <= MAX_NUMBER_OF_CONNECTIONS)
      {
        Change_Central_State(Connecting_Devices);
      }
      else
      {
        Change_Central_State(Connections_Completed);

      }
      Connecting_Devices_Counter++;
      break;

    // -------------------------------
    // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:
      app_log("Connection closed\n");
      Connected_Devices_Counter--;
      // Restart advertising after client has disconnected.

      break;

    ///////////////////////////////////////////////////////////////////////////
    // Add additional event handlers here as your application requires!      //
    ///////////////////////////////////////////////////////////////////////////

    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}

/**************************************************************************//**
 * UART TX Wrapper.
 *****************************************************************************/
static void uart_tx_wrapper(uint32_t len, uint8_t *data)
{
  if (0 > uartTx(len, data)) {
    app_log("Failed to write to serial port\n");
    exit(EXIT_FAILURE);
  }
}

/**************************************************************************//**
 * TCP TX Wrapper.
 *****************************************************************************/
static void tcp_tx_wrapper(uint32_t len, uint8_t *data)
{
  if (0 > tcp_tx(len, data)) {
    app_log("Failed to write to TCP port\n");
    tcp_close();
    exit(EXIT_FAILURE);
  }
}
