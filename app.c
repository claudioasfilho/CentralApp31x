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
static uint8_t advertising_set_handle = 0xff;

static void uart_tx_wrapper(uint32_t len, uint8_t *data);

static void tcp_tx_wrapper(uint32_t len, uint8_t *data);

typedef struct {
  bd_addr	address;
  uint8_t	address_type;
}CONN_DEVICES;

static CONN_DEVICES Connectable_Devices_Array[40];
static uint8_t Connectable_Devices_Counter = 0;

#define MAX_NUMBER_OF_CONNECTABLE_DEVICES 35
typedef struct {
  uint16_t connection_handle;   //This is used for connection handle for connection oriented, and for sync handle for connection less mode
  bd_addr address;
  uint8_t address_type;
  uint32_t cte_service_handle;
  uint16_t cte_enable_char_handle;
  //connection_state_t connection_state;
  //aoa_libitems_t aoa_states;
} conn_properties_t;


#define AD_FIELD_I 0x06
#define AD_FIELD_C 0x07

#define SERVICE_UUID_LEN 2
static const uint8_t cte_service[SERVICE_UUID_LEN] = { 0x9, 0x18 };

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
      else printf(" NO MF Match \n\r");
    }
    // Jump to next AD record
    i = i + adLen + 1;
  }

  return (adMatchFound);
}

// uint8_t find_service_in_advertisement(uint8_t *advdata, uint8_t advlen, uint8_t *service_uuid)
// {
//   uint8_t ad_field_length;
//   uint8_t ad_field_type;
//   uint8_t *ad_uuid_field;
//   uint32_t i = 0;
//   uint32_t next_ad_structure;
//   uint8_t ret = 0;
//
//   // Parse advertisement packet
//   while (i < advlen) {
//     ad_field_length = advdata[i];
//     ad_field_type = advdata[i + 1];
//     next_ad_structure = i + ad_field_length + 1;
//     // incomplete or complete UUIDs
//     if (ad_field_type == AD_FIELD_I || ad_field_type == AD_FIELD_C) {
//       // compare UUID to the service UUID to be found
//       for (ad_uuid_field = advdata + i + 2; ad_uuid_field < advdata + next_ad_structure; ad_uuid_field += SERVICE_UUID_LEN) {
//         if (memcmp(ad_uuid_field, service_uuid, SERVICE_UUID_LEN) == 0) {
//           ret = 1;
//           break;
//         }
//       }
//       if (ret == 1) {
//         break;
//       }
//     }
//     // advance to the next AD struct
//     i = next_ad_structure;
//   }
//   return ret;
// }

/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;
  uint8_t conn_handle;
  conn_properties_t* conn;



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

      //sl_bt_scanner_set_timing(1,0xff,0xff);
      sl_bt_system_set_soft_timer(5*32768, 0xFE, 1);
      sl_bt_scanner_start(1, 1);

      break;

      case sl_bt_evt_scanner_scan_report_id:
        // Check if the tag is whitelisted
      {


        //printf("Number of devices found %d \n\r",sizeof(evt->data.evt_scanner_scan_report.address.addr) );

        // printf("\n\r address ---> ");
        // for(uint8_t i=0; i< 6; i++) {
        //   printf("%02x", evt->data.evt_scanner_scan_report.address.addr[i]);
        // }

        process_scan_response(&(evt->data.evt_scanner_scan_report));

        if (Connectable_Devices_Counter >= MAX_NUMBER_OF_CONNECTABLE_DEVICES)
        {
          sl_bt_scanner_stop();
          printf("Final count on Connectable devices %d\n\r",Connectable_Devices_Counter+1);
          printf("List of Connectable devices\n\r");

          for (uint8_t i =0 ; i<MAX_NUMBER_OF_CONNECTABLE_DEVICES-1; i++)
          {
            printf("%2x:%2x:%2x:%2x:%2x:%2x \n\r",Connectable_Devices_Array[i].address.addr[5],Connectable_Devices_Array[i].address.addr[4],Connectable_Devices_Array[i].address.addr[3],Connectable_Devices_Array[i].address.addr[2],Connectable_Devices_Array[i].address.addr[1],Connectable_Devices_Array[i].address.addr[0]);
          }
        }


        // if (process_scan_response(&(evt->data.evt_scanner_scan_report))==1)
        // {
        //   if (Connectable_Devices_Counter < 32)
        //   {
        //     Connectable_Devices_Array[Connectable_Devices_Counter].address = evt->data.evt_scanner_scan_report.address;
        //     Connectable_Devices_Array[Connectable_Devices_Counter].address_type = evt->data.evt_scanner_scan_report.address_type;
        //     printf("Connectable devices found %d",Connectable_Devices_Counter);
        //     Connectable_Devices_Counter++;
        //   }
        // }

            //
            // sc = sl_bt_connection_open(evt->data.evt_scanner_scan_report.address,
            //                            evt->data.evt_scanner_scan_report.address_type,
            //                            gap_1m_phy, &conn_handle);
            // app_assert(sc == SL_STATUS_OK || sc == SL_STATUS_INVALID_STATE,
            //            "[E: 0x%04x] Failed to open connection\n",
            //            (int)sc);


        //}
      }
        break;

    case sl_bt_evt_system_soft_timer_id:
      sl_bt_scanner_stop();

      break;
    // -------------------------------
    // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:
      app_log("Connection opened\n");
      break;

    // -------------------------------
    // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:
      app_log("Connection closed\n");
      // Restart advertising after client has disconnected.
      sc = sl_bt_advertiser_start(
        advertising_set_handle,
        advertiser_general_discoverable,
        advertiser_connectable_scannable);
      app_assert(sc == SL_STATUS_OK,
                 "[E: 0x%04x] Failed to start advertising\n",
                 (int)sc);
      app_log("Started advertising\n");
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
