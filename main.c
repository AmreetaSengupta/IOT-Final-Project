/***************************************************************************//**
 * @file
 * @brief Silicon Labs Bluetooth mesh light switch example
 * This example implements a Bluetooth mesh light switch.
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

/* C Standard Library headers */
#include <stdlib.h>
#include <stdio.h>

/* Board headers */
#include "init_mcu.h"
#include "init_board.h"
#include "init_app.h"
#include "ble-configuration.h"
#include "board_features.h"
#include "retargetserial.h"

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"
#include <gecko_configuration.h>
#include "mesh_generic_model_capi_types.h"
#include "mesh_lighting_model_capi_types.h"
#include "mesh_lib.h"
#include <mesh_sizes.h>

/* Libraries containing default Gecko configuration values */
#include "em_emu.h"
#include "em_cmu.h"
#include <em_gpio.h>
#include <em_rtcc.h>
#include <gpiointerrupt.h>

/* Coex header */
#include "coexistence-ble.h"

/* Device initialization header */
#include "hal-config.h"

/* Display Interface header */
#include "display_interface.h"

/* Other headers */
#include "src/gpio.h"

#if defined(HAL_CONFIG)
#include "bsphalconfig.h"
#else
#include "bspconfig.h"
#endif

/***********************************************************************************************//**
 * @addtogroup Application
 * @{
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup app
 * @{
 **************************************************************************************************/

/// Maximum number of simultaneous Bluetooth connections
#define MAX_CONNECTIONS 2

/// Heap for Bluetooth stack
uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS) + BTMESH_HEAP_SIZE + 1760];

bool mesh_bgapi_listener(struct gecko_cmd_packet *evt);

/// Bluetooth advertisement set configuration
///
/// At minimum the following is required:
/// * One advertisement set for Bluetooth LE stack (handle number 0)
/// * One advertisement set for Mesh data (handle number 1)
/// * One advertisement set for Mesh unprovisioned beacons (handle number 2)
/// * One advertisement set for Mesh unprovisioned URI (handle number 3)
/// * N advertisement sets for Mesh GATT service advertisements
/// (one for each network key, handle numbers 4 .. N+3)
///
#define MAX_ADVERTISERS (4 + MESH_CFG_MAX_NETKEYS)

/// Priorities for bluetooth link layer operations
static gecko_bluetooth_ll_priorities linklayer_priorities = GECKO_BLUETOOTH_PRIORITIES_DEFAULT;

/// Bluetooth stack configuration
const gecko_configuration_t config =
{
#if defined(FEATURE_LFXO)
// Disable sleep on the following boards to make buttons's interrupts work.
// If you enable sleep on these boards, WSTK pushbuttons will not wake up chip.
// Only Ports A and B support EM2 wake-up. Please refer to EFR32xG21 reference manual GPIO chapter.
#if defined(BRD4180A) || defined(BRD4181A)
  .sleep.flags = 0,
#else
  .sleep.flags = SLEEP_FLAGS_DEEP_SLEEP_ENABLE,
#endif
#else
  .sleep.flags = 0,
#endif // LFXO
  .bluetooth.max_connections = MAX_CONNECTIONS,
  .bluetooth.max_advertisers = MAX_ADVERTISERS,
  .bluetooth.heap = bluetooth_stack_heap,
  .bluetooth.heap_size = sizeof(bluetooth_stack_heap) - BTMESH_HEAP_SIZE,
  .bluetooth.sleep_clock_accuracy = 100,
  .bluetooth.linklayer_priorities = &linklayer_priorities,
  .gattdb = &bg_gattdb_data,
  .btmesh_heap_size = BTMESH_HEAP_SIZE,
  .pa.config_enable = 1, // Set this to be a valid PA config
#if defined(FEATURE_PA_INPUT_FROM_VBAT)
  .pa.input = GECKO_RADIO_PA_INPUT_VBAT, // Configure PA input to VBAT
#else
  .pa.input = GECKO_RADIO_PA_INPUT_DCDC,
#endif // defined(FEATURE_PA_INPUT_FROM_VBAT)
  .max_timers = 16,
  .rf.flags = GECKO_RF_CONFIG_ANTENNA,   // Enable antenna configuration.
  .rf.antenna = GECKO_RF_ANTENNA,   // Select antenna path!
};

/// Flag for indicating DFU Reset must be performed
uint8_t boot_to_dfu = 0;

/// Timer Frequency used
#define TIMER_CLK_FREQ ((uint32_t)32768)
/// Convert miliseconds to timer ticks
#define TIMER_MS_2_TIMERTICK(ms) ((TIMER_CLK_FREQ * ms) / 1000)
/** Stop timer. */
#define TIMER_STOP 0

/*******************************************************************************
 * Timer handles defines.
 ******************************************************************************/
#define TIMER_ID_RESTART            78
#define TIMER_ID_FACTORY_RESET      77
#define TIMER_ID_PROVISIONING       66
#define TIMER_ID_RETRANS_ONOFF      10
#define TIMER_ID_RETRANS_LIGHTNESS  11
#define TIMER_ID_RETRANS_CTL        12
#define TIMER_ID_RETRANS_SCENE      13
#define TIMER_ID_FRIEND_FIND        20
#define TIMER_ID_NODE_CONFIGURED    30

/// Minimum color temperature 800K
#define TEMPERATURE_MIN      0x0320
/// Maximum color temperature 20000K
#define TEMPERATURE_MAX      0x4e20
/// Delta UV is hardcoded to 0 in this example
#define DELTA_UV  0

#define IMMEDIATE          0 ///< Immediate transition time is 0 seconds
#define PUBLISH_ADDRESS    0 ///< The unused 0 address is used for publishing
#define IGNORED            0 ///< Parameter ignored for publishing
#define NO_FLAGS           0 ///< No flags used for message

/*******************************************************************************
 * Global variables
 ******************************************************************************/
/// For indexing elements of the node (this example has only one element)
static uint16_t _elem_index = 0xffff;
/// Address of the Primary Element of the Node
static uint16_t _my_address = 0;
/// number of active Bluetooth connections
static uint8_t num_connections = 0;
/// handle of the last opened LE connection
static uint8_t conn_handle = 0xFF;
/// Flag for indicating that lpn feature is active
static uint8_t lpn_active = 0;

void lpn_init(void)
{
  uint16_t result;

  // Do not initialize LPN if lpn is currently active
  // or any GATT connection is opened
  if (lpn_active || num_connections) {
    return;
  }

  // Initialize LPN functionality.
  result = gecko_cmd_mesh_lpn_init()->result;
  if (result) {
    printf("LPN init failed (0x%x)\r\n", result);
    return;
  }
  lpn_active = 1;
  printf("LPN initialized\r\n");
  DI_Print("LPN on", DI_ROW_LPN);

  // Configure LPN Minimum friend queue length = 2
  result = gecko_cmd_mesh_lpn_config(mesh_lpn_queue_length, 2)->result;
  if (result) {
    printf("LPN queue configuration failed (0x%x)\r\n", result);
    return;
  }
  // Configure LPN Poll timeout = 5 seconds
  result = gecko_cmd_mesh_lpn_config(mesh_lpn_poll_timeout, 5 * 1000)->result;
  if (result) {
    printf("LPN Poll timeout configuration failed (0x%x)\r\n", result);
    return;
  }
  printf("trying to find friend...\r\n");
  result = gecko_cmd_mesh_lpn_establish_friendship(0)->result;

  if (result != 0) {
    printf("ret.code 0x%x\r\n", result);
  }
}

/***************************************************************************//**
 * Deinitialize LPN functionality.
 ******************************************************************************/
void lpn_deinit(void)
{
  uint16_t result;

  if (!lpn_active) {
    return; // lpn feature is currently inactive
  }

  // Cancel friend finding timer
  result = gecko_cmd_hardware_set_soft_timer(TIMER_STOP,
                                             TIMER_ID_FRIEND_FIND,
                                             1)->result;

  // Terminate friendship if exist
  result = gecko_cmd_mesh_lpn_terminate_friendship()->result;
  if (result) {
    printf("Friendship termination failed (0x%x)\r\n", result);
  }
  // turn off lpn feature
  result = gecko_cmd_mesh_lpn_deinit()->result;
  if (result) {
    printf("LPN deinit failed (0x%x)\r\n", result);
  }
  lpn_active = 0;
  printf("LPN deinitialized\r\n");
  DI_Print("LPN off", DI_ROW_LPN);
}

/***************************************************************************//**
 * Switch node initialization.
 * This is called at each boot if provisioning is already done.
 * Otherwise this function is called after provisioning is completed.
 ******************************************************************************/
void switch_node_init(void)
{
  // Initialize mesh lib, up to 8 models
  mesh_lib_init(malloc, free, 8);
}

/***************************************************************************//**
 * Handling of stack events. Both Bluetooth LE and Bluetooth mesh events
 * are handled here.
 * @param[in] evt_id  Incoming event ID.
 * @param[in] evt     Pointer to incoming event.
 ******************************************************************************/
static void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt);

/***************************************************************************//**
 * Set device name in the GATT database. A unique name is generated using
 * the two last bytes from the Bluetooth address of this device. Name is also
 * displayed on the LCD.
 *
 * @param[in] pAddr  Pointer to Bluetooth address.
 ******************************************************************************/
void set_device_name(bd_addr *pAddr)
{
  char name[20];
  uint16_t res;

  // create unique device name using the last two bytes of the Bluetooth address
  sprintf(name, "switch node %02x:%02x", pAddr->addr[1], pAddr->addr[0]);

  printf("Device name: '%s'\r\n", name);

  // write device name to the GATT database
  res = gecko_cmd_gatt_server_write_attribute_value(gattdb_device_name, 0, strlen(name), (uint8_t *)name)->result;
  if (res) {
    printf("gecko_cmd_gatt_server_write_attribute_value() failed, code 0x%x\r\n", res);
  }

  // show device name on the LCD
  DI_Print(name, DI_ROW_NAME);
}

/***************************************************************************//**
 * This function is called to initiate factory reset. Factory reset may be
 * initiated by keeping one of the WSTK pushbuttons pressed during reboot.
 * Factory reset is also performed if it is requested by the provisioner
 * (event gecko_evt_mesh_node_reset_id).
 ******************************************************************************/
void initiate_factory_reset(void)
{
  printf("factory reset\r\n");
  DI_Print("\n***\nFACTORY RESET\n***", DI_ROW_STATUS);

  /* if connection is open then close it before rebooting */
  if (conn_handle != 0xFF) {
    gecko_cmd_le_connection_close(conn_handle);
  }

  /* perform a factory reset by erasing PS storage. This removes all the keys and other settings
     that have been configured for this node */
  gecko_cmd_flash_ps_erase_all();
  // reboot after a small delay
  gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_FACTORY_RESET, 1);
}

/***************************************************************************//**
 * Main function.
 ******************************************************************************/
int main(void)
{
  // Initialize device
  initMcu();
  // Initialize board
  initBoard();
  // Initialize application
  initApp();
  initVcomEnable();

  //Initialize gpio
  gpioInit();

  //Initialize logging
  logInit();

  // Minimize advertisement latency by allowing the advertiser to always
  // interrupt the scanner.
  linklayer_priorities.scan_max = linklayer_priorities.adv_min + 1;

  gecko_stack_init(&config);
  gecko_bgapi_class_dfu_init();
  gecko_bgapi_class_system_init();
  gecko_bgapi_class_le_gap_init();
  gecko_bgapi_class_le_connection_init();
  //gecko_bgapi_class_gatt_init();
  gecko_bgapi_class_gatt_server_init();
  gecko_bgapi_class_hardware_init();
  gecko_bgapi_class_flash_init();
  gecko_bgapi_class_test_init();
  //gecko_bgapi_class_sm_init();
  gecko_bgapi_class_mesh_node_init();
  //gecko_bgapi_class_mesh_prov_init();
  gecko_bgapi_class_mesh_proxy_init();
  gecko_bgapi_class_mesh_proxy_server_init();
  //gecko_bgapi_class_mesh_proxy_client_init();
  gecko_bgapi_class_mesh_generic_client_init();
  //gecko_bgapi_class_mesh_generic_server_init();
  //gecko_bgapi_class_mesh_vendor_model_init();
  //gecko_bgapi_class_mesh_health_client_init();
  //gecko_bgapi_class_mesh_health_server_init();
  //gecko_bgapi_class_mesh_test_init();
  gecko_bgapi_class_mesh_lpn_init();
  //gecko_bgapi_class_mesh_friend_init();
  gecko_bgapi_class_mesh_scene_client_init();

  // Initialize coexistence interface. Parameters are taken from HAL config.
  gecko_initCoexHAL();

  RETARGET_SerialInit();

  // Display Interface initialization
  DI_Init();

#if defined(_SILICON_LABS_32B_SERIES_1_CONFIG_3)
  /* xG13 devices have two RTCCs, one for the stack and another for the application.
   * The clock for RTCC needs to be enabled in application code. In xG12 RTCC init
   * is handled by the stack */
  CMU_ClockEnable(cmuClock_RTCC, true);
#endif

  while (1) {
    struct gecko_cmd_packet *evt = gecko_wait_event();
    bool pass = mesh_bgapi_listener(evt);
    if (pass) {
      handle_gecko_event(BGLIB_MSG_ID(evt->header), evt);
    }
  }
}

/*******************************************************************************
 * Handling of stack events. Both Bluetooth LE and Bluetooth mesh events
 * are handled here.
 * @param[in] evt_id  Incoming event ID.
 * @param[in] evt     Pointer to incoming event.
 ******************************************************************************/
static void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt)
{
	  uint16_t result;
	  char buf[30];

	  if (NULL == evt) {
	    return;
	  }

	  switch (evt_id) {
	    case gecko_evt_system_boot_id:
	      // check pushbutton state at startup. If either PB0 or PB1 is held down then do factory reset
	    	if(GPIO_PinInGet(Button_port,Button_pin) == 0 || GPIO_PinInGet(Button_port,Button1) == 0){
	        initiate_factory_reset();
	      } else {
	        struct gecko_msg_system_get_bt_address_rsp_t *pAddr = gecko_cmd_system_get_bt_address();

	        set_device_name(&pAddr->address);

	        // Initialize Mesh stack in Node operation mode, it will generate initialized event
	        result = gecko_cmd_mesh_node_init()->result;
	        if (result) {
	          sprintf(buf, "init failed (0x%x)", result);
	          DI_Print(buf, DI_ROW_STATUS);
	        }
	      }
	      break;

	    case gecko_evt_hardware_soft_timer_id:
	      switch (evt->data.evt_hardware_soft_timer.handle) {
	        case TIMER_ID_FACTORY_RESET:
	          // reset the device to finish factory reset
	          gecko_cmd_system_reset(0);
	          break;

	        case TIMER_ID_RESTART:
	          // restart timer expires, reset the device
	          gecko_cmd_system_reset(0);
	          break;

	        case TIMER_ID_FRIEND_FIND:
	        {
	          printf("trying to find friend...\r\n");
	          result = gecko_cmd_mesh_lpn_establish_friendship(0)->result;

	          if (result != 0) {
	            printf("ret.code 0x%x\r\n", result);
	          }
	        }
	        break;

	        default:
	          break;
	      }

	      break;

	    case gecko_evt_mesh_node_initialized_id:
	      printf("node initialized\r\n");

	      // Initialize generic client models
	      result = gecko_cmd_mesh_generic_client_init()->result;
	      if (result) {
	        printf("mesh_generic_client_init failed, code 0x%x\r\n", result);
	      }

	      // Initialize scene client model
	      result = gecko_cmd_mesh_scene_client_init(0)->result;
	      if (result) {
	        printf("mesh_scene_client_init failed, code 0x%x\r\n", result);
	      }

	      struct gecko_msg_mesh_node_initialized_evt_t *pData = (struct gecko_msg_mesh_node_initialized_evt_t *)&(evt->data);

	      if (pData->provisioned) {
	        printf("node is provisioned. address:%x, ivi:%ld\r\n", pData->address, pData->ivi);

	        _my_address = pData->address;
	        _elem_index = 0;   // index of primary element is zero. This example has only one element.

	        switch_node_init();

	        // Initialize Low Power Node functionality
	        lpn_init();

	       // DI_Print("provisioned", DI_ROW_STATUS);
	      } else {
	        printf("node is unprovisioned\r\n");
	        DI_Print("unprovisioned", DI_ROW_STATUS);

	        printf("starting unprovisioned beaconing...\r\n");
	        gecko_cmd_mesh_node_start_unprov_beaconing(0x3);   // enable ADV and GATT provisioning bearer
	      }
	      break;

	    case gecko_evt_system_external_signal_id:
	    break;

	    case gecko_evt_mesh_node_provisioning_started_id:
	      printf("Started provisioning\r\n");
	      DI_Print("provisioning...", DI_ROW_STATUS);

	      // start timer for blinking LEDs to indicate which node is being provisioned
	      gecko_cmd_hardware_set_soft_timer(32768 / 4, TIMER_ID_PROVISIONING, 0);
	      break;

	    case gecko_evt_mesh_node_provisioned_id:
	      _elem_index = 0;   // index of primary element is zero. This example has only one element.
	      switch_node_init();

	      printf("node provisioned, got address=%x\r\n", evt->data.evt_mesh_node_provisioned.address);

	      DI_Print("provisioned", DI_ROW_STATUS);

	      break;

	    case gecko_evt_mesh_node_provisioning_failed_id:
	      printf("provisioning failed, code 0x%x\r\n", evt->data.evt_mesh_node_provisioning_failed.result);
	      DI_Print("prov failed", DI_ROW_STATUS);
	      /* start a one-shot timer that will trigger soft reset after small delay */
	      gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_RESTART, 1);
	      break;

	    case gecko_evt_le_connection_opened_id:
	      printf("evt:gecko_evt_le_connection_opened_id\r\n");
	      num_connections++;
	      conn_handle = evt->data.evt_le_connection_opened.connection;
	       DI_Print("connected", DI_ROW_CONNECTION);
	      // turn off lpn feature after GATT connection is opened
	      lpn_deinit();
	      break;

	    case gecko_evt_le_connection_closed_id:
	      /* Check if need to boot to dfu mode */
	      if (boot_to_dfu) {
	        /* Enter to DFU OTA mode */
	        gecko_cmd_system_reset(2);
	      }

	      printf("evt:conn closed, reason 0x%x\r\n", evt->data.evt_le_connection_closed.reason);
	      conn_handle = 0xFF;
	      if (num_connections > 0) {
	        if (--num_connections == 0) {
	         // DI_Print("", DI_ROW_CONNECTION);
	          // initialize lpn when there is no active connection
	          lpn_init();
	        }
	      }
	      break;

	    case gecko_evt_mesh_node_reset_id:
	      printf("evt gecko_evt_mesh_node_reset_id\r\n");
	      initiate_factory_reset();
	      break;

	    case gecko_evt_le_connection_parameters_id:
	      printf("connection params: interval %d, timeout %d\r\n",
	             evt->data.evt_le_connection_parameters.interval,
	             evt->data.evt_le_connection_parameters.timeout);
	      break;

	    case gecko_evt_le_gap_adv_timeout_id:
	      // these events silently discarded
	      break;

	    case gecko_evt_gatt_server_user_write_request_id:
	      if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_ota_control) {
	        /* Set flag to enter to OTA mode */
	        boot_to_dfu = 1;
	        /* Send response to Write Request */
	        gecko_cmd_gatt_server_send_user_write_response(
	          evt->data.evt_gatt_server_user_write_request.connection,
	          gattdb_ota_control,
	          bg_err_success);

	        /* Close connection to enter to DFU OTA mode */
	        gecko_cmd_le_connection_close(evt->data.evt_gatt_server_user_write_request.connection);
	      }
	      break;

	    case gecko_evt_mesh_lpn_friendship_established_id:
	      printf("friendship established\r\n");
	      DI_Print("LPN with friend", DI_ROW_LPN);
	      break;

	    case gecko_evt_mesh_lpn_friendship_failed_id:
	      printf("friendship failed\r\n");
	      DI_Print("no friend", DI_ROW_LPN);
	      break;

	    case gecko_evt_mesh_lpn_friendship_terminated_id:
	      printf("friendship terminated\r\n");
	      DI_Print("friend lost", DI_ROW_LPN);
	      if (num_connections == 0) {
	        // try again in 2 seconds
	        result = gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(2000),
	                                                   TIMER_ID_FRIEND_FIND,
	                                                   1)->result;
	        if (result) {
	          printf("timer failure?!  0x%x\r\n", result);
	        }
	      }
	      break;

	    default:
	      //printf("unhandled evt: %8.8x class %2.2x method %2.2x\r\n", evt_id, (evt_id >> 16) & 0xFF, (evt_id >> 24) & 0xFF);
	      break;
	  }
}
