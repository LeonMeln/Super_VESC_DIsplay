#include "ble_vesc_driver.h"
#include "ble_config.h"
#include "ble_system.h"  // For centralized BLE system
#include "comm_can.h"
#include "packet_parser.h"
#include "debug_log.h"
#include "dev_settings.h"
#include "settings_ble_commands.h"
#include "vesc_rt_data.h"
#include "datatypes.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// BLE Configuration variables
int MTU_SIZE = 23;
int PACKET_SIZE = MTU_SIZE - 3;
bool deviceConnected = false;
bool oldDeviceConnected = false;
bool bleNotificationsSubscribed = false;

// BLE Objects
static NimBLEServer *pServer = nullptr;
static NimBLEService *pServiceVesc = nullptr;
static NimBLECharacteristic *pCharacteristicVescTx = nullptr;
static NimBLECharacteristic *pCharacteristicVescRx = nullptr;
static SemaphoreHandle_t ble_send_mutex;

// Buffer for BLE data (will be used for CAN communication later)
static std::string vescBuffer;
static std::string updateBuffer;

// Packet parser for framed VESC protocol
static packet_parser_t ble_packet_parser;

// FIFO Command Queue
#define BLE_QUEUE_SIZE 10
static QueueHandle_t ble_command_queue = NULL;

// Connection state callbacks (called from general BLE system callbacks)
void vesc_ble_driver_on_connect(void)
{
  deviceConnected = true;
}

void vesc_ble_driver_on_disconnect(void)
{
  deviceConnected = false;
  bleNotificationsSubscribed = false; // Reset subscription flag on disconnect
}

void vesc_ble_driver_on_mtu_change(uint16_t MTU)
{
  // Update VESC-specific MTU settings
  LOG_INFO(BLE_UART, "🔵 Packet size adjusted to %d bytes", MTU - 3);
  MTU_SIZE = MTU;
  PACKET_SIZE = MTU_SIZE - 3;
}

// Track if we're waiting for a CAN response to forward to BLE (used in Bridge mode)
static bool waiting_for_can_response = false;
static uint8_t expected_response_vesc_id = 255;
static unsigned long last_command_time = 0;
#define COMMAND_TIMEOUT_MS 10000  // 10 seconds for long operations like hall detection

void ble_vesc_send_frame_response_to_ble(uint8_t* data, unsigned int len) {
  if (!deviceConnected || !pCharacteristicVescTx) {
    LOG_WARN(BLE_UART, "Cannot send response - not connected");
    return;
  }
  // Wait for previous notification to complete before sending new one
  xSemaphoreTake(ble_send_mutex, 200/portTICK_PERIOD_MS);
  LOG_HEX_INFO(BLE_HEX, data, len, "BLE TX:");
  pCharacteristicVescTx->setValue(data, len);
  pCharacteristicVescTx->notify();
  //Serial.println("send frame");
}

// Packet processed callback - called when valid packet is parsed from BLE
void BLE_OnPacketParsed(uint8_t* data, uint16_t len) {
  LOG_DEBUG(BLE_UART, "📦 Parsed complete packet (%d bytes)", len);
  
  if (len < 1) {
    return;
  }
  
  // ============================================================================
  // BLE-CAN Bridge Mode (like vesc_express)
  // Packet parser has already extracted clean payload (without 02/03 framing)
  // Just forward the entire payload to CAN bus
  // ============================================================================
  
  // The data[] is already parsed payload from BLE packet:
  // Example: BLE receives "02 01 11 02 10 03"
  //          Parser extracts: data[0]=0x11 (payload), len=1
  //          0x11 = 17 = CAN_PACKET_PING
  
  // Log command type for debugging
  const char* cmd_name = "UNKNOWN";
  switch (data[0]) {
    case 0: cmd_name = "FW_VERSION"; break;
    case 4: cmd_name = "GET_VALUES"; break;
    case 28: cmd_name = "DETECT_HALL_FOC"; break;
    case 24: cmd_name = "DETECT_MOTOR_PARAM"; break;
    case 25: cmd_name = "DETECT_MOTOR_R_L"; break;
    case 27: cmd_name = "DETECT_ENCODER"; break;
  }
  LOG_INFO(BLE_UART, "📦 BLE→CAN: Command 0x%02X (%s), len=%d", data[0], cmd_name, len);
  LOG_HEX_VERBOSE(BLE_UART, data, len, "");
  
  // Forward entire payload to CAN bus
  // comm_can_send_buffer handles:
  // - Fragmentation (FILL_RX_BUFFER if >6 bytes)
  // - CRC calculation
  // - PROCESS_RX_BUFFER / PROCESS_SHORT_BUFFER wrapping
  
  waiting_for_can_response = true;
  expected_response_vesc_id = settings_get_target_vesc_id();
  last_command_time = millis();
  
  LOG_DEBUG(BLE_UART, "🔄 Forwarding to VESC ID %d via CAN bus", expected_response_vesc_id);
  
  // send_type = 0: commands_send (wait for response and send it back)
  comm_can_send_buffer(settings_get_target_vesc_id(), data, len, 0);
}

// BLE Characteristic Callbacks Implementation
void MyCallbacksRX::onWrite(BLECharacteristic *pCharacteristic)
{
  //LOG_VERBOSE(BLE, "onWrite to characteristics: %s", pCharacteristic->getUUID().toString().c_str());
  std::string rxValue = pCharacteristic->getValue();
  if (rxValue.length() > 0)
  {
    if (pCharacteristic->getUUID().equals(pCharacteristicVescRx->getUUID()))
    {
      LOG_HEX_VERBOSE(BLE_UART, (uint8_t*)rxValue.data(), rxValue.length(), "📥 Received bytes: ");
      LOG_HEX_INFO(BLE_HEX, (uint8_t*)rxValue.data(), rxValue.length(), "BLE RX:");
      
      // Process each byte through the packet parser
      for (size_t i = 0; i < rxValue.length(); i++) {
        bool packet_complete = packet_parser_process_byte(&ble_packet_parser, 
                                                          (uint8_t)rxValue[i], 
                                                          BLE_OnPacketParsed);
        
        if (packet_complete) {
          LOG_DEBUG(BLE_UART, "✅ Packet successfully parsed and processed");
        }
      }
    }
  }
}

void MyCallbacksTX::onNotify(NimBLECharacteristic* pCharacteristic)
{
  // onNotify is called BEFORE notification is sent
  //Serial.println("onNotify called");
  //LOG_DEBUG(BLE_UART, "📤 onNotify: About to send notification");
  xSemaphoreGive(ble_send_mutex);
}

void MyCallbacksRX::onSubscribe(NimBLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc, uint16_t subValue)
{
  // subValue: 0 = unsubscribed, 1 = notify, 2 = indicate, 3 = notify+indicate
  if (pCharacteristic->getUUID().equals(pCharacteristicVescTx->getUUID()))
  {
    if (subValue > 0) {
      bleNotificationsSubscribed = true;
      LOG_INFO(BLE_UART, "🔔 Client subscribed to notifications - device RT data requests paused");
    } else {
      bleNotificationsSubscribed = false;
      LOG_INFO(BLE_UART, "🔕 Client unsubscribed from notifications - device RT data requests resumed");
    }
  }
}

// Initialize BLE Server (now accepts server as parameter)
bool vesc_ble_driver_init(NimBLEServer* pServer) {
  if (pServer == nullptr) {
    LOG_ERROR(BLE_UART, "BLE server is null");
    return false;
  }

  ble_send_mutex = xSemaphoreCreateBinary();
  if (ble_send_mutex == NULL) {
    LOG_ERROR(BLE_UART, "Failed to create send mutex");
    return false;
  }
  // Binary semaphore starts in "taken" state (0), so give it once to make it available
  xSemaphoreGive(ble_send_mutex);

  try {
    // Store server reference
    ::pServer = pServer;
    
    // Register VESC driver callbacks with BLE system
    ble_system_register_connect_callback(vesc_ble_driver_on_connect);
    ble_system_register_disconnect_callback(vesc_ble_driver_on_disconnect);
    ble_system_register_mtu_change_callback(vesc_ble_driver_on_mtu_change);

    // Create the BLE Service
    BLEService *pService = pServer->createService(VESC_SERVICE_UUID);

    // Create a BLE TX Characteristic
    pCharacteristicVescTx = pService->createCharacteristic(
        VESC_CHARACTERISTIC_UUID_TX,
        NIMBLE_PROPERTY::NOTIFY |
            NIMBLE_PROPERTY::READ);

    // Create a BLE RX Characteristic
    pCharacteristicVescRx = pService->createCharacteristic(
        VESC_CHARACTERISTIC_UUID_RX,
        NIMBLE_PROPERTY::WRITE |
            NIMBLE_PROPERTY::WRITE_NR);

    pCharacteristicVescRx->setCallbacks(new MyCallbacksRX());
    pCharacteristicVescTx->setCallbacks(new MyCallbacksTX());

    // Start the VESC service
    pService->start();

    // Add service UUID to advertising (but don't start advertising yet)
    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(VESC_SERVICE_UUID);
    
    LOG_INFO(BLE_UART, "🔵 VESC service initialized - characteristics added");
    
    // Initialize command queue
    if (!BLE_InitCommandQueue()) {
      return false;
    }
    
    // Initialize packet parser
    packet_parser_init(&ble_packet_parser);
    LOG_INFO(BLE_UART, "🔵 Packet parser initialized");
    
    return true;
  }
  catch (const std::exception& e) {
    LOG_ERROR(BLE_UART, "Initialization failed: %s", e.what());
    return false;
  }
}

// Check if BLE is connected
bool BLE_IsConnected() {
  return deviceConnected;
}

bool BLE_IsSubscribed() {
  return bleNotificationsSubscribed;
}

// Process CAN command received from BLE
bool BLE_ProcessCANCommand(uint8_t* data, uint8_t len) {
  if (len < sizeof(ble_can_message_t)) {
    LOG_ERROR(BLE_UART, "Invalid CAN message size: %d", len);
    return false;
  }
  
  ble_can_message_t* can_msg = (ble_can_message_t*)data;
  
  LOG_DEBUG(BLE_UART, "📥 Received CAN command ID=0x%03X, Len=%d", can_msg->can_id, can_msg->data_length);
  LOG_HEX_VERBOSE(BLE_UART, can_msg->data, can_msg->data_length, "Data: ");
  
  // Forward to VESC via CAN
  BLE_ForwardCANToVESC(can_msg->data, can_msg->data_length);
  return true;
}

// Forward CAN data to VESC (using existing CAN infrastructure)
void BLE_ForwardCANToVESC(uint8_t* can_data, uint8_t len) {
  if (len < 4) {
    LOG_WARN(BLE_UART, "BLE->CAN: Invalid data length %d", len);
    return;
  }
  
  // Parse as real VESC CAN packet format
  // First 4 bytes: little-endian int32 value
  int32_t value = (can_data[3] << 24) | (can_data[2] << 16) | (can_data[1] << 8) | can_data[0];
  
  // Check if we have a CAN ID in the message (extended format)
  if (len >= 8) {
    // Extended format: [CAN_ID:4][DATA:4]
    uint32_t can_id = (can_data[7] << 24) | (can_data[6] << 16) | (can_data[5] << 8) | can_data[4];
    uint8_t packet_type = (can_id >> 8) & 0xFF;
    uint8_t vesc_id = can_id & 0xFF;
    
    LOG_DEBUG(BLE_UART, "🎯 BLE->CAN: Raw packet Type=0x%02X, VESC=%d, Value=%d", packet_type, vesc_id, value);
    
    // Send raw CAN message
    uint8_t raw_data[4];
    raw_data[0] = can_data[0];
    raw_data[1] = can_data[1]; 
    raw_data[2] = can_data[2];
    raw_data[3] = can_data[3];
    
    comm_can_transmit_eid(can_id, raw_data, 4);
  } else {
    // Simple format: interpret first byte as command type
    switch (can_data[0]) {
      case 0x00: // Set duty cycle (CAN_PACKET_SET_DUTY)
        {
          float duty = (float)value / 100000.0f; // VESC duty scaling
          LOG_DEBUG(BLE_UART, "🎯 BLE->CAN: Setting duty %.3f", duty);
          comm_can_set_duty(255, duty);
        }
        break;
        
      case 0x01: // Set current (CAN_PACKET_SET_CURRENT)
        {
          float current = (float)value / 1000.0f; // VESC current scaling
          LOG_DEBUG(BLE_UART, "🎯 BLE->CAN: Setting current %.2fA", current);
          comm_can_set_current(255, current);
        }
        break;
        
      case 0x02: // Set brake current (CAN_PACKET_SET_CURRENT_BRAKE)
        {
          float current = (float)value / 1000.0f;
          LOG_DEBUG(BLE_UART, "🎯 BLE->CAN: Setting brake current %.2fA", current);
          comm_can_set_current_brake(255, current);
        }
        break;
        
      case 0x03: // Set RPM (CAN_PACKET_SET_RPM)
        {
          float rpm = (float)value;
          LOG_DEBUG(BLE_UART, "🎯 BLE->CAN: Setting RPM %.0f", rpm);
          comm_can_set_rpm(255, rpm);
        }
        break;
        
      case 0x11: // Ping (CAN_PACKET_PING)
        LOG_DEBUG(BLE_UART, "🎯 BLE->CAN: Sending ping");
        {
          HW_TYPE hw_type;
          comm_can_ping(255, &hw_type);
        }
        break;
        
      case 0xFF: // Request status (custom command)
        LOG_DEBUG(BLE_UART, "🎯 BLE->CAN: Requesting VESC status");
        // Status is received automatically from VESC STATUS packets
        break;
        
      default:
        LOG_WARN(BLE_UART, "BLE->CAN: Unknown command type 0x%02X", can_data[0]);
        break;
    }
  }
}

// Main BLE processing loop
void vesc_ble_driver_loop() {
  // Process command queue (FIFO)
  BLE_ProcessCommandQueue();


  // Handle BLE connection state changes
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);                  // give the bluetooth stack the chance to get things ready
    ble_system_restart_advertising(); // restart advertising using centralized function
    //Serial.println("[%lu] 🔵 BLE: Restarted advertising", millis());
    oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
    //Serial.println("[%lu] 🔵 BLE: Client connected and ready", millis());
    oldDeviceConnected = deviceConnected;
  }
}

// Initialize FIFO command queue
bool BLE_InitCommandQueue() {
  ble_command_queue = xQueueCreate(BLE_QUEUE_SIZE, sizeof(ble_command_t));
  if (ble_command_queue == NULL) {
    LOG_ERROR(BLE_UART, "Failed to create command queue");
    return false;
  }
  //LOG_INFO(BLE, "Command queue initialized (size: %d)", BLE_QUEUE_SIZE);
  return true;
}

// Queue a command for processing in main loop
bool BLE_QueueCommand(uint8_t* data, uint16_t length, uint8_t target_vesc_id, uint8_t send_type) {
  if (ble_command_queue == NULL) {
    LOG_ERROR(BLE_UART, "Command queue not initialized");
    return false;
  }
  
  if (length > BLE_CMD_BUFFER_SIZE) {
    LOG_ERROR(BLE_UART, "Command too large (%d > %d bytes)", length, BLE_CMD_BUFFER_SIZE);
    return false;
  }
  
  ble_command_t cmd;
  memcpy(cmd.data, data, length);
  cmd.length = length;
  cmd.target_vesc_id = target_vesc_id;
  cmd.send_type = send_type;
  
  BaseType_t result = xQueueSend(ble_command_queue, &cmd, 0); // Non-blocking
  if (result != pdTRUE) {
    LOG_WARN(BLE_UART, "Command queue full, dropping command");
    return false;
  }
  
  //LOG_VERBOSE(BLE, "Queued command (%d bytes) for VESC %d", length, target_vesc_id);
  return true;
}

// Process commands from FIFO queue (called from main loop)
void BLE_ProcessCommandQueue() {
  if (ble_command_queue == NULL) {
    return;
  }
  
  ble_command_t cmd;
  while (xQueueReceive(ble_command_queue, &cmd, 0) == pdTRUE) { // Non-blocking
    //Serial.printf("[%lu] 🔄 BLE: Processing queued command (%d bytes) to VESC %d\n", 
    //              millis(), cmd.length, cmd.target_vesc_id);
    
    // Send command using VESC fragmentation protocol
    comm_can_send_buffer(cmd.target_vesc_id, cmd.data, cmd.length, cmd.send_type);
    
    //Serial.printf("[%lu] ✅ BLE: Sent queued command (%d bytes) to VESC %d\n", 
    //              millis(), cmd.length, cmd.target_vesc_id);
  }
}

// Send framed response via BLE 
void ble_vesc_send_frame_resppnse(uint8_t* data, unsigned int len) {
  if (!deviceConnected || !pCharacteristicVescTx) {
    LOG_WARN(BLE_UART, "Cannot send response - not connected");
    return;
  }
  
  // Build framed packet
  uint8_t framed_buffer[600]; // payload + framing overhead
  uint16_t framed_len = packet_build_frame(data, len, framed_buffer, sizeof(framed_buffer));
  
  if (framed_len == 0) {
    LOG_ERROR(BLE_UART, "Failed to build framed packet");
    return;
  }
  
  // Send via BLE
  if (framed_len <= PACKET_SIZE) {
    ble_vesc_send_frame_response_to_ble(framed_buffer, framed_len);
    
    LOG_DEBUG(BLE_UART, "📤 BLE←Local: Sent framed response (%d bytes total, %d payload)", framed_len, len);
    LOG_HEX_VERBOSE(BLE_UART, framed_buffer, framed_len, "");
  } else {
    // Need to fragment the response
    LOG_WARN(BLE_UART, "BLE←Local: Response too large (%d > %d), fragmenting...", framed_len, PACKET_SIZE);
    
    int offset = 0;
    while (offset < framed_len) {
      int chunk_size = (framed_len - offset > PACKET_SIZE) ? PACKET_SIZE : (framed_len - offset);
      
      ble_vesc_send_frame_response_to_ble(framed_buffer + offset, chunk_size);
      
      LOG_DEBUG(BLE_UART, "📤 BLE←Local: Sent fragment %d bytes (offset %d)", chunk_size, offset);
      offset += chunk_size;
      
      // Small delay between fragments
      delay(10);
    }
  }
}

// CAN response handler - called when CAN response is received (Bridge mode only)
void BLE_OnCANResponse(uint8_t* data, unsigned int len) {
  // Check timeout
  if (waiting_for_can_response && (millis() - last_command_time > COMMAND_TIMEOUT_MS)) {
    LOG_WARN(BLE_UART, "CAN response timeout, resetting flag");
    waiting_for_can_response = false;
  }
  
  // Only forward if BLE is connected (removed waiting_for_can_response check for better compatibility)
  if (!deviceConnected || !pCharacteristicVescTx) {
    return;
  }
  vesc_rt_data_set_rx_time();
  
  // Log response command type
  const char* resp_name = "UNKNOWN";
  if (len > 0) {
    switch (data[0]) {
      case 0: resp_name = "FW_VERSION"; break;
      case 4: resp_name = "GET_VALUES"; break;
      case 28: resp_name = "DETECT_HALL_FOC"; break;
      case 24: resp_name = "DETECT_MOTOR_PARAM"; break;
      case 25: resp_name = "DETECT_MOTOR_R_L"; break;
      case 27: resp_name = "DETECT_ENCODER"; break;
    }
  }
  LOG_INFO(BLE_UART, "📦 CAN→BLE: Response 0x%02X (%s), len=%d, forwarding to BLE", 
           len > 0 ? data[0] : 0, resp_name, len);
  LOG_HEX_VERBOSE(BLE_UART, data, len > 16 ? 16 : len, "");
  
  // Forward the response to BLE with framing
  waiting_for_can_response = false; // Reset flag
  
  // Build framed packet
  uint8_t framed_buffer[600];
  uint16_t framed_len = packet_build_frame(data, len, framed_buffer, sizeof(framed_buffer));
  
  if (framed_len == 0) {
    LOG_ERROR(BLE_UART, "BLE←CAN: Failed to build framed packet");
    return;
  }
  
  // Send via BLE
  if (framed_len <= PACKET_SIZE) {
    ble_vesc_send_frame_response_to_ble(framed_buffer, framed_len);
    
    LOG_DEBUG(BLE_UART, "📤 BLE←CAN: Sent framed response (%d bytes total, %d payload)", framed_len, len);
    LOG_HEX_VERBOSE(BLE_UART, framed_buffer, framed_len, "");
  } else {
    // Need to fragment the response
    LOG_WARN(BLE_UART, "BLE←CAN: Response too large (%d > %d), fragmenting...", framed_len, PACKET_SIZE);
    
    int offset = 0;
    while (offset < framed_len) {
      int chunk_size = (framed_len - offset > PACKET_SIZE) ? PACKET_SIZE : (framed_len - offset);
      
      ble_vesc_send_frame_response_to_ble(framed_buffer + offset, chunk_size);
      
      LOG_DEBUG(BLE_UART, "📤 BLE←CAN: Sent fragment %d bytes (offset %d)", chunk_size, offset);
      offset += chunk_size;
      
      // Small delay between fragments
      delay(10);
    }
  }
}
