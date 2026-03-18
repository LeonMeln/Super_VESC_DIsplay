#ifndef DEBUG_LOG_H
#define DEBUG_LOG_H

#include <Arduino.h>

// ============================================================================
// LOGGING CONFIGURATION
// ============================================================================

// Master logging switch - set to 0 to disable ALL logging
#define DEBUG_LOGGING_ENABLED 1

// Individual log level controls (comment out or set to 0 to disable)
// Using SVESC_ prefix to avoid conflicts with NimBLE's LOG_LEVEL_* macros
#define SVESC_LOG_LEVEL_ERROR   1  // Critical errors
#define SVESC_LOG_LEVEL_WARN    1  // Warnings
#define SVESC_LOG_LEVEL_INFO    1  // Important information
#define SVESC_LOG_LEVEL_DEBUG   1  // Debug information
#define SVESC_LOG_LEVEL_VERBOSE 0  // Verbose/detailed logs (disabled by default)

// Module-specific logging (set to 0 to disable specific modules)
// BLE-UART: Communication channel between phone and ESP32 (Nordic UART Service)
#define LOG_BLE_UART_ENABLED  0  // BLE-UART debugging enabled (phone ↔ ESP32)
#define LOG_BLE_HEX_ENABLED  0  // BLE-UART hex debugging enabled (phone ↔ ESP32)

// CAN Bus: Communication channel between ESP32 and VESC controller
#define LOG_CAN_BUS_ENABLED   0  // CAN bus debugging enabled (ESP32 ↔ VESC)
#define LOG_CAN_HEX_ENABLED  0  // CAN bus hex debugging enabled (ESP32 ↔ VESC)

// Legacy/other modules (disabled)
#define LOG_CAN_ENABLED     0  // Legacy CAN debugging (use LOG_CAN_BUS_ENABLED instead)
#define LOG_BLE_ENABLED     0  // Legacy BLE debugging (use LOG_BLE_UART_ENABLED instead)
#define LOG_PACKET_ENABLED  0  // Packet debugging disabled
#define LOG_VESC_ENABLED    0  // VESC debugging disabled
#define LOG_SYSTEM_ENABLED  0  // System debugging disabled
#define LOG_UI_ENABLED      0  // UI debugging disabled
#define LOG_LIMITS_ENABLED  0  // Limits debugging disabled
#define LOG_BLE_CLIENT_ENABLED 0  // BLE client debugging disabled
#define LOG_BLE_KEYBOARD_ENABLED 0  // BLE keyboard debugging disabled
#define LOG_MEDIA_ENABLED   0  // Media debugging disabled
#define LOG_OTA_ENABLED     0  // OTA debugging disabled
#define LOG_MUSIC_ENABLED   0  // Music debugging disabled
#define LOG_LISP_POLL_ENABLED 1  // LISP poll debugging enabled

// ============================================================================
// LOGGING MACROS - DO NOT MODIFY BELOW THIS LINE
// ============================================================================

#if DEBUG_LOGGING_ENABLED

  // Helper macro to check if logging is enabled for a module and level
  #define LOG_ENABLED(module, level) (LOG_##module##_ENABLED && SVESC_LOG_LEVEL_##level)

  // Error logs (always with ❌ emoji)
  #if SVESC_LOG_LEVEL_ERROR
    #define LOG_ERROR(module, ...) \
      if (LOG_##module##_ENABLED) { \
        Serial.printf("[%lu] ❌ [%s] ", millis(), #module); \
        Serial.printf(__VA_ARGS__); \
        Serial.println(); \
      }
  #else
    #define LOG_ERROR(module, ...)
  #endif

  // Warning logs (with ⚠️ emoji)
  #if SVESC_LOG_LEVEL_WARN
    #define LOG_WARN(module, ...) \
      if (LOG_##module##_ENABLED) { \
        Serial.printf("[%lu] ⚠️  [%s] ", millis(), #module); \
        Serial.printf(__VA_ARGS__); \
        Serial.println(); \
      }
  #else
    #define LOG_WARN(module, ...)
  #endif

  // Info logs (with ✅ or 📋 emoji)
  #if SVESC_LOG_LEVEL_INFO
    #define LOG_INFO(module, ...) \
      if (LOG_##module##_ENABLED) { \
        Serial.printf("[%lu] ✅ [%s] ", millis(), #module); \
        Serial.printf(__VA_ARGS__); \
        Serial.println(); \
      }
  #else
    #define LOG_INFO(module, ...)
  #endif

  // Debug logs (with 🔍 emoji)
  #if SVESC_LOG_LEVEL_DEBUG
    #define LOG_DEBUG(module, ...) \
      if (LOG_##module##_ENABLED) { \
        Serial.printf("[%lu] 🔍 [%s] ", millis(), #module); \
        Serial.printf(__VA_ARGS__); \
        Serial.println(); \
      }
  #else
    #define LOG_DEBUG(module, ...)
  #endif

  // Verbose logs (with 📝 emoji)
  #if SVESC_LOG_LEVEL_VERBOSE
    #define LOG_VERBOSE(module, ...) \
      if (LOG_##module##_ENABLED) { \
        Serial.printf("[%lu] 📝 [%s] ", millis(), #module); \
        Serial.printf(__VA_ARGS__); \
        Serial.println(); \
      }
  #else
    #define LOG_VERBOSE(module, ...)
  #endif

  // Special macro for data dumps (hex output)
  #define LOG_HEX(module, data, len, prefix) \
    if (LOG_##module##_ENABLED && SVESC_LOG_LEVEL_VERBOSE) { \
      Serial.printf("[%lu] 📊 [%s] %s", millis(), #module, prefix); \
      for (int i = 0; i < len && i < 16; i++) { \
        Serial.printf("%02X ", (unsigned int)((data)[i])); \
      } \
      if (len > 16) Serial.printf("... (%d more)", len - 16); \
      Serial.println(); \
    }

    
  // Special macro for data dumps (hex output)
  #define LOG_HEX_VERBOSE(module, data, len, prefix) \
  if (LOG_##module##_ENABLED && SVESC_LOG_LEVEL_VERBOSE) { \
    Serial.printf("[%lu] 📊 [%s] %s", millis(), #module, prefix); \
    for (int i = 0; i < len && i < 16; i++) { \
      Serial.printf("%02X ", (unsigned int)((data)[i])); \
    } \
    if (len > 16) Serial.printf("... (%d more)", len - 16); \
    Serial.println(); \
  }

  // Special macro for data dumps (hex output)
  #define LOG_HEX_DEBUG(module, data, len, prefix) \
    if (LOG_##module##_ENABLED && SVESC_LOG_LEVEL_DEBUG) { \
      Serial.printf("[%lu] 📊 [%s] %s", millis(), #module, prefix); \
      for (int i = 0; i < len && i < 16; i++) { \
        Serial.printf("%02X ", (unsigned int)((data)[i])); \
      } \
      if (len > 16) Serial.printf("... (%d more)", len - 16); \
      Serial.println(); \
    }

  // Special macro for data dumps (hex output)
  #define LOG_HEX_INFO(module, data, len, prefix) \
  if (LOG_##module##_ENABLED && SVESC_LOG_LEVEL_INFO) { \
    Serial.printf("[%lu] 📊 [%s] %s", millis(), #module, prefix); \
    for (int i = 0; i < len && i < 16; i++) { \
      Serial.printf("%02X ", (unsigned int)((data)[i])); \
    } \
    if (len > 16) Serial.printf("... (%d more)", len - 16); \
    Serial.println(); \
  }

  // Special macro for data dumps (hex output)
  #define LOG_HEX_WARN(module, data, len, prefix) \
  if (LOG_##module##_ENABLED && SVESC_LOG_LEVEL_WARN) { \
    Serial.printf("[%lu] 📊 [%s] %s", millis(), #module, prefix); \
    for (int i = 0; i < len && i < 16; i++) { \
      Serial.printf("%02X ", (unsigned int)((data)[i])); \
    } \
    if (len > 16) Serial.printf("... (%d more)", len - 16); \
    Serial.println(); \
  }
  // Raw printf without module tag (for custom formatting)
  #define LOG_RAW(...) Serial.printf(__VA_ARGS__)

#else
  // All logging disabled
  #define LOG_ERROR(module, ...)
  #define LOG_WARN(module, ...)
  #define LOG_INFO(module, ...)
  #define LOG_DEBUG(module, ...)
  #define LOG_VERBOSE(module, ...)
  #define LOG_HEX(module, data, len, prefix)
  #define LOG_RAW(...)
#endif

// Legacy compatibility macro (if you want to keep some direct Serial.printf)
#define LOG_PRINT(...) Serial.printf(__VA_ARGS__)

#endif // DEBUG_LOG_H

