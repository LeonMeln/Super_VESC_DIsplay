/*
	Copyright 2025 Super VESC Display

	VESC LISP Poll Module
	Polls LISP variables via CAN bus at 10 Hz (100ms interval)
*/

#include "vesc_lisp_poll.h"
#include "comm_can.h"
#include "buffer.h"
#include "datatypes.h"
#include "dev_settings.h"
#include <Arduino.h>
#include <string.h>
#include "debug_log.h"

#define SELECTED_VARIABLES_ENABLED 1

// Poll state
static bool lisp_poll_active = false;
static uint32_t last_poll_time = 0;
static const uint32_t POLL_INTERVAL_MS = 100; // 10 Hz

// LISP stats
static lisp_stats_t lisp_stats;
static bool stats_received = false;

void vesc_lisp_poll_init(void) {
	lisp_poll_active = false;
	last_poll_time = 0;
	memset(&lisp_stats, 0, sizeof(lisp_stats_t));
	stats_received = false;
}

void vesc_lisp_poll_start(void) {
	lisp_poll_active = true;
	last_poll_time = 0; // Force immediate poll
}

void vesc_lisp_poll_stop(void) {
	lisp_poll_active = false;
}

void vesc_lisp_poll_loop(void) {
	if (!lisp_poll_active) {
		return;
	}

	uint32_t now = millis();
	if (now - last_poll_time >= POLL_INTERVAL_MS) {
		last_poll_time = now;
		//Serial.println("poll lisp stats");
		// Send COMM_LISP_GET_STATS command with poll_all = true (1)
		uint8_t send_buffer[3];
		int32_t ind = 0;
		
		send_buffer[ind++] = COMM_LISP_GET_STATS;
		send_buffer[ind++] = 1; // poll_all = true (get all variables)

		// Send to VESC via CAN
		// send_type = 0: wait for response
		comm_can_send_buffer(settings_get_target_vesc_id(), send_buffer, ind, 0);
	}
}

void vesc_lisp_poll_process_response(unsigned char *data, unsigned int len) {
	if (len < 1) {
		return;
	}

	uint8_t cmd = data[0];
	
	// Check if this is COMM_LISP_GET_STATS response
	if (cmd != COMM_LISP_GET_STATS) {
		return;
	}

	// Debug: print raw data
	LOG_INFO(LISP_POLL, "LISP Response: len=%d, hex: ", len);
	LOG_HEX_INFO(LISP_POLL, data, len, "");
	LOG_INFO(LISP_POLL, "\n");

	// Parse LISP stats response
	// Format: [COMM_LISP_GET_STATS][cpu_use:float16][heap_use:float16][mem_use:float16][stack_use:float16][done_ctx_r:string][name1:string][value1:float32_auto][name2:string][value2:float32_auto]...
	
	int32_t ind = 1; // Skip command byte
	
	// Clear previous stats
	memset(&lisp_stats, 0, sizeof(lisp_stats_t));
	
	// Parse CPU, heap, mem, stack usage (float16 with scale 1e2)
	if (ind + 2 <= len) {
		lisp_stats.cpu_use = buffer_get_float16(data, 1e2, &ind);
	}
	if (ind + 2 <= len) {
		lisp_stats.heap_use = buffer_get_float16(data, 1e2, &ind);
	}
	if (ind + 2 <= len) {
		lisp_stats.mem_use = buffer_get_float16(data, 1e2, &ind);
	}
	if (ind + 2 <= len) {
		lisp_stats.stack_use = buffer_get_float16(data, 1e2, &ind);
	}
	
	// Parse done_ctx_r string
	if (ind < len) {
		buffer_get_string(data, len, lisp_stats.done_ctx_r, sizeof(lisp_stats.done_ctx_r), &ind);
	}
	
	// Parse variable bindings (name:value pairs)
	lisp_stats.variable_count = 0;
	while (ind < len && lisp_stats.variable_count < 32) {
		// Get variable name (string until null terminator)
		if (ind >= len) break;
		
		char var_name[64];
		int name_len = buffer_get_string(data, len, var_name, sizeof(var_name), &ind);
		if (name_len == 0) {
			// Empty string means end of data
			break;
		}
		
		// Get variable value (float32_auto = 4 bytes)
		if (ind + 4 > len) {
			LOG_WARN(LISP_POLL, "Warning: Not enough data for value at ind=%d, len=%d\n", ind, len);
			break;
		}
		
		double var_value = buffer_get_float32_auto(data, &ind);
		
		// Debug: print each variable as we parse it
		LOG_INFO(LISP_POLL, "  Parsed: '%s' = %f (ind=%d)\n", var_name, var_value, ind);
		
		// Store variable
		strncpy(lisp_stats.variables[lisp_stats.variable_count].name, var_name, sizeof(lisp_stats.variables[0].name) - 1);
		lisp_stats.variables[lisp_stats.variable_count].name[sizeof(lisp_stats.variables[0].name) - 1] = '\0';
		lisp_stats.variables[lisp_stats.variable_count].value = var_value;
		lisp_stats.variable_count++;
	}
	
	stats_received = true;
	
	#if SELECTED_VARIABLES_ENABLED == 1
	// Print stats via Serial.printf (full output)
	LOG_INFO(LISP_POLL, "\n=== LISP Stats (Poll All) ===\n");
	LOG_INFO(LISP_POLL, "CPU Use:    %.2f%%\n", lisp_stats.cpu_use);
	LOG_INFO(LISP_POLL, "Heap Use:   %.2f%%\n", lisp_stats.heap_use);
	LOG_INFO(LISP_POLL, "Mem Use:    %.2f%%\n", lisp_stats.mem_use);
	LOG_INFO(LISP_POLL, "Stack Use:  %.2f%%\n", lisp_stats.stack_use);
	LOG_INFO(LISP_POLL, "Done Ctx R: %s\n", lisp_stats.done_ctx_r);
	LOG_INFO(LISP_POLL, "Variables (%d):\n", lisp_stats.variable_count);

	for (uint8_t i = 0; i < lisp_stats.variable_count; i++) {
		LOG_INFO(LISP_POLL, "  %s = %.6f\n", 
			lisp_stats.variables[i].name,
			lisp_stats.variables[i].value);
	}
	Serial.printf("=============================\n");
	#endif

	
	// Find and print cruise-active
	int32_t cruise_active_int = 0;
	if (vesc_lisp_poll_get_variable_int("cruise-active", &cruise_active_int)) {
		LOG_INFO(LISP_POLL, "cruise-active = %d\n", cruise_active_int);
	}
	else {
		LOG_INFO(LISP_POLL, "cruise-active = NOT FOUND\n");
	}
	
	// Find and print cruise-rpm
	float cruise_rpm_float = 0.0f;
	if (vesc_lisp_poll_get_variable_float("cruise-rpm", &cruise_rpm_float)) {
		LOG_INFO(LISP_POLL, "cruise-rpm = %.6f\n", cruise_rpm_float);
	} else {
		LOG_INFO(LISP_POLL, "cruise-rpm = NOT FOUND\n");
	}

	int32_t current-profile_int = 0;
	if (vesc_lisp_poll_get_variable_int("current-profile", &current-profile_int)) {
		LOG_INFO(LISP_POLL, "current-profile = %d\n", current-profile_int);
	}
	else {
		LOG_INFO(LISP_POLL, "current-profile = NOT FOUND\n");
	}
}

lisp_stats_t* vesc_lisp_poll_get_stats(void) {
	return stats_received ? &lisp_stats : nullptr;
}

bool vesc_lisp_poll_get_variable_int(const char *var_name, int32_t *value) {
	if (!stats_received || !var_name || !value) {
		return false;
	}
	
	for (uint8_t i = 0; i < lisp_stats.variable_count; i++) {
		if (strcmp(lisp_stats.variables[i].name, var_name) == 0) {
			*value = (int32_t)lisp_stats.variables[i].value;
			return true;
		}
	}
	
	return false;
}

bool vesc_lisp_poll_get_variable_float(const char *var_name, float *value) {
	if (!stats_received || !var_name || !value) {
		return false;
	}
	
	for (uint8_t i = 0; i < lisp_stats.variable_count; i++) {
		if (strcmp(lisp_stats.variables[i].name, var_name) == 0) {
			*value = (float)lisp_stats.variables[i].value;
			return true;
		}
	}
	
	return false;
}

