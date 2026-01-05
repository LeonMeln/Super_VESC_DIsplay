/*
	Copyright 2025 Super VESC Display

	VESC LISP Poll Module
	Polls LISP variables via CAN bus at 10 Hz
*/

#ifndef VESC_LISP_POLL_H_
#define VESC_LISP_POLL_H_

#include <stdint.h>
#include <stdbool.h>

// LISP variable structure
typedef struct {
	char name[64];
	double value;
} lisp_variable_t;

// LISP stats structure
typedef struct {
	double cpu_use;
	double heap_use;
	double mem_use;
	double stack_use;
	char done_ctx_r[128];
	lisp_variable_t variables[128];  // Max 128 variables
	uint8_t variable_count;
} lisp_stats_t;

// Functions
void vesc_lisp_poll_init(void);
void vesc_lisp_poll_start(void);
void vesc_lisp_poll_stop(void);
void vesc_lisp_poll_loop(void);
void vesc_lisp_poll_process_response(unsigned char *data, unsigned int len);
lisp_stats_t* vesc_lisp_poll_get_stats(void);

// Get variable value by name
// Returns true if variable found, false otherwise
// value is set to the variable's value
bool vesc_lisp_poll_get_variable_int(const char *var_name, int32_t *value);
bool vesc_lisp_poll_get_variable_float(const char *var_name, float *value);

#endif /* VESC_LISP_POLL_H_ */

