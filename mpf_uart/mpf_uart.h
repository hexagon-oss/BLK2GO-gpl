/* SPDX-License-Identifier: MIT */

#ifndef MPF_UART_H
#define MPF_UART_H

#include <linux/ioctl.h>

struct mpf_uart_status {
	uint32_t error;
	int error_count;
};

#define MPF_UART_STATUS	_IOR('U', 0, struct mpf_uart_status)

#endif
