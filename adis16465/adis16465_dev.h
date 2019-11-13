/* SPDX-License-Identifier: LGPL-2.0 */

#ifndef _ADIS_DEV_H
#define _ADIS_DEV_H

#define ADIS16465_REG_DIAG_STAT		0x02
#define ADIS16465_REG_X_GYRO_LOW	0x04
#define ADIS16465_REG_X_GYRO_OUT	0x06
#define ADIS16465_REG_Y_GYRO_LOW	0x08
#define ADIS16465_REG_Y_GYRO_OUT	0x0a
#define ADIS16465_REG_Z_GYRO_LOW	0x0c
#define ADIS16465_REG_Z_GYRO_OUT	0x0e
#define ADIS16465_REG_X_ACCEL_LOW	0x10
#define ADIS16465_REG_X_ACCEL_OUT	0x12
#define ADIS16465_REG_Y_ACCEL_LOW	0x14
#define ADIS16465_REG_Y_ACCEL_OUT	0x16
#define ADIS16465_REG_Z_ACCEL_LOW	0x18
#define ADIS16465_REG_Z_ACCEL_OUT	0x1a
#define ADIS16465_REG_TEMP_OUT		0x1c
#define ADIS16465_REG_DATA_CNTR		0x22
#define ADIS16465_REG_MSC_CTRL		0x60 /* misc control */
#define MSC_CTRL_SYNC_MASK		0x07
#define MSC_CTRL_SYNC_SHIFT		0x02
#define MSC_CTRL_SYNC_POL_HIGH		0x02
#define MSC_CTRL_DR_POL_HIGH		0x01
#define ADIS16465_REG_UP_SCALE		0x62
#define ADIS16465_REG_DEC_RATE		0x64
#define ADIS16465_REG_PROD_ID		0x72 /* Product identifier */
#define ADIS16465_REG_SERIAL_NUM	0x74 /* Serial number, lot specific */

#define ADIS_WRITE_REG(reg) ((0x80 | (reg)))
#define ADIS_READ_REG(reg) ((reg) & 0x7f)

#define ADIS16465_BURST_DATA_LEN	20	/* excluding the cmd bytes */

#if defined(__cplusplus)
extern "C" {
#endif

/* 32-bit data with added timestamp */
struct adis16465_data32_ts {
/* x00 */ uint32_t x_gyro;
/* x04 */ uint32_t y_gyro;
/* x08 */ uint32_t z_gyro;
/* x0c */ uint32_t x_accel;
/* x10 */ uint32_t y_accel;
/* x14 */ uint32_t z_accel;
/* x18 */ uint16_t temp;
/* x1a */ uint16_t data_cntr;
/* x1c */ uint16_t adis_ts;
/* x1e */ uint16_t reserved;
/* x20 */ int64_t timestamp;
};

struct adis16465_burst_data {
	uint16_t diag;
	uint16_t x_gyro;
	uint16_t y_gyro;
	uint16_t z_gyro;
	uint16_t x_accel;
	uint16_t y_accel;
	uint16_t z_accel;
	uint16_t temp;
	uint16_t data_cntr;
	uint16_t checksum;
};

/* burst data with added timestamp */
struct adis16465_burst_data_ts {
/* x00 */ uint16_t diag;
/* x02 */ uint16_t x_gyro;
/* x04 */ uint16_t y_gyro;
/* x06 */ uint16_t z_gyro;
/* x08 */ uint16_t x_accel;
/* x0a */ uint16_t y_accel;
/* x0c */ uint16_t z_accel;
/* x0e */ uint16_t temp;
/* x10 */ uint16_t data_cntr;
/* x12 */ uint16_t checksum;
/* x14 */ uint32_t alignment_fill;
/* x18 */ int64_t timestamp;
};

/* burst data when operating in scaled sync mode */
struct adis16465_burst_data_scaled_sync_ts {
	uint16_t diag;
	uint16_t x_gyro;
	uint16_t y_gyro;
	uint16_t z_gyro;
	uint16_t x_accel;
	uint16_t y_accel;
	uint16_t z_accel;
	uint16_t temp;
	uint16_t adis_time_stamp;
	uint16_t checksum;
/*x14*/	uint32_t alignment_fill;
/*x18*/	int64_t timestamp;
};

/* processed burst data, in CPU byte order */
struct adis16465_data {
	int64_t timestamp;	/* trigger timestamp */
	uint16_t x_gyro;
	uint16_t y_gyro;
	uint16_t z_gyro;
	uint16_t x_accel;
	uint16_t y_accel;
	uint16_t z_accel;
	uint16_t temp;
};

/* IOCTLs */

#define ADIS_IOC_MAGIC	'A'

struct adis_reg_set_get {
	unsigned int addr;	/* Register number */
	unsigned int val;	/* Register value */
};
#define ADIS_IOC_REG_SET _IOW(ADIS_IOC_MAGIC, 0, struct adis_reg_set_get)
#define ADIS_IOC_REG_GET _IOR(ADIS_IOC_MAGIC, 1, struct adis_reg_set_get)

enum adis_data_mode {
	DATA32 = 0,		/* default */
	BURST = 1,		/* burst converted to CPU endianess */
	BURSTRAW = 2,		/* raw burst */
};
#define ADIS_IOC_DATA_MODE_SET _IOW(ADIS_IOC_MAGIC, 2, enum adis_data_mode)

#define ADIS_IOC_START _IO(ADIS_IOC_MAGIC, 3)
#define ADIS_IOC_STOP _IO(ADIS_IOC_MAGIC, 4)

enum adis_sync_mode {
	INTERNAL    = 0,
	DIRECT_SYNC = 1,
	SCALED_SYNC = 2,
	OUT_SYNC    = 3, /* DO NOT USE, otherwise 2 outputs collide */
	reserved_4  = 4,
	PULSE_SYNC  = 5,
	reserved_6  = 6,
	reserved_7  = 7,
};

#define ADIS_IOC_SYNC_MODE_SET _IOW(ADIS_IOC_MAGIC, 5, enum adis_sync_mode)
#define ADIS_IOC_SYNC_MODE_GET _IOR(ADIS_IOC_MAGIC, 6, enum adis_sync_mode)

#define ADIS_IOC_SAMPLE_RATE_GET _IO(ADIS_IOC_MAGIC, 7)

#if defined(__cplusplus)
}
#endif

#endif /* _ADIS_DEV_H */
