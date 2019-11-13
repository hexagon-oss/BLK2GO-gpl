/* SPDX-License-Identifier: LGPL-2.0 */

#ifndef _MPF_IMX_DEV_H
#define _MPF_IMX_DEV_H

/**
 * struct mpf_imx_buf - IMX sensor data buffer
 *
 * @index:      id number of the buffer
 * @size:       size in bytes of the buffer (its payload)
 * @offset:     offset from the start of the device memory for this buffer,
 *		actually the physical DMA address.
 * @vaddr:	kernel virtual address of buffer
 * @ts:		64-bit ktime (ns) timestamp
 * @sequence:   sequence count of this sample buffer (SW generated).
 *
 * Contains data exchanged by application and driver.
 */
struct mpf_imx_buf {
	unsigned int index;	/* buffer index/id */
	size_t size;		/* out */
	off_t offset;		/* out, suitable for mmap (DMA address) */
	void *vaddr;		/* kernel virtual address */
	int64_t ts;		/* out, IRQ kernel timestamp in ns */
	uint32_t sequence;	/* out */
};

/* XXX: No error status information implemented in FPGA */
struct mpf_imx_capinfo {
	unsigned int error;		/* out, indicates error */
};

/**
 * struct mpf_imx_ctrl - the control structure
 *
 * @val:	The control's value.
 * @id:		The control ID.
 */
struct mpf_imx_ctrl {
	unsigned int id;
	uint32_t val;
};
/* global controls */
#define MPF_IMX_CID_TIMEOUT	0	/* in ms, global */
#define MPF_IMX_CID_XTRIG_DELAY	1	/* in ms, global */
/* per sensor controls */
#define MPF_IMX_CID_EXPOSURE	16	/* in ms, per sensor */
#define MPF_IMX_CID_TAG		17	/* per sensor */

/* IOCTLs */
#define MPF_IMX_IOC_ALLOCBUFS	_IOR('M', 0, struct mpf_imx_buf)
#define MPF_IMX_IOC_QUERYBUF	_IOR('M', 1, struct mpf_imx_buf)
#define MPF_IMX_IOC_QBUF	_IOR('M', 2, struct mpf_imx_buf)
#define MPF_IMX_IOC_DQBUF	_IOR('M', 3, struct mpf_imx_buf)
#define MPF_IMX_IOC_START	_IO('M', 4)
#define MPF_IMX_IOC_STOP	_IO('M', 5)
#define MPF_IMX_IOC_CAPINFO	_IOR('M', 6, struct mpf_imx_capinfo)
#define MPF_IMX_IOC_S_CTRL	_IOR('M', 7, struct mpf_imx_ctrl)
#define MPF_IMX_IOC_G_CTRL	_IOR('M', 8, struct mpf_imx_ctrl)

/*
 * The length of an DMA transfer is fixed to the size of 1 frame
 *   1 Frame := 3.197.376 (0x30c9c0) Bytes (1456 * 1098 * 2)
 *		+ 8 Bytes Timestamp (ticks)
 *	      + 4 byte information tag + 4 Byte exposure time
 * This gives hexadecimal 0x30C9D0
 */
#define MPF_IMX_FRAME_SIZE	0x0030C9D0
#define MPF_IMX_N_SENSORS 3

/* stream buffers per sensor */
#define MPF_IMX_N_BUFS		3
#define MPF_IMX_MAX_BUFS	6

#endif /* _MPF_IMX_DEV_H */
