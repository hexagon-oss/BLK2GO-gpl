/* SPDX-License-Identifier: LGPL-2.0 */

#ifndef _MPF_EDM_DEV_H
#define _MPF_EDM_DEV_H

/**
 * struct edm_buf - EDM sample data buffer
 * @index:      id number of the buffer
 * @size:       size in bytes of the buffer (its payload)
 * @offset:     offset from the start of the device memory for this buffer,
 *		actually the physical DMA address.
 * @vaddr:	kernel virtual address of buffer
 * @ts:		64-bit ktime (ns) timestamp
 * @sequence:   sequence count of this sample buffer
 *
 * Contains data exchanged by application and driver.
 */
struct edm_buf {
	unsigned int index;	/* buffer index/id */
	size_t size;		/* out */
	off_t offset;		/* out, suitable for mmap (DMA address) */
	void *vaddr;		/* kernel virtual address */
	int64_t ts;		/* out, IRQ kernel timestamp in ns */
	uint32_t sequence;	/* out */
};

struct mpf_capinfo {
	unsigned int error;	  /* out, indicates error */
	uint32_t crc_error_counter;	/* copy of FPGA register */
	uint32_t frame_error_counter;	/* copy of FPGA register */
};

/* IOCTLs */
#define EDM_IOC_ALLOCBUFS	_IOR('M', 0, struct edm_buf)
#define EDM_IOC_QUERYBUF	_IOR('M', 1, struct edm_buf)
#define EDM_IOC_QBUF		_IOR('M', 2, struct edm_buf)
#define EDM_IOC_DQBUF		_IOR('M', 3, struct edm_buf)
#define EDM_IOC_START		_IO('M', 4)
#define EDM_IOC_STOP		_IO('M', 5)
#define EDM_IOC_CAPINFO		_IOR('M', 6, struct mpf_capinfo)

/*
 * The length of an DMA transfer is fixed to the size of 1 frame
 *   1 Frame := xxx Bytes + 8 Bytes Timestamp
 */
/*
 * Do not make the SA DMA size too large, otherwise the DMA might time out.
 * Do not make the DIST DMA size too small, otherwise the IRQ load is too high.
 */
#define MPF_EDM_SA_DMA_SIZE	0x00000100
#define MPF_EDM_FA_DMA_SIZE	0x00000400
#define MPF_EDM_DIST_DMA_SIZE	0x00018000

#define MPF_EDM_SA_N_BUFS	6
#define MPF_EDM_FA_N_BUFS	6
#define MPF_EDM_DIST_N_BUFS	6
#define MPF_EDM_MAX_BUFS	6

#endif /* _MPF_EDM_DEV_H */
