/****************************************************************************
 *
 * Copyright (c) 2014 - 2018 Samsung Electronics Co., Ltd. All rights reserved
 *
 ****************************************************************************/

#ifndef __SCSC_LOG_COLLECTOR_H__
#define __SCSC_LOG_COLLECTOR_H__

/* High nibble is Major, Low nibble is Minor */
#define SCSC_LOG_HEADER_VERSION_MAJOR	0x00
#define SCSC_LOG_HEADER_VERSION_MINOR	0x00
/* Magic string. 4 bytes "SCSC"*/
/* Header version. 1 byte */
/* Num chunks. 1 byte */
/* Offset first Chunk. 2 bytes  */
/* Collection reason. 1 byte */
/* Reserved. 1 byte */
#define SCSC_LOG_HEADER_SIZE		(10)
#define SCSC_LOG_FW_VERSION_SIZE	(64)
#define SCSC_LOG_HOST_VERSION_SIZE	(64)
#define SCSC_LOG_FAPI_VERSION_SIZE	(64)
/* Reserved 2 . 6 byte */
#define SCSC_LOG_RESERVED_2             6
/* Ideally header + versions should be 16 bytes aligne*/
#define SCSC_SUPPORTED_CHUNKS_HEADER    48

#define SCSC_LOG_CHUNK_ALIGN		1
/* First chunk should be aligned */
#define SCSC_LOG_OFFSET_FIRST_CHUNK	(((SCSC_LOG_HEADER_SIZE + SCSC_LOG_FW_VERSION_SIZE + \
					SCSC_LOG_HOST_VERSION_SIZE + SCSC_LOG_FAPI_VERSION_SIZE + \
					SCSC_LOG_RESERVED_2 + SCSC_SUPPORTED_CHUNKS_HEADER) + \
					(SCSC_LOG_CHUNK_ALIGN - 1)) & ~(SCSC_LOG_CHUNK_ALIGN - 1))
enum scsc_log_reason {
	SCSC_LOG_REASON_UNKNOWN = 0,
	SCSC_LOG_REASON_FW_PANIC,
	SCSC_LOG_REASON_HOST_PROC_TRIGGERED,
	SCSC_LOG_REASON_FW_TRIGGERED,
	SCSC_LOG_REASON_DUMPSTATE,
	SCSC_LOG_REASON_WLAN_DISCONNECT,
	SCSC_LOG_REASON_BT_TRIGGERED,
	/* Add others */
};

extern const char *scsc_loc_reason_str[];

#define SCSC_CHUNK_DAT_LEN_SIZE		4
#define SCSC_CHUNK_TYP_LEN_SIZE		4
#define SCSC_CHUNK_HEADER_SIZE		(SCSC_CHUNK_DAT_LEN_SIZE + SCSC_CHUNK_TYP_LEN_SIZE)

/* CHUNKS WILL COLLECTED ON THIS ORDER -
 * SYNC SHOULD BE THE FIRST CHUNK
 * LOGRING SHOULD BE THE LAST ONE SO IT COULD CAPTURE COLLECTION ERRORS
 */
enum scsc_log_chunk_type {
	SCSC_LOG_CHUNK_SYNC, /* SYNC should be the first chunk to collect */
	SCSC_LOG_CHUNK_IMP,
	SCSC_LOG_CHUNK_MXL,
	SCSC_LOG_CHUNK_UDI,
	SCSC_LOG_CHUNK_BT_HCF,
	SCSC_LOG_CHUNK_WLAN_HCF,
	SCSC_LOG_CHUNK_HIP4_SAMPLER,
	SCSC_LOG_RESERVED_COMMON,
	SCSC_LOG_RESERVED_BT,
	SCSC_LOG_RESERVED_WLAN,
	SCSC_LOG_RESERVED_RADIO,
	SCSC_LOG_CHUNK_LOGRING,
	/* Add other chunks */
	SCSC_LOG_CHUNK_INVALID = 255,
};

/* SBL HEADER v 0.0*/
struct scsc_log_sbl_header {
	char magic[4];
	u8   version_major;
	u8   version_minor;
	u8   num_chunks;
	u8   reason;
	u16  offset_data;
	char fw_version[SCSC_LOG_FW_VERSION_SIZE];
	char host_version[SCSC_LOG_HOST_VERSION_SIZE];
	char fapi_version[SCSC_LOG_FAPI_VERSION_SIZE];
	u8   reserved2[SCSC_LOG_RESERVED_2];
	char supported_chunks[SCSC_SUPPORTED_CHUNKS_HEADER];
} __packed;

struct scsc_log_chunk_header {
	char magic[3];
	u8   type;
	u32  chunk_size;
} __packed;

struct scsc_log_collector_client {
	char *name;
	enum scsc_log_chunk_type type;
	int (*collect_init)(struct scsc_log_collector_client *collect_client);
	int (*collect)(struct scsc_log_collector_client *collect_client, size_t size);
	int (*collect_end)(struct scsc_log_collector_client *collect_client);
	void *prv;
};

int scsc_log_collector_register_client(struct scsc_log_collector_client *collect_client);
int scsc_log_collector_unregister_client(struct scsc_log_collector_client *collect_client);

/* Public method to register FAPI version. */
void scsc_log_collector_write_fapi(char __user *buf, size_t len);

int scsc_log_collector_collect(enum scsc_log_reason reason);
void scsc_log_collector_schedule_collection(enum scsc_log_reason reason);
int scsc_log_collector_write(char __user *buf, size_t count, u8 align);
#endif /* __SCSC_LOG_COLLECTOR_H__ */
