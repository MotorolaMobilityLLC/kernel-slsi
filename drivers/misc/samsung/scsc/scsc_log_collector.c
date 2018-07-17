/********************************************************************************
 *
 *   Copyright (c) 2016 - 2018 Samsung Electronics Co., Ltd. All rights reserved.
 *
 ********************************************************************************/
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/list_sort.h>
#include <linux/limits.h>
#include <linux/workqueue.h>

#include <scsc/scsc_log_collector.h>
#include "scsc_log_collector_proc.h"
#include <scsc/scsc_mx.h>

#define SCSC_NUM_CHUNKS_SUPPORTED	12

/* Add-remove supported chunks on this kernel */
static u8 chunk_supported_sbl[SCSC_NUM_CHUNKS_SUPPORTED] = {
	SCSC_LOG_CHUNK_SYNC,
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
};

/* Collect logs in an intermediate buffer to be collected at later time (mmap or wq) */
static bool collect_to_ram;
module_param(collect_to_ram, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(collect_to_ram, "Collect buffer in ram");

static char collection_dir_buf[256] = "/data/exynos/log/wifi/";
module_param_string(collection_target_directory, collection_dir_buf, sizeof(collection_dir_buf), S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(collection_target_directory, "Specify collection target directory");

struct scsc_log_client {
	struct list_head list;
	struct scsc_log_collector_client *collect_client;
};
static struct scsc_log_collector_list { struct list_head list; } scsc_log_collector_list = {
	.list = LIST_HEAD_INIT(scsc_log_collector_list.list)
};

struct scsc_log_status {
	struct file *fp;
	loff_t pos;
	bool in_collection;
	char fapi_ver[SCSC_LOG_FAPI_VERSION_SIZE];

	struct workqueue_struct *collection_workq;
	struct work_struct	collect_work;
	enum scsc_log_reason    collect_reason;
} log_status;

static DEFINE_MUTEX(log_mutex);

static void collection_worker(struct work_struct *work)
{
	struct scsc_log_status *ls;

	ls = container_of(work, struct scsc_log_status, collect_work);
	if (!ls)
		return;
	pr_info("SCSC running scheduled Log Collection - reason:%d\n", ls->collect_reason);
	scsc_log_collector_collect(ls->collect_reason);
}

/* Module init */
int __init scsc_log_collector(void)
{
	pr_info("Log Collector Init\n");

	log_status.in_collection = false;
	log_status.collection_workq = create_workqueue("log_collector");
	if (log_status.collection_workq)
		INIT_WORK(&log_status.collect_work, collection_worker);
	scsc_log_collect_proc_create();
	return 0;
}

void __exit scsc_log_collector_exit(void)
{
	scsc_log_collect_proc_remove();
	if (log_status.collection_workq) {
		cancel_work_sync(&log_status.collect_work);
		destroy_workqueue(log_status.collection_workq);
		log_status.collection_workq = NULL;
	}

	pr_info("Log Collect Unloaded\n");
}

module_init(scsc_log_collector);
module_exit(scsc_log_collector_exit);

static bool scsc_is_chunk_supported(u8 type)
{
	u8 i;

	for (i = 0; i < SCSC_NUM_CHUNKS_SUPPORTED; i++) {
		if (type == chunk_supported_sbl[i])
			return true;
	}

	return false;
}

static int scsc_log_collector_compare(void *priv, struct list_head *A, struct list_head *B)
{
	struct scsc_log_client *a = list_entry(A, typeof(*a), list);
	struct scsc_log_client *b = list_entry(B, typeof(*b), list);

	if (a->collect_client->type < b->collect_client->type)
		return -1;
	else
		return 1;
}

int scsc_log_collector_register_client(struct scsc_log_collector_client *collect_client)
{
	struct scsc_log_client *lc;

	if (!scsc_is_chunk_supported(collect_client->type)) {
		pr_info("Type not supported: %d\n", collect_client->type);
		return -EIO;
	}

	mutex_lock(&log_mutex);
	lc = kzalloc(sizeof(*lc), GFP_KERNEL);
	if (!lc) {
		mutex_unlock(&log_mutex);
		return -ENOMEM;
	}

	lc->collect_client = collect_client;
	list_add_tail(&lc->list, &scsc_log_collector_list.list);

	/* Sort the list */
	list_sort(NULL, &scsc_log_collector_list.list, scsc_log_collector_compare);

	pr_info("Registered client: %s\n", collect_client->name);
	mutex_unlock(&log_mutex);
	return 0;
}
EXPORT_SYMBOL(scsc_log_collector_register_client);

int scsc_log_collector_unregister_client(struct scsc_log_collector_client *collect_client)
{
	struct scsc_log_client *lc, *next;
	bool match = false;

	/* block any attempt of unregistering while a collection is in progres */
	mutex_lock(&log_mutex);
	list_for_each_entry_safe(lc, next, &scsc_log_collector_list.list, list) {
		if (lc->collect_client == collect_client) {
			match = true;
			list_del(&lc->list);
			kfree(lc);
		}
	}

	if (match == false)
		pr_err("FATAL, no match for given scsc_log_collector_client\n");

	pr_info("Unregistered client: %s\n", collect_client->name);
	mutex_unlock(&log_mutex);

	return 0;
}
EXPORT_SYMBOL(scsc_log_collector_unregister_client);


static inline int __scsc_log_collector_write_to_ram(char __user *buf, size_t count, u8 align)
{
	return 0;
}

static inline int __scsc_log_collector_write_to_file(char __user *buf, size_t count, u8 align)
{
	int ret = 0;

	if (!log_status.in_collection)
		return -EIO;

	log_status.pos = (log_status.pos + align - 1) & ~(align - 1);
	/* Write buf to file */
	ret = vfs_write(log_status.fp, buf, count, &log_status.pos);
	if (ret < 0) {
		pr_err("write file error, err = %d\n", ret);
		return ret;
	}
	return 0;
}

int scsc_log_collector_write(char __user *buf, size_t count, u8 align)
{
	if (collect_to_ram)
		return __scsc_log_collector_write_to_ram(buf, count, align);
	else
		return __scsc_log_collector_write_to_file(buf, count, align);
}
EXPORT_SYMBOL(scsc_log_collector_write);

static inline int __scsc_log_collector_collect_to_ram(enum scsc_log_reason reason)
{
	return 0;
}

#define align_chunk(ppos) (((ppos) + (SCSC_LOG_CHUNK_ALIGN - 1)) & \
			  ~(SCSC_LOG_CHUNK_ALIGN - 1))

const char *scsc_loc_reason_str[] = { "unknown", "fw_panic", "host_trig",
				      "fw_trig", "dumpstate", "wlan_disc", "bt_trig" /* Add others */};

static inline int __scsc_log_collector_collect_to_file(enum scsc_log_reason reason)
{
	struct scsc_log_client *lc, *next;
	struct timeval t;
	struct tm tm_n;
	mm_segment_t old_fs;
	char memdump_path[128];
	int ret = 0;
	char version_fw[SCSC_LOG_FW_VERSION_SIZE] = {0};
	char version_host[SCSC_LOG_HOST_VERSION_SIZE] = {0};
	u32 mem_pos, temp_pos, chunk_size;
	ktime_t start;
	u8 num_chunks = 0;
	u16 first_chunk_pos = SCSC_LOG_OFFSET_FIRST_CHUNK;
	struct scsc_log_sbl_header sbl_header;
	struct scsc_log_chunk_header chk_header;
	u8 j;

	mutex_lock(&log_mutex);

	pr_info("Log collection to file triggered\n");

	start = ktime_get();
	do_gettimeofday(&t);
	time_to_tm(t.tv_sec, 0, &tm_n);

	snprintf(memdump_path, sizeof(memdump_path), "%s%s_%s.sbl",
		 collection_dir_buf, "scsc_log", scsc_loc_reason_str[reason]);

	/* change to KERNEL_DS address limit */
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	log_status.fp = filp_open(memdump_path, O_CREAT | O_WRONLY | O_SYNC | O_TRUNC, 0664);
	if (IS_ERR(log_status.fp)) {
		pr_err("open file error, err = %ld\n", PTR_ERR(log_status.fp));
		goto exit;
	}

	log_status.in_collection = true;
	/* Position index to start of the first chunk */
	log_status.pos = SCSC_LOG_OFFSET_FIRST_CHUNK;

	/* Call client init callbacks if any */
	list_for_each_entry_safe(lc, next, &scsc_log_collector_list.list, list) {
		if (lc->collect_client && lc->collect_client->collect_init)
			lc->collect_client->collect_init(lc->collect_client);
	}
	/* Traverse all the clients from the list.. Those would start calling scsc_log_collector_write!!*/
	/* Create chunk */
	list_for_each_entry_safe(lc, next, &scsc_log_collector_list.list, list) {
		if (lc->collect_client) {
			num_chunks++;
			/* Create Chunk */
			/* Store current post */
			temp_pos = log_status.pos;
			/* Make room for chunck header */
			log_status.pos += SCSC_CHUNK_HEADER_SIZE;
			/* Execute clients callbacks */
			if (lc->collect_client->collect(lc->collect_client, 0))
				goto exit;
			/* Write chunk headers */
			/* Align log_status.pos */
			mem_pos = log_status.pos = align_chunk(log_status.pos);
			chunk_size = log_status.pos - temp_pos - SCSC_CHUNK_HEADER_SIZE;
			/* rewind pos */
			log_status.pos = temp_pos;
			/* Write chunk header */
			memcpy(chk_header.magic, "CHK", 3);
			chk_header.type = (char)lc->collect_client->type;
			chk_header.chunk_size = chunk_size;
			scsc_log_collector_write((char *)&chk_header, sizeof(struct scsc_log_chunk_header), 1);
			/* restore position for next chunk */
			log_status.pos = mem_pos;
		}
	}
	/* Callbacks to clients have finished at this point. */
	/* Write file header */
	/* Move position to start of file */
	log_status.pos = 0;
	/* Write header */
	memset(&sbl_header, 0, sizeof(sbl_header));
	memcpy(sbl_header.magic, "SCSC", 4);
	sbl_header.version_major = SCSC_LOG_HEADER_VERSION_MAJOR;
	sbl_header.version_minor = SCSC_LOG_HEADER_VERSION_MINOR;
	sbl_header.num_chunks = num_chunks;
	sbl_header.reason = reason;
	sbl_header.offset_data = first_chunk_pos;
	mxman_get_fw_version(version_fw, SCSC_LOG_FW_VERSION_SIZE);
	memcpy(sbl_header.fw_version, version_fw, SCSC_LOG_FW_VERSION_SIZE);
	mxman_get_driver_version(version_host, SCSC_LOG_HOST_VERSION_SIZE);
	memcpy(sbl_header.host_version, version_host, SCSC_LOG_HOST_VERSION_SIZE);
	memcpy(sbl_header.fapi_version, log_status.fapi_ver, SCSC_LOG_FAPI_VERSION_SIZE);

	memset(sbl_header.supported_chunks, SCSC_LOG_CHUNK_INVALID, SCSC_SUPPORTED_CHUNKS_HEADER);
	for (j = 0; j < SCSC_NUM_CHUNKS_SUPPORTED; j++)
		sbl_header.supported_chunks[j] = chunk_supported_sbl[j];

	scsc_log_collector_write((char *)&sbl_header, sizeof(struct scsc_log_sbl_header), 1);

	/* Sync file from filesystem to physical media */
	ret = vfs_fsync(log_status.fp, 0);
	if (ret < 0) {
		pr_err("sync file error, error = %d\n", ret);
		goto exit;
	}

exit:
	/* close file before return */
	if (!IS_ERR(log_status.fp))
		filp_close(log_status.fp, current->files);

	/* restore previous address limit */
	set_fs(old_fs);

	log_status.in_collection = false;

	list_for_each_entry_safe(lc, next, &scsc_log_collector_list.list, list) {
		if (lc->collect_client && lc->collect_client->collect_end)
			lc->collect_client->collect_end(lc->collect_client);
	}

	pr_info("File %s collection end. Took: %lld\n", memdump_path, ktime_to_ns(ktime_sub(ktime_get(), start)));

	mutex_unlock(&log_mutex);
	return ret;
}

int scsc_log_collector_collect(enum scsc_log_reason reason)
{
	int ret;

	if (collect_to_ram)
		ret = __scsc_log_collector_collect_to_ram(reason);
	else
		ret =  __scsc_log_collector_collect_to_file(reason);

	return ret;
}
EXPORT_SYMBOL(scsc_log_collector_collect);

void  scsc_log_collector_schedule_collection(enum scsc_log_reason reason)
{
	if (log_status.collection_workq) {
		log_status.collect_reason = reason;
		queue_work(log_status.collection_workq, &log_status.collect_work);
	} else {
		pr_err("Log Collection Workqueue NOT available...aborting scheduled collection.\n");
	}
}
EXPORT_SYMBOL(scsc_log_collector_schedule_collection);

void scsc_log_collector_write_fapi(char __user *buf, size_t len)
{
	if (len > SCSC_LOG_FAPI_VERSION_SIZE)
		len = SCSC_LOG_FAPI_VERSION_SIZE;
	memcpy(log_status.fapi_ver, buf, len);
}
EXPORT_SYMBOL(scsc_log_collector_write_fapi);

MODULE_DESCRIPTION("SCSC Log collector");
MODULE_AUTHOR("SLSI");
MODULE_LICENSE("GPL and additional rights");
