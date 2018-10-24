/*
 * Copyright (C) 2016 Samsung Electronics.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/netdevice.h>
#include "modem_prj.h"
#include "modem_utils.h"
#include "modem_link_device_shmem.h"

static int save_dump_file(struct link_device *ld, struct io_device *iod,
		unsigned long arg, u8 __iomem *dump_base, size_t dump_size)
{
	size_t copied = 0;
	struct sk_buff *skb;
	unsigned int alloc_size = 0xE00;
	int ret;

	if (dump_size == 0 || dump_base == NULL) {
		mif_err("ERR! save_dump_file fail!\n");
		return -EFAULT;
	}

	while (copied < dump_size) {
		if (dump_size - copied < alloc_size)
			alloc_size =  dump_size - copied;

		skb = alloc_skb(alloc_size, GFP_ATOMIC);
		if (!skb) {
			skb_queue_purge(&iod->sk_rx_q);
			mif_err("ERR! alloc_skb fail, purged skb_rx_q\n");
			return -ENOMEM;
		}

		memcpy(skb_put(skb, alloc_size), dump_base + copied,
				alloc_size);
		copied += alloc_size;

		/* Record the IO device and the link device into the &skb->cb */
		skbpriv(skb)->iod = iod;
		skbpriv(skb)->ld = ld;

		skbpriv(skb)->lnk_hdr = false;
		skbpriv(skb)->exynos_ch = iod->id;

		ret = iod->recv_skb_single(iod, ld, skb);
		if (unlikely(ret < 0)) {
			struct modem_ctl *mc = ld->mc;

			mif_err_limited("%s: %s<-%s: %s->recv_skb fail (%d)\n",
				ld->name, iod->name, mc->name, iod->name, ret);
			dev_kfree_skb_any(skb);
			return ret;
		}
	}

	mif_info("Complete! (%zu bytes)\n", copied);

	return 0;
}

int save_vss_dump(struct link_device *ld, struct io_device *iod,
		unsigned long arg)
{
	struct shmem_link_device *shmd = to_shmem_link_device(ld);
	size_t vss_size = shm_get_vss_size();
	int ret = 0;

	if (vss_size == 0 || shmd->vss_base == NULL) {
		mif_err("ERR! save_vss_dump fail!\n");
		return -EFAULT;
	}

	ret = copy_to_user((void __user *)arg, &vss_size, sizeof(vss_size));
	if (ret) {
		mif_err("ERR! copy_from_user fail!\n");
		return -EFAULT;
	}

	return save_dump_file(ld, iod, arg, shmd->vss_base, vss_size);
}

int save_acpm_dump(struct link_device *ld, struct io_device *iod,
		unsigned long arg)
{
	struct shmem_link_device *shmd = to_shmem_link_device(ld);
	size_t acpm_size = shm_get_acpm_size();
	int ret = 0;

	if (acpm_size == 0 || shmd->acpm_base == NULL) {
		mif_err("ERR! save_acpm_dump fail!\n");
		return -EFAULT;
	}

	ret = copy_to_user((void __user *)arg, &acpm_size, sizeof(acpm_size));
	if (ret) {
		mif_err("ERR! copy_from_user fail!\n");
		return -EFAULT;
	}

	return save_dump_file(ld, iod, arg, shmd->acpm_base, acpm_size);
}

int save_shmem_dump(struct link_device *ld, struct io_device *iod,
		unsigned long arg)
{
	struct shmem_link_device *shmd = to_shmem_link_device(ld);
	size_t shmem_size = shmd->size;
	int ret = 0;
#ifdef CONFIG_CACHED_RAW_RX_BUFFER
	u8 *rxq_buff;
	int rxq_buff_size;

	rxq_buff = get_rxq_buff(shmd, IPC_RAW);
	rxq_buff_size = get_rxq_buff_size(shmd, IPC_RAW);
#endif

	if (shmem_size == 0 || shmd->base == NULL) {
		mif_err("ERR! save_shmem_dump fail!\n");
		return -EFAULT;
	}

	ret = copy_to_user((void __user *)arg, &shmem_size, sizeof(shmem_size));
	if (ret) {
		mif_err("ERR! copy_from_user fail!\n");
		return -EFAULT;
	}

#ifdef CONFIG_CACHED_RAW_RX_BUFFER
	ret = save_dump_file(ld, iod, arg, (u8 __iomem *)shmd->base, shmem_size - rxq_buff_size);
	__inval_dcache_area((void *)rxq_buff, rxq_buff_size);
	ret = save_dump_file(ld, iod, arg, (u8 __iomem *)rxq_buff, rxq_buff_size);
#else
	ret = save_dump_file(ld, iod, arg, (u8 __iomem *)shmd->base, shmem_size);
#endif
	return ret;
}
