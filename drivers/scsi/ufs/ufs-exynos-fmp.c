/*
 * Copyright (C) 2016 Samsung Electronics Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <crypto/diskcipher.h>
#include <crypto/fmp.h>

#include "ufshcd.h"
#include "ufs-exynos.h"

int exynos_ufs_fmp_cfg(struct ufs_hba *hba,
		       struct ufshcd_lrb *lrbp,
		       struct scatterlist *sg,
		       uint32_t index, int sector_offset)
{
	struct fmp_request req;
	struct scsi_cmnd *cmd = lrbp->cmd;
	struct bio *bio;
	struct crypto_diskcipher *dtfm;
	sector_t iv;

	if (!cmd || !virt_addr_valid(cmd)) {
		dev_err(hba->dev, "Invalid cmd:%p\n", cmd);
		goto no_crypto;
	}

	/* fill fmp_data_setting */
	bio = cmd->request->bio;
	dtfm = crypto_diskcipher_get(bio);
	if (dtfm) {
#ifdef EANBLE_DISKCIPHER_DEBUG
		crypto_diskcipher_check(bio, sg_page(sg));
#endif
		iv = bio->bi_iter.bi_sector + (sector_t) sector_offset;
		req.table = (void *)&lrbp->ucd_prdt_ptr[index];
		req.cmdq_enabled = 0;
		req.iv = &iv;
		req.ivsize = sizeof(iv);
#ifdef CONFIG_EXYNOS_FMP_FIPS
		/* check fips flag. use fmp without diskcipher */
		if (!dtfm->algo) {
			if (exynos_fmp_crypt((void *)dtfm, &req))
				goto no_crypto;
			return 0;
		}
#endif
		if (crypto_diskcipher_set_crypt(dtfm, &req)) {
			pr_warn("%s: fails to set crypt\n", __func__);
			return -EINVAL;
		}

		return 0;
	}
no_crypto:
	exynos_fmp_bypass(&lrbp->ucd_prdt_ptr[index], 0);
	return 0;
}

int exynos_ufs_fmp_clear(struct ufs_hba *hba, struct ufshcd_lrb *lrbp)
{
	int ret = 0;
	int sg_segments, idx;
	struct scatterlist *sg;
	struct ufshcd_sg_entry *prd_table;
	struct crypto_diskcipher *dtfm;
	struct fmp_crypto_info *ci;
	struct fmp_request req;

	if (!lrbp->cmd)
		goto out;

	sg_segments = scsi_sg_count(lrbp->cmd);
	if (!sg_segments)
		goto out;

	dtfm = crypto_diskcipher_get(lrbp->cmd->request->bio);
	if (dtfm) {
#ifdef CONFIG_EXYNOS_FMP_FIPS
		/* check fips flag. use fmp without diskcipher */
		if (!dtfm->algo) {
			prd_table =
				(struct ufshcd_sg_entry *)lrbp->ucd_prdt_ptr;
			scsi_for_each_sg(lrbp->cmd, sg, sg_segments, idx) {
				req.table = (void *)&prd_table[idx];
				ret = exynos_fmp_clear((void *)dtfm, &req);
				if (ret)
					break;
			}
			goto out;
		}
#endif
		/* clear key on descrptor */
		ci = crypto_tfm_ctx(crypto_diskcipher_tfm(dtfm));
		if (ci && (ci->enc_mode == EXYNOS_FMP_FILE_ENC)) {
			prd_table =
			    (struct ufshcd_sg_entry *)lrbp->ucd_prdt_ptr;
			scsi_for_each_sg(lrbp->cmd, sg, sg_segments, idx) {
				req.table = (void *)&prd_table[idx];
				ret = crypto_diskcipher_clear_crypt(dtfm, &req);
				if (ret)
					break;
			}
		}
	}
out:
	if (ret)
		pr_err("%s: Fail to clear desc (%d)\n", __func__, ret);
	return ret;
}
