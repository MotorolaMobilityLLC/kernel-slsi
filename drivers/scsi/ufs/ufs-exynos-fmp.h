/*
 * Copyright (C) 2013-2014 Samsung Electronics Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _UFS_EXYNOS_FMP_H_
#define _UFS_EXYNOS_FMP_H_

#include "ufshcd.h"
#include "ufs-exynos.h"

#ifdef CONFIG_SCSI_UFS_EXYNOS_FMP
int exynos_ufs_fmp_cfg(struct ufs_hba *hba,
				struct ufshcd_lrb *lrbp,
				struct scatterlist *sg,
				uint32_t index,
				int sector_offset);
int exynos_ufs_fmp_clear(struct ufs_hba *hba, struct ufshcd_lrb *lrbp);
#else
inline int exynos_ufs_fmp_cfg(struct ufs_hba *hba,
				struct ufshcd_lrb *lrbp,
				struct scatterlist *sg,
				uint32_t index,
				int sector_offset)
{
	return 0;
}

int exynos_ufs_fmp_clear(struct ufs_hba *hba, struct ufshcd_lrb *lrbp)
{
	return 0;
}
#endif /* CONFIG_SCSI_UFS_EXYNOS_FMP */
#endif /* _UFS_EXYNOS_FMP_H_ */
