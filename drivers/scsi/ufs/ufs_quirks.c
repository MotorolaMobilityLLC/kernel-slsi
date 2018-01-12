/*
 * Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "ufshcd.h"
#include "ufs_quirks.h"

#ifndef UFS_VENDOR_ID_SAMSUNG
#define UFS_VENDOR_ID_SAMSUNG	0x1ce
#endif

#define SERIAL_NUM_SIZE 6
#define TOSHIBA_SERIAL_NUM_SIZE 10


/*UN policy
*  16 digits : mandate + serial number(6byte, hex raw data)
*  18 digits : manid + mandate + serial number(sec, hynix : 6byte hex,
*                                                                toshiba : 10byte + 00, ascii)
*/
void ufs_set_sec_unique_number(struct ufs_hba *hba, u8 *str_desc_buf, u8 *desc_buf)
{
	u8 manid;
	u8 snum_buf[UFS_UN_MAX_DIGITS];

	manid = hba->manufacturer_id & 0xFF;
	memset(hba->unique_number, 0, sizeof(hba->unique_number));
	memset(snum_buf, 0, sizeof(snum_buf));

#if defined(CONFIG_UFS_UN_18DIGITS)

	memcpy(snum_buf, str_desc_buf + QUERY_DESC_HDR_SIZE, SERIAL_NUM_SIZE);

	sprintf(hba->unique_number, "%02x%02x%02x%02x%02x%02x%02x%02x%02x",
		manid,
		desc_buf[DEVICE_DESC_PARAM_MANF_DATE], desc_buf[DEVICE_DESC_PARAM_MANF_DATE+1],
		snum_buf[0], snum_buf[1], snum_buf[2], snum_buf[3], snum_buf[4], snum_buf[5]);

	/* Null terminate the unique number string */
	hba->unique_number[UFS_UN_18_DIGITS] = '\0';

#else
	/*default is 16 DIGITS UN*/
	memcpy(snum_buf, str_desc_buf + QUERY_DESC_HDR_SIZE, SERIAL_NUM_SIZE);

	sprintf(hba->unique_number, "%02x%02x%02x%02x%02x%02x%02x%02x",
		desc_buf[DEVICE_DESC_PARAM_MANF_DATE], desc_buf[DEVICE_DESC_PARAM_MANF_DATE+1],
		snum_buf[0], snum_buf[1], snum_buf[2], snum_buf[3], snum_buf[4], snum_buf[5]);

	/* Null terminate the unique number string */
	hba->unique_number[UFS_UN_16_DIGITS] = '\0';
#endif
}


