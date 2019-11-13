/******************************************************************************
 *
 * Copyright (c) 2012 - 2019 Samsung Electronics Co., Ltd. All rights reserved
 *
 *****************************************************************************/
#include "dev.h"
#include "reg_info.h"
#include "debug.h"

void slsi_regd_init(struct slsi_dev *sdev)
{
	struct ieee80211_regdomain *slsi_world_regdom_custom = sdev->device_config.domain_info.regdomain;
	struct ieee80211_reg_rule  reg_rules[] = {
		/* Channel 1 - 11*/
		REG_RULE(2412 - 10, 2462 + 10, 40, 0, 20, 0),
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 0))
		/* Channel 12 - 13 NO_IR*/
		REG_RULE(2467 - 10, 2472 + 10, 40, 0, 20, NL80211_RRF_NO_IR),
#endif
		/* Channel 36 - 48 */
		REG_RULE(5180 - 10, 5240 + 10, 80, 0, 20, 0),
		/* Channel 52 - 64 */
		REG_RULE(5260 - 10, 5320 + 10, 80, 0, 20, NL80211_RRF_DFS),
		/* Channel 100 - 140 */
		REG_RULE(5500 - 10, 5700 + 10, 80, 0, 20, NL80211_RRF_DFS),
		/* Channel 149 - 165 */
		REG_RULE(5745 - 10, 5825 + 10, 80, 0, 20, 0),
	};

	int                        i;

	SLSI_DBG1_NODEV(SLSI_INIT_DEINIT, "regulatory init\n");
	sdev->regdb.regdb_state = SLSI_REG_DB_NOT_SET;
	slsi_world_regdom_custom->n_reg_rules = 6;
	for (i = 0; i < slsi_world_regdom_custom->n_reg_rules; i++)
		slsi_world_regdom_custom->reg_rules[i] = reg_rules[i];

	/* Country code '00' indicates world regulatory domain */
	slsi_world_regdom_custom->alpha2[0] = '0';
	slsi_world_regdom_custom->alpha2[1] = '0';

	wiphy_apply_custom_regulatory(sdev->wiphy, slsi_world_regdom_custom);
}

void  slsi_regd_deinit(struct slsi_dev *sdev)
{
	SLSI_DBG1(sdev, SLSI_INIT_DEINIT, "slsi_regd_deinit\n");

	kfree(sdev->device_config.domain_info.countrylist);
	sdev->device_config.domain_info.countrylist = NULL;
	kfree(sdev->regdb.freq_ranges);
	sdev->regdb.freq_ranges = NULL;
	kfree(sdev->regdb.reg_rules);
	sdev->regdb.reg_rules = NULL;
	kfree(sdev->regdb.rules_collection);
	sdev->regdb.rules_collection = NULL;
	kfree(sdev->regdb.country);
	sdev->regdb.country = NULL;
}

int slsi_read_regulatory(struct slsi_dev *sdev)
{
	struct file *fptr = NULL;
	char *reg_file_t = "/vendor/etc/wifi/slsi_reg_database.bin";
	int i = 0, j = 0, index = 0;
	uint32_t num_freqbands = 0, num_rules = 0, num_collections = 0;

	if (sdev->regdb.regdb_state == SLSI_REG_DB_SET) {
		SLSI_INFO(sdev, "Regulatory is already set!\n");
		sdev->regdb.regdb_state = SLSI_REG_DB_ERROR;
		return 0;
	}

	fptr = filp_open(reg_file_t, O_RDONLY, 0);
	if (IS_ERR(fptr) || !fptr) {
		SLSI_INFO(sdev, "Error! opening file %s\n", reg_file_t);
		return -EINVAL;
	}

	kernel_read(fptr, &sdev->regdb.version, sizeof(uint32_t), &fptr->f_pos);
	kernel_read(fptr, &num_freqbands, sizeof(uint32_t), &fptr->f_pos);

	sdev->regdb.freq_ranges = kmalloc(sizeof(*sdev->regdb.freq_ranges) * num_freqbands, GFP_KERNEL);
	if (!sdev->regdb.freq_ranges) {
		SLSI_ERR(sdev, "kmalloc of sdev->regdb->freq_ranges failed\n");
		sdev->regdb.regdb_state = SLSI_REG_DB_ERROR;
		return -EINVAL;
	}
	for (i = 0; i < num_freqbands; i++)
		kernel_read(fptr, &sdev->regdb.freq_ranges[i], sizeof(struct regdb_file_freq_range), &fptr->f_pos);

	kernel_read(fptr, &num_rules, sizeof(uint32_t), &fptr->f_pos);

	sdev->regdb.reg_rules = kmalloc(sizeof(*sdev->regdb.reg_rules) * num_rules, GFP_KERNEL);
	if (!sdev->regdb.reg_rules) {
		SLSI_ERR(sdev, "kmalloc of sdev->regdb->reg_rules failed\n");
		kfree(sdev->regdb.freq_ranges);
		sdev->regdb.freq_ranges = NULL;
		sdev->regdb.regdb_state = SLSI_REG_DB_ERROR;
		return -EINVAL;
	}
	for (i = 0; i < num_rules; i++) {
		kernel_read(fptr, &index, sizeof(uint32_t), &fptr->f_pos);
		sdev->regdb.reg_rules[i].freq_range = &sdev->regdb.freq_ranges[index];
		kernel_read(fptr, &sdev->regdb.reg_rules[i].max_eirp, sizeof(uint32_t), &fptr->f_pos);
		kernel_read(fptr, &sdev->regdb.reg_rules[i].flags, sizeof(uint32_t), &fptr->f_pos);
	}

	kernel_read(fptr, &num_collections, sizeof(uint32_t), &fptr->f_pos);

	sdev->regdb.rules_collection = kmalloc(sizeof(*sdev->regdb.rules_collection) * num_collections, GFP_KERNEL);
	if (!sdev->regdb.rules_collection) {
		SLSI_ERR(sdev, "kmalloc of sdev->regdb->rules_collection failed\n");
		kfree(sdev->regdb.freq_ranges);
		sdev->regdb.freq_ranges = NULL;
		kfree(sdev->regdb.reg_rules);
		sdev->regdb.reg_rules = NULL;
		sdev->regdb.regdb_state = SLSI_REG_DB_ERROR;
		return -EINVAL;
	}
	for (i = 0; i < num_collections; i++) {
		kernel_read(fptr, &sdev->regdb.rules_collection[i].reg_rule_num, sizeof(uint32_t), &fptr->f_pos);
		for (j = 0; j < sdev->regdb.rules_collection[i].reg_rule_num; j++) {
			kernel_read(fptr, &index, sizeof(uint32_t), &fptr->f_pos);
			sdev->regdb.rules_collection[i].reg_rule[j] = &sdev->regdb.reg_rules[index];
		}
	}

	kernel_read(fptr, &sdev->regdb.num_countries, sizeof(uint32_t), &fptr->f_pos);
	SLSI_INFO(sdev, "Regulatory Version: %d ,Number of Countries: %d\n", sdev->regdb.version, sdev->regdb.num_countries);

	sdev->regdb.country = kmalloc(sizeof(*sdev->regdb.country) * sdev->regdb.num_countries, GFP_KERNEL);
	if (!sdev->regdb.country) {
		SLSI_ERR(sdev, "kmalloc of sdev->regdb->country failed\n");
		kfree(sdev->regdb.freq_ranges);
		sdev->regdb.freq_ranges = NULL;
		kfree(sdev->regdb.reg_rules);
		sdev->regdb.reg_rules = NULL;
		kfree(sdev->regdb.rules_collection);
		sdev->regdb.rules_collection = NULL;
		sdev->regdb.regdb_state = SLSI_REG_DB_ERROR;
		return -EINVAL;
	}
	for (i = 0; i < sdev->regdb.num_countries; i++) {
		kernel_read(fptr, &sdev->regdb.country[i].alpha2, 2 * sizeof(uint8_t), &fptr->f_pos);
		kernel_read(fptr, &sdev->regdb.country[i].pad_byte, sizeof(uint8_t), &fptr->f_pos);
		kernel_read(fptr, &sdev->regdb.country[i].dfs_region, sizeof(uint8_t), &fptr->f_pos);
		kernel_read(fptr, &index, sizeof(uint32_t), &fptr->f_pos);
		sdev->regdb.country[i].collection = &sdev->regdb.rules_collection[index];
	}

	filp_close(fptr, NULL);
	sdev->regdb.regdb_state = SLSI_REG_DB_SET;
	return 0;
}
