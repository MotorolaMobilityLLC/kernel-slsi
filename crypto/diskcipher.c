/*
 * Copyright (C) 2017 Samsung Electronics Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/blkdev.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/string.h>
#include <linux/crypto.h>
#include <crypto/algapi.h>
#include <crypto/diskcipher.h>

#include "internal.h"

#ifdef EANBLE_DISKCIPHER_DEBUG
#include <crypto/fmp.h>
#include <linux/mm_types.h>

#define DUMP_MAX 20

enum diskcipher_api {
	DISKC_API_ALLOC, DISKC_API_FREE, DISKC_API_SET,
	DISKC_API_GET, DISKC_API_CRYPT, DISKC_API_CLEAR, DISKC_API_MAX,
};

struct dump_err {
	struct page *page;
	struct bio bio;
	struct fmp_crypto_info ci;
	enum diskcipher_api api;
};

struct diskc_debug_info {
	struct dump_err dump[DUMP_MAX];
	int err;
	u32 cnt[DISKC_API_MAX][2];
};

static struct diskc_debug_info diskc_dbg;

static void print_err(void)
{
	int i, j;
	struct bio_vec *bv;	/* bio page list */
	struct bio *bio;
	struct fmp_crypto_info *ci;
	struct diskc_debug_info *dbg = &diskc_dbg;

	for (j = 0; j < dbg->err; j++) {
		bio = &dbg->dump[j].bio;
		ci = &dbg->dump[j].ci;

		if (bio) {
			pr_info
			    ("%s(%d/%d): bio:%p ci:%p page:%p flag:%x, opf:%x, crypt:%p\n",
			     __func__, j, dbg->err, bio, ci, &dbg->dump[j].page,
			     bio->bi_flags, bio->bi_opf, bio->bi_aux_private);
			print_hex_dump(KERN_CONT, "bio:", DUMP_PREFIX_OFFSET,
				16, 1, bio, sizeof(struct bio), false);
			for (i = 0; i < bio->bi_max_vecs; i++) {
				bv = &bio->bi_io_vec[i];
				pr_info("bv[%d] page:%p len:%d offset:%d\n",
					i, bv->bv_page, bv->bv_len, bv->bv_offset);
			}
		}

		if (ci) {
			pr_info("[ci] key_size:%d algo_mode:%d\n",
				ci->key_size, ci->algo_mode);
			print_hex_dump(KERN_CONT, "key:", DUMP_PREFIX_OFFSET,
				       16, 1, ci->key, sizeof(ci->key), false);
		}
	}
}

static void dump_err(struct crypto_diskcipher *ci, enum diskcipher_api api,
	      struct bio *bio, struct page *page)
{
	struct diskc_debug_info *dbg = &diskc_dbg;

	if ((dbg->err < DUMP_MAX) && ci) {
		struct crypto_tfm *tfm = crypto_diskcipher_tfm(ci);

		dbg->dump[dbg->err].api = api;
		memcpy(&dbg->dump[dbg->err].ci, crypto_tfm_ctx(tfm),
		       sizeof(struct fmp_crypto_info));

		if (page)
			dbg->dump[dbg->err].page = page;
		if (bio)
			memcpy(&dbg->dump[dbg->err].bio, bio,
				sizeof(struct bio));
	}
	dbg->err++;
}

static inline void disckipher_log(enum diskcipher_api api, int ret,
			       struct crypto_diskcipher *ci)
{
	struct diskc_debug_info *dbg = &diskc_dbg;

	dbg->cnt[api][0]++;
	if (ret) {
		dbg->cnt[api][1]++;
		if (ci)
			dump_err(ci, api, NULL, NULL);
	}
}

static void disckipher_log_show(struct seq_file *m)
{
	int i;
	char name[DISKC_API_MAX][8]
	    = {"alloc", "free", "set", "get", "crypt", "clear"};
	struct diskc_debug_info *dbg = &diskc_dbg;

	for (i = 0; i < DISKC_API_MAX; i++)
		seq_printf(m, "%s\t: %6u(err:%u)\n",
			name[i], dbg->cnt[i][0], dbg->cnt[i][1]);

	if (dbg->err)
		print_err();
}

/* check diskcipher for FBE */
void crypto_diskcipher_check(struct bio *bio, struct page *page)
{
#ifdef FBE_DEBUG
	int ret = 0;
	struct crypto_diskcipher *ci = NULL;

	if (page && !PageAnon(page) && bio)
		if (page->mapping)
			if (page->mapping->host)
				if (page->mapping->host->i_crypt_info) {
					ci = page->mapping->host->i_crypt_info->ci_dtfm;
					if (ci && (bio->bi_aux_private != ci)
					    && (!(bio->bi_flags & REQ_OP_DISCARD))) {
						dump_err(ci, DISKC_API_GET, bio, page);
						ret = 1;
					}
				}
	disckipher_log(DISKC_API_GET, ret, ci);
#endif
}
#else
enum diskcipher_api {
	DISKC_API_ALLOC, DISKC_API_FREE, DISKC_API_SET,
	DISKC_API_GET, DISKC_API_CRYPT, DISKC_API_CLEAR, DISKC_API_MAX,
};

#define disckipher_log_show(a) do { } while (0)
#define disckipher_log(a, b, c) do { } while (0)
#endif

#define DISKC_NAME "-disk"
#define DISKC_NAME_SIZE (5)

struct crypto_diskcipher *crypto_diskcipher_get(struct bio *bio)
{
	if (!bio || !virt_addr_valid(bio)) {
		pr_err("%s: Invalid bio:%p\n", __func__, bio);
		return NULL;
	}
	if (bio->bi_opf & REQ_CRYPT)
		return bio->bi_aux_private;
	else
		return NULL;
}

void crypto_diskcipher_set(struct bio *bio,
			   struct crypto_diskcipher *diskcipher)
{
	if (bio && diskcipher) {
		bio->bi_opf |= (REQ_CRYPT | REQ_AUX_PRIV);
		bio->bi_aux_private = diskcipher;
	}
	disckipher_log(DISKC_API_SET, 0, NULL);
}

int crypto_diskcipher_setkey(struct crypto_diskcipher *tfm, const char *in_key,
			     unsigned int key_len, bool persistent)
{
	struct crypto_tfm *base = crypto_diskcipher_tfm(tfm);
	struct diskcipher_alg *cra = crypto_diskcipher_alg(base->__crt_alg);

	if (!cra) {
		pr_err("%s: doesn't exist cra", __func__);
		return -EINVAL;
	}
	return cra->setkey(base, in_key, key_len, persistent);
}

int crypto_diskcipher_clearkey(struct crypto_diskcipher *tfm)
{
	struct crypto_tfm *base = crypto_diskcipher_tfm(tfm);
	struct diskcipher_alg *cra = crypto_diskcipher_alg(base->__crt_alg);

	if (!cra) {
		pr_err("%s: doesn't exist cra", __func__);
		return -EINVAL;
	}
	return cra->clearkey(base);
}

int crypto_diskcipher_set_crypt(struct crypto_diskcipher *tfm, void *req)
{
	int ret = 0;
	struct crypto_tfm *base = crypto_diskcipher_tfm(tfm);
	struct diskcipher_alg *cra = crypto_diskcipher_alg(base->__crt_alg);

	if (!cra) {
		pr_err("%s: doesn't exist cra", __func__);
		ret = EINVAL;
		goto out;
	}

	ret = cra->crypt(base, req);
out:
	if (ret)
		pr_err("%s fails ret:%d, cra:%p\n", __func__, ret, cra);
	pr_debug("%s done\n", __func__);
	disckipher_log(DISKC_API_CRYPT, ret, tfm);
	return ret;
}

int crypto_diskcipher_clear_crypt(struct crypto_diskcipher *tfm, void *req)
{
	int ret = 0;
	struct crypto_tfm *base = crypto_diskcipher_tfm(tfm);
	struct diskcipher_alg *cra = crypto_diskcipher_alg(base->__crt_alg);

	if (!cra) {
		pr_err("%s: doesn't exist cra", __func__);
		ret = EINVAL;
		goto out;
	}

	ret = cra->clear(base, req);
	if (ret)
		pr_err("%s fails", __func__);

out:
	pr_debug("%s done\n", __func__);
	disckipher_log(DISKC_API_CLEAR, ret, tfm);
	return ret;
}

#ifndef CONFIG_CRYPTO_MANAGER_DISABLE_TESTS
int diskcipher_do_crypt(struct crypto_diskcipher *tfm,
			struct diskcipher_test_request *req)
{
	int ret;
	struct crypto_tfm *base = crypto_diskcipher_tfm(tfm);
	struct diskcipher_alg *cra = crypto_diskcipher_alg(base->__crt_alg);

	if (!cra) {
		pr_err("%s: doesn't exist cra", __func__);
		ret = EINVAL;
		goto out;
	}

	if (cra->do_crypt)
		ret = cra->do_crypt(base, req);
	else
		ret = -EINVAL;
	if (ret)
		pr_err("%s fails ret:%d", __func__, ret);

out:
	pr_debug("%s done\n", __func__);
	return ret;
}
#endif

static int crypto_diskcipher_init_tfm(struct crypto_tfm *tfm)
{
	return 0;
}

unsigned int crypto_diskcipher_extsize(struct crypto_alg *alg)
{
	return alg->cra_ctxsize +
	    (alg->cra_alignmask & ~(crypto_tfm_ctx_alignment() - 1));
}

static void crypto_diskcipher_show(struct seq_file *m, struct crypto_alg *alg)
{
	seq_printf(m, "type         : diskcipher\n");
	disckipher_log_show(m);
}

static const struct crypto_type crypto_diskcipher_type = {
	.extsize = crypto_diskcipher_extsize,
	.init_tfm = crypto_diskcipher_init_tfm,
#ifdef CONFIG_PROC_FS
	.show = crypto_diskcipher_show,
#endif
	.maskclear = ~CRYPTO_ALG_TYPE_MASK,
	.maskset = CRYPTO_ALG_TYPE_MASK,
	.type = CRYPTO_ALG_TYPE_DISKCIPHER,
	.tfmsize = offsetof(struct crypto_diskcipher, base),
};

struct crypto_diskcipher *crypto_alloc_diskcipher(const char *alg_name,
			u32 type, u32 mask, bool force)
{
	disckipher_log(DISKC_API_ALLOC, 0, NULL);
	if (force) {
		if (strlen(alg_name) + DISKC_NAME_SIZE < CRYPTO_MAX_ALG_NAME) {
			char diskc_name[CRYPTO_MAX_ALG_NAME];

			strcpy(diskc_name, alg_name);
			strcat(diskc_name, DISKC_NAME);
			return crypto_alloc_tfm(diskc_name,
				&crypto_diskcipher_type, type, mask);
		}
	} else {
		return crypto_alloc_tfm(alg_name, &crypto_diskcipher_type, type, mask);
	}

	return NULL;
}

void crypto_free_diskcipher(struct crypto_diskcipher *tfm)
{
	disckipher_log(DISKC_API_FREE, 0, NULL);
	crypto_destroy_tfm(tfm, crypto_diskcipher_tfm(tfm));
}

int crypto_register_diskcipher(struct diskcipher_alg *alg)
{
	struct crypto_alg *base = &alg->base;

	base->cra_type = &crypto_diskcipher_type;
	base->cra_flags = CRYPTO_ALG_TYPE_DISKCIPHER;
	return crypto_register_alg(base);
}

void crypto_unregister_diskcipher(struct diskcipher_alg *alg)
{
	crypto_unregister_alg(&alg->base);
}

int crypto_register_diskciphers(struct diskcipher_alg *algs, int count)
{
	int i, ret;

	for (i = 0; i < count; i++) {
		ret = crypto_register_diskcipher(algs + i);
		if (ret)
			goto err;
	}
	return 0;

err:
	for (--i; i >= 0; --i)
		crypto_unregister_diskcipher(algs + i);
	return ret;
}

void crypto_unregister_diskciphers(struct diskcipher_alg *algs, int count)
{
	int i;

	for (i = count - 1; i >= 0; --i)
		crypto_unregister_diskcipher(algs + i);
}
