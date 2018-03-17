/*-
 * Copyright (c) 2018 Jian-Hong, Pan <starnight@g.ncu.edu.tw>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce at minimum a disclaimer
 *    similar to the "NO WARRANTY" disclaimer below ("Disclaimer") and any
 *    redistribution must be conditioned upon including a substantially
 *    similar Disclaimer requirement for further binary redistribution.
 * 3. Neither the names of the above-listed copyright holders nor the names
 *    of any contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 *
 * NO WARRANTY
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ''AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF NONINFRINGEMENT, MERCHANTIBILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGES.
 *
 */

#include <linux/scatterlist.h>
#include <crypto/hash.h>
#include <crypto/skcipher.h>
#include "lrwsec.h"

struct crypto_shash *
lrw_mic_key_setup(u8 *k, size_t k_len)
{
	char *algo = "cmac(aes)";
	struct crypto_shash *tfm;
	int err;

	tfm = crypto_alloc_shash(algo, 0, 0);
	if (!IS_ERR(tfm)) {
		err = crypto_shash_setkey(tfm, k, k_len);
		if (err) {
			crypto_free_shash(tfm);
			tfm = NULL;
		}
	}

	return tfm;
}

int
lrw_aes_cmac(struct crypto_shash *tfm, u8 *bz, u8 *data, size_t len, u8 *out)
{
	SHASH_DESC_ON_STACK(desc, tfm);
	int err;

	desc->tfm = tfm;

	err = crypto_shash_init(desc);
	if (err)
		goto lrw_aes_cmac_end;

	err = crypto_shash_update(desc, bz, 16);
	if (err)
		goto lrw_aes_cmac_end;

	err = crypto_shash_update(desc, data, len);
	if (err)
		goto lrw_aes_cmac_end;

	err = crypto_shash_final(desc, out);

lrw_aes_cmac_end:
	return err;
}

int
lrw_set_bzero(u8 dir, u8 *devaddr, u32 fcnt, u8 len, u8 *bz)
{
	__le32 _fcnt = cpu_to_le32(fcnt);

	bz[0] = 0x49;
	memset(bz + 1, 0x00, 4);
	bz[5] = dir;
	memcpy(bz + 6, devaddr, 4);
	memcpy(bz + 10, &_fcnt, 4);
	bz[14] = 0x00;
	bz[15] = len;

	return 0;
}

int
lrw_calc_mic(struct crypto_shash *tfm,
	     u8 dir, u8 *devaddr, u32 fcnt, u8* buf, size_t len, u8 *mic4)
{
	u8 mic[16];
	u8 bz[16];
	int err;

	/* According to LoRaWAN Specification Version 1.0.2
	 * - 4.4 Massege Integrity Code (MIC) */
	lrw_set_bzero(dir, devaddr, fcnt, len, bz);
	err = lrw_aes_cmac(tfm, bz, buf, len, mic);
	if (!err)
		memcpy(mic4, mic, 4);

	return err;
}

void
lrw_mic_key_free(struct crypto_shash *tfm)
{
	crypto_free_shash(tfm);
}

struct crypto_skcipher *
lrw_aes_enc_key_setup(char *algo, u8 *k, size_t k_len)
{
	struct crypto_skcipher *tfm;
	int err;

	tfm = crypto_alloc_skcipher(algo, 0, CRYPTO_ALG_ASYNC);
	if (!IS_ERR(tfm)) {
		err = crypto_skcipher_setkey(tfm, k, k_len);
		if (err) {
			crypto_free_skcipher(tfm);
			tfm = NULL;
		}
	}

	return tfm;
}

struct crypto_skcipher *
lrw_encrypt_key_setup(u8 *k, size_t k_len)
{
	return lrw_aes_enc_key_setup("cbc(aes)", k, k_len);
}

int
lrw_aes_enc(struct crypto_skcipher *tfm, u8 *in, size_t len, u8 *out)
{
	u8 iv[16];
	struct scatterlist src, dst;
	SKCIPHER_REQUEST_ON_STACK(req, tfm);
	int err;

	memset(iv, 0, 16);
	/* The buffer for sg_init_one cannot be a global or const local
	 * (will confuse the scatterlist) */
	sg_init_one(&src, in, len);
	sg_init_one(&dst, out, len);

	skcipher_request_set_tfm(req, tfm);
	skcipher_request_set_callback(req, 0, NULL, NULL);
	skcipher_request_set_crypt(req, &src, &dst, len, iv);
	err = crypto_skcipher_encrypt(req);
	skcipher_request_zero(req);

	return err;
}

#define	LRW_SEQUENCE_OF_BLOCK_LEN	16

int
lrw_set_sob(u8 dir, u8 *devaddr, u32 fcnt, u8 index, u8 *sob)
{
	__le32 _fcnt = cpu_to_le32(fcnt);

	sob[0] = 0x01;
	memset(sob + 1, 0x00, 4);
	sob[5] = dir;
	memcpy(sob + 6, devaddr, 4);
	memcpy(sob + 10, &_fcnt, 4);
	sob[14] = 0x00;
	sob[15] = index;

	return 0;
}

int
lrw_encrypt_sob(struct crypto_skcipher *tfm, u8 *sob)
{
	return lrw_aes_enc(tfm, sob, LRW_SEQUENCE_OF_BLOCK_LEN, sob);
}

int
lrw_encrypt_buf(struct crypto_skcipher *tfm,
		u8 dir, u8 *devaddr, u32 fcnt, u8 *buf, size_t len)
{
	u8 sob[LRW_SEQUENCE_OF_BLOCK_LEN];
	u8 i, j;

	/* According to LoRaWAN Specification Version 1.0.2
	 * - 4.3.3 MAC Frame Payload Encryption (FRMPayload) */
	for (i = 0; (i * LRW_SEQUENCE_OF_BLOCK_LEN) < len; i++) {
		lrw_set_sob(dir, devaddr, fcnt, i, sob);
		lrw_encrypt_sob(tfm, sob);
		for (j = 0; (i * LRW_SEQUENCE_OF_BLOCK_LEN + j) < len; j++)
			buf[i * LRW_SEQUENCE_OF_BLOCK_LEN + j] ^= sob[j];
	}

	return 0;
}

int
lrw_decrypt_buf(struct crypto_skcipher *tfm,
		u8 dir, u8 *devaddr, u32 fcnt, u8 *buf, size_t len)
{
	/* Accoding to XOR swap algorithm */
	return lrw_encrypt_buf(tfm, dir, devaddr, fcnt, buf, len);
}

void
lrw_aes_enc_key_free(struct crypto_skcipher *tfm)
{
	pr_debug("lrwsec: %s\n", __func__);
	crypto_free_skcipher(tfm);
}

void
lrw_encrypt_key_free(struct crypto_skcipher *tfm)
{
	pr_debug("lrwsec: %s\n", __func__);
	lrw_aes_enc_key_free(tfm);
}
