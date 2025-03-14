/****************************************************************************
 * crypto/xform.c
 *
 * SPDX-License-Identifier: 0BSD
 * SPDX-FileCopyrightText: 1995, 1996, 1997, 1998, 1999 John Ioannidis
 * SPDX-FileCopyrightText: 1995, 1996, 1997, 1998, 1999 Angelos D. Keromytis
 * SPDX-FileCopyrightText: 1995, 1996, 1997, 1998, 1999 Niels Provos.
 * SPDX-FileCopyrightText: 2001 Angelos D. Keromytis.
 * SPDX-FileCopyrightText: 2008 Damien Miller
 * SPDX-FileCopyrightText: 2010, 2015 Mike Belopuhov
 * SPDX-FileContributor: John Ioannidis (ji@tla.org)
 * SPDX-FileContributor: Angelos D. Keromytis (kermit@csd.uch.gr)
 * SPDX-FileContributor: Niels Provos (provos@physnet.uni-hamburg.de)
 * SPDX-FileContributor: Damien Miller (djm@mindrot.org)
 * SPDX-FileContributor: Mike Belopuhov (mikeb@openbsd.org)
 *
 * The authors of this code are John Ioannidis (ji@tla.org),
 * Angelos D. Keromytis (kermit@csd.uch.gr),
 * Niels Provos (provos@physnet.uni-hamburg.de),
 * Damien Miller (djm@mindrot.org) and
 * Mike Belopuhov (mikeb@openbsd.org).
 *
 * This code was written by John Ioannidis for BSD/OS in Athens, Greece,
 * in November 1995.
 *
 * Ported to OpenBSD and NetBSD, with additional transforms,
 * in December 1996,
 * by Angelos D. Keromytis.
 *
 * Additional transforms and features in 1997 and 1998 by
 * Angelos D. Keromytis and Niels Provos.
 *
 * Additional features in 1999 by Angelos D. Keromytis.
 *
 * AES XTS implementation in 2008 by Damien Miller
 *
 * AES-GCM-16 and Chacha20-Poly1305 AEAD modes by Mike Belopuhov.
 *
 * Copyright (C) 1995, 1996, 1997, 1998, 1999 by John Ioannidis,
 * Angelos D. Keromytis and Niels Provos.
 *
 * Copyright (C) 2001, Angelos D. Keromytis.
 *
 * Copyright (C) 2008, Damien Miller
 *
 * Copyright (C) 2010, 2015, Mike Belopuhov
 *
 * Permission to use, copy, and modify this software with or without fee
 * is hereby granted, provided that this entire notice is included in
 * all copies of any software which is or includes a copy or
 * modification of this software.
 * You may use this code under the GNU public license if you so wish. Please
 * contribute changes back to the authors under this freer than GPL license
 * so that we may further the use of strong encryption without limitations to
 * all.
 *
 * THIS SOFTWARE IS BEING PROVIDED "AS IS", WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTY. IN PARTICULAR, NONE OF THE AUTHORS MAKES ANY
 * REPRESENTATION OR WARRANTY OF ANY KIND CONCERNING THE
 * MERCHANTABILITY OF THIS SOFTWARE OR ITS FITNESS FOR ANY PARTICULAR
 * PURPOSE.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <errno.h>
#include <string.h>
#include <strings.h>
#include <sys/param.h>
#include <sys/time.h>

#include <crypto/md5.h>
#include <crypto/sha1.h>
#include <crypto/sha2.h>
#include <crypto/rmd160.h>
#include <crypto/blf.h>
#include <crypto/cast.h>
#include <crypto/rijndael.h>
#include <crypto/aes.h>
#include <crypto/cryptodev.h>
#include <crypto/xform.h>
#include <crypto/gmac.h>
#include <crypto/cmac.h>
#include <crypto/chachapoly.h>
#include <crypto/poly1305.h>
#include <nuttx/crc32.h>

#include "des_locl.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CRC32_XOR_VALUE 0xFFFFFFFFUL

/****************************************************************************
 * Public Functions
 ****************************************************************************/

extern void des_ecb3_encrypt(caddr_t, caddr_t, caddr_t,
                             caddr_t, caddr_t, int);

int des_set_key(FAR void *, caddr_t);
int des3_setkey(FAR void *, FAR uint8_t *, int);
int blf_setkey(FAR void *, FAR uint8_t *, int);
int cast5_setkey(FAR void *, FAR uint8_t *, int);
int aes_setkey_xform(FAR void *, FAR uint8_t *, int);
int aes_ctr_setkey(FAR void *, FAR uint8_t *, int);
int aes_xts_setkey(FAR void *, FAR uint8_t *, int);
int aes_ofb_setkey(FAR void *, FAR uint8_t *, int);
int null_setkey(FAR void *, FAR uint8_t *, int);

void des3_encrypt(caddr_t, FAR uint8_t *);
void blf_encrypt(caddr_t, FAR uint8_t *);
void cast5_encrypt(caddr_t, FAR uint8_t *);
void aes_encrypt_xform(caddr_t, FAR uint8_t *);
void null_encrypt(caddr_t, FAR uint8_t *);
void aes_xts_encrypt(caddr_t, FAR uint8_t *);
void aes_ofb_encrypt(caddr_t, FAR uint8_t *);
void aes_cfb8_encrypt(caddr_t, FAR uint8_t *);
void aes_cfb128_encrypt(caddr_t, FAR uint8_t *);

void des3_decrypt(caddr_t, FAR uint8_t *);
void blf_decrypt(caddr_t, FAR uint8_t *);
void cast5_decrypt(caddr_t, FAR uint8_t *);
void aes_decrypt_xform(caddr_t, FAR uint8_t *);
void null_decrypt(caddr_t, FAR uint8_t *);
void aes_xts_decrypt(caddr_t, FAR uint8_t *);
void aes_cfb8_decrypt(caddr_t, FAR uint8_t *);
void aes_cfb128_decrypt(caddr_t, FAR uint8_t *);

void aes_ctr_crypt(caddr_t, FAR uint8_t *);

void aes_ctr_reinit(caddr_t, FAR uint8_t *);
void aes_xts_reinit(caddr_t, FAR uint8_t *);
void aes_gcm_reinit(caddr_t, FAR uint8_t *);
void aes_ofb_reinit(caddr_t, FAR uint8_t *);

void null_init(FAR void *);
void poly1305_setkey(FAR void *, FAR const uint8_t *, uint16_t);
int poly1305update_int(FAR void *, FAR const uint8_t *, size_t);
int poly1305_final(FAR uint8_t *, FAR void *);
int md5update_int(FAR void *, FAR const uint8_t *, size_t);
int sha1update_int(FAR void *, FAR const uint8_t *, size_t);
int rmd160update_int(FAR void *, FAR const uint8_t *, size_t);
int sha224update_int(FAR void *, FAR const uint8_t *, size_t);
int sha256update_int(FAR void *, FAR const uint8_t *, size_t);
int sha384update_int(FAR void *, FAR const uint8_t *, size_t);
int sha512update_int(FAR void *, FAR const uint8_t *, size_t);
void crc32setkey(FAR void *, FAR const uint8_t *, uint16_t);
int crc32update(FAR void *, FAR const uint8_t *, size_t);
void crc32final(FAR uint8_t *, FAR void *);

struct aes_ctr_ctx
{
  AES_CTX ac_key;
  uint8_t ac_block[AESCTR_BLOCKSIZE];
};

struct aes_xts_ctx
{
  rijndael_ctx key1;
  rijndael_ctx key2;
  uint8_t tweak[AES_XTS_BLOCKSIZE];
};

struct aes_ofb_ctx
{
  AES_CTX ac_key;
  FAR uint8_t *iv;
};

/* Helper */

void aes_xts_crypt(FAR struct aes_xts_ctx *, FAR uint8_t *, u_int);

/* Encryption instances */

const struct enc_xform enc_xform_3des =
{
  CRYPTO_3DES_CBC, "3DES",
  8, 8, 24, 24, 384,
  des3_encrypt,
  des3_decrypt,
  des3_setkey,
  NULL
};

const struct enc_xform enc_xform_blf =
{
  CRYPTO_BLF_CBC, "Blowfish",
  8, 8, 5, 56 /* 448 bits, max key */,
  sizeof(blf_ctx),
  blf_encrypt,
  blf_decrypt,
  blf_setkey,
  NULL
};

const struct enc_xform enc_xform_cast5 =
{
  CRYPTO_CAST_CBC, "CAST-128",
  8, 8, 5, 16,
  sizeof(cast_key),
  cast5_encrypt,
  cast5_decrypt,
  cast5_setkey,
  NULL
};

const struct enc_xform enc_xform_aes =
{
  CRYPTO_AES_CBC, "AES",
  16, 16, 16, 32,
  sizeof(AES_CTX),
  aes_encrypt_xform,
  aes_decrypt_xform,
  aes_setkey_xform,
  NULL
};

const struct enc_xform enc_xform_aes_ctr =
{
  CRYPTO_AES_CTR, "AES-CTR",
  16, 8, 16 + 4, 32 + 4,
  sizeof(struct aes_ctr_ctx),
  aes_ctr_crypt,
  aes_ctr_crypt,
  aes_ctr_setkey,
  aes_ctr_reinit
};

const struct enc_xform enc_xform_aes_gcm =
{
  CRYPTO_AES_GCM_16, "AES-GCM",
  1, 8, 16 + 4, 32 + 4,
  sizeof(struct aes_ctr_ctx),
  aes_ctr_crypt,
  aes_ctr_crypt,
  aes_ctr_setkey,
  aes_gcm_reinit
};

const struct enc_xform enc_xform_aes_gmac =
{
  CRYPTO_AES_GMAC, "AES-GMAC",
  1, 8, 16 + 4, 32 + 4, 0,
  NULL,
  NULL,
  NULL,
  NULL
};

const struct enc_xform enc_xform_aes_cmac =
{
  CRYPTO_AES_CMAC, "AES-CMAC",
  1, 0, 16, 32, 0,
  null_encrypt,
  null_decrypt,
  null_setkey,
  NULL
};

const struct enc_xform enc_xform_aes_xts =
{
  CRYPTO_AES_XTS, "AES-XTS",
  16, 8, 32, 64,
  sizeof(struct aes_xts_ctx),
  aes_xts_encrypt,
  aes_xts_decrypt,
  aes_xts_setkey,
  aes_xts_reinit
};

const struct enc_xform enc_xform_aes_ofb =
{
  CRYPTO_AES_OFB, "AES-OFB",
  16, 16, 16, 32,
  sizeof(struct aes_ofb_ctx),
  aes_ofb_encrypt,
  aes_ofb_encrypt,
  aes_ofb_setkey,
  aes_ofb_reinit
};

const struct enc_xform enc_xform_aes_cfb_8 =
{
  CRYPTO_AES_CFB_8, "AES-CFB-8",
  16, 16, 16, 32,
  sizeof(struct aes_ofb_ctx),
  aes_cfb8_encrypt,
  aes_cfb8_decrypt,
  aes_ofb_setkey,
  aes_ofb_reinit
};

const struct enc_xform enc_xform_aes_cfb_128 =
{
  CRYPTO_AES_CFB_128, "AES-CFB-128",
  16, 16, 16, 32,
  sizeof(struct aes_ofb_ctx),
  aes_cfb128_encrypt,
  aes_cfb128_decrypt,
  aes_ofb_setkey,
  aes_ofb_reinit
};

const struct enc_xform enc_xform_chacha20_poly1305 =
{
  CRYPTO_CHACHA20_POLY1305, "CHACHA20-POLY1305",
  1, 8, 32 + 4, 32 + 4,
  sizeof(struct chacha20_ctx),
  chacha20_crypt,
  chacha20_crypt,
  chacha20_setkey,
  chacha20_reinit
};

const struct enc_xform enc_xform_null =
{
  CRYPTO_NULL, "NULL",
  4, 0, 0, 256, 0,
  null_encrypt,
  null_decrypt,
  null_setkey,
  NULL
};

/* Authentication instances */

const struct auth_hash auth_hash_hmac_md5_96 =
{
  CRYPTO_MD5_HMAC, "HMAC-MD5",
  16, 16, 12, sizeof(MD5_CTX), HMAC_MD5_BLOCK_LEN,
  (void (*) (FAR void *)) md5init, NULL, NULL,
  md5update_int,
  (void (*) (FAR uint8_t *, FAR void *)) md5final
};

const struct auth_hash auth_hash_hmac_sha1_96 =
{
  CRYPTO_SHA1_HMAC, "HMAC-SHA1",
  20, 20, 12, sizeof(SHA1_CTX), HMAC_SHA1_BLOCK_LEN,
  (void (*) (FAR void *)) sha1init, NULL, NULL,
  sha1update_int,
  (void (*) (FAR uint8_t *, FAR void *)) sha1final
};

const struct auth_hash auth_hash_hmac_ripemd_160_96 =
{
  CRYPTO_RIPEMD160_HMAC, "HMAC-RIPEMD-160",
  20, 20, 12, sizeof(RMD160_CTX), HMAC_RIPEMD160_BLOCK_LEN,
  (void (*)(FAR void *)) rmd160init, NULL, NULL,
  rmd160update_int,
  (void (*)(FAR uint8_t *, FAR void *)) rmd160final
};

const struct auth_hash auth_hash_hmac_sha2_256_128 =
{
  CRYPTO_SHA2_256_HMAC, "HMAC-SHA2-256",
  32, 32, 16, sizeof(SHA2_CTX), HMAC_SHA2_256_BLOCK_LEN,
  (void (*)(FAR void *)) sha256init, NULL, NULL,
  sha256update_int,
  (void (*)(FAR uint8_t *, FAR void *)) sha256final
};

const struct auth_hash auth_hash_hmac_sha2_384_192 =
{
  CRYPTO_SHA2_384_HMAC, "HMAC-SHA2-384",
  48, 48, 24, sizeof(SHA2_CTX), HMAC_SHA2_384_BLOCK_LEN,
  (void (*)(FAR void *)) sha384init, NULL, NULL,
  sha384update_int,
  (void (*)(FAR uint8_t *, FAR void *)) sha384final
};

const struct auth_hash auth_hash_hmac_sha2_512_256 =
{
  CRYPTO_SHA2_512_HMAC, "HMAC-SHA2-512",
  64, 64, 32, sizeof(SHA2_CTX), HMAC_SHA2_512_BLOCK_LEN,
  (void (*)(FAR void *)) sha512init, NULL, NULL,
  sha512update_int,
  (void (*)(FAR uint8_t *, FAR void *)) sha512final
};

const struct auth_hash auth_hash_gmac_aes_128 =
{
  CRYPTO_AES_128_GMAC, "GMAC-AES-128",
  16 + 4, GMAC_BLOCK_LEN, GMAC_DIGEST_LEN, sizeof(AES_GMAC_CTX),
  AESCTR_BLOCKSIZE, aes_gmac_init, aes_gmac_setkey, aes_gmac_reinit,
  aes_gmac_update, aes_gmac_final
};

const struct auth_hash auth_hash_gmac_aes_192 =
{
  CRYPTO_AES_192_GMAC, "GMAC-AES-192",
  24 + 4, GMAC_BLOCK_LEN, GMAC_DIGEST_LEN, sizeof(AES_GMAC_CTX),
  AESCTR_BLOCKSIZE, aes_gmac_init, aes_gmac_setkey, aes_gmac_reinit,
  aes_gmac_update, aes_gmac_final
};

const struct auth_hash auth_hash_gmac_aes_256 =
{
  CRYPTO_AES_256_GMAC, "GMAC-AES-256",
  32 + 4, GMAC_BLOCK_LEN, GMAC_DIGEST_LEN, sizeof(AES_GMAC_CTX),
  AESCTR_BLOCKSIZE, aes_gmac_init, aes_gmac_setkey, aes_gmac_reinit,
  aes_gmac_update, aes_gmac_final
};

const struct auth_hash auth_hash_chacha20_poly1305 =
{
  CRYPTO_CHACHA20_POLY1305_MAC, "CHACHA20-POLY1305",
  CHACHA20_KEYSIZE + CHACHA20_SALT, POLY1305_BLOCK_LEN, POLY1305_TAGLEN,
  sizeof(CHACHA20_POLY1305_CTX), CHACHA20_BLOCK_LEN,
  chacha20_poly1305_init, chacha20_poly1305_setkey,
  chacha20_poly1305_reinit, chacha20_poly1305_update,
  chacha20_poly1305_final
};

const struct auth_hash auth_hash_cmac_aes_128 =
{
  CRYPTO_AES_128_CMAC, "CMAC-AES-128",
  16, AES_CMAC_KEY_LENGTH, AES_CMAC_DIGEST_LENGTH, sizeof(AES_CMAC_CTX),
  AESCTR_BLOCKSIZE, (void (*)(FAR void *)) aes_cmac_init,
  (void (*)(FAR void *, FAR const uint8_t *, uint16_t)) aes_cmac_setkey,
  NULL, (int (*)(FAR void *, FAR const uint8_t *, size_t)) aes_cmac_update,
  (void (*) (FAR uint8_t *, FAR void *)) aes_cmac_final
};

const struct auth_hash auth_hash_md5 =
{
  CRYPTO_MD5, "MD5",
  0, 16, 16, sizeof(MD5_CTX), HMAC_MD5_BLOCK_LEN,
  (void (*) (FAR void *)) md5init, NULL, NULL,
  md5update_int,
  (void (*) (FAR uint8_t *, FAR void *)) md5final
};

const struct auth_hash auth_hash_poly1305 =
{
  CRYPTO_POLY1305, "POLY1305",
  0, 16, 16, sizeof(poly1305_state), poly1305_block_size,
  (void (*) (FAR void *)) null_init, poly1305_setkey, NULL,
  poly1305update_int,
  (void (*) (FAR uint8_t *, FAR void *)) poly1305_final
};

const struct auth_hash auth_hash_ripemd_160 =
{
  CRYPTO_RIPEMD160, "RIPEMD160",
  0, 20, 20, sizeof(RMD160_CTX), HMAC_RIPEMD160_BLOCK_LEN,
  (void (*) (FAR void *)) rmd160init, NULL, NULL,
  rmd160update_int,
  (void (*) (FAR uint8_t *, FAR void *)) rmd160final
};

const struct auth_hash auth_hash_sha1 =
{
  CRYPTO_SHA1, "SHA1",
  0, 20, 20, sizeof(SHA1_CTX), HMAC_SHA1_BLOCK_LEN,
  (void (*) (FAR void *)) sha1init, NULL, NULL,
  sha1update_int,
  (void (*) (FAR uint8_t *, FAR void *)) sha1final
};

const struct auth_hash auth_hash_sha2_224 =
{
  CRYPTO_SHA2_224, "SHA2-224",
  0, 28, 16, sizeof(SHA2_CTX), SHA224_BLOCK_LENGTH,
  (void (*)(FAR void *)) sha224init, NULL, NULL,
  sha224update_int,
  (void (*)(FAR uint8_t *, FAR void *)) sha224final
};

const struct auth_hash auth_hash_sha2_256 =
{
  CRYPTO_SHA2_256, "SHA2-256",
  0, 32, 16, sizeof(SHA2_CTX), HMAC_SHA2_256_BLOCK_LEN,
  (void (*)(FAR void *)) sha256init, NULL, NULL,
  sha256update_int,
  (void (*)(FAR uint8_t *, FAR void *)) sha256final
};

const struct auth_hash auth_hash_sha2_384 =
{
  CRYPTO_SHA2_384, "SHA2-384",
  0, 48, 24, sizeof(SHA2_CTX), HMAC_SHA2_384_BLOCK_LEN,
  (void (*)(FAR void *)) sha384init, NULL, NULL,
  sha384update_int,
  (void (*)(FAR uint8_t *, FAR void *)) sha384final
};

const struct auth_hash auth_hash_sha2_512 =
{
  CRYPTO_SHA2_512, "SHA2-512",
  0, 64, 32, sizeof(SHA2_CTX), HMAC_SHA2_512_BLOCK_LEN,
  (void (*)(FAR void *)) sha512init, NULL, NULL,
  sha512update_int,
  (void (*)(FAR uint8_t *, FAR void *)) sha512final
};

const struct auth_hash auth_hash_crc32 =
{
  CRYPTO_CRC32, "CRC32",
  0, 32, 0, sizeof(uint32_t), 1,
  null_init, crc32setkey, NULL, crc32update, crc32final
};

/* Encryption wrapper routines. */

void des3_encrypt(caddr_t key, FAR uint8_t *blk)
{
  des_ecb3_encrypt((caddr_t)blk, (caddr_t)blk, key, key + 128, key + 256, 1);
}

void des3_decrypt(caddr_t key, FAR uint8_t *blk)
{
  des_ecb3_encrypt((caddr_t)blk, (caddr_t)blk, key + 256, key + 128, key, 0);
}

int des3_setkey(FAR void *sched, FAR uint8_t *key, int len)
{
  if (des_set_key(key, sched) < 0 || des_set_key(key + 8, sched + 128)
      < 0 || des_set_key(key + 16, sched + 256) < 0)
    {
      return -1;
    }

  return 0;
}

void blf_encrypt(caddr_t key, FAR uint8_t *blk)
{
  blf_ecb_encrypt((FAR blf_ctx *) key, blk, 8);
}

void blf_decrypt(caddr_t key, FAR uint8_t *blk)
{
  blf_ecb_decrypt((FAR blf_ctx *) key, blk, 8);
}

int blf_setkey(FAR void *sched, FAR uint8_t *key, int len)
{
  blf_key((FAR blf_ctx *)sched, key, len);

  return 0;
}

int null_setkey(FAR void *sched, FAR uint8_t *key, int len)
{
  return 0;
}

void null_encrypt(caddr_t key, FAR uint8_t *blk)
{
}

void null_decrypt(caddr_t key, FAR uint8_t *blk)
{
}

void cast5_encrypt(caddr_t key, FAR uint8_t *blk)
{
  cast_encrypt((FAR cast_key *) key, blk, blk);
}

void cast5_decrypt(caddr_t key, FAR uint8_t *blk)
{
  cast_decrypt((FAR cast_key *) key, blk, blk);
}

int cast5_setkey(FAR void *sched, FAR uint8_t *key, int len)
{
  cast_setkey((FAR cast_key *)sched, key, len);

  return 0;
}

void aes_encrypt_xform(caddr_t key, FAR uint8_t *blk)
{
  aes_encrypt((FAR AES_CTX *)key, blk, blk);
}

void aes_decrypt_xform(caddr_t key, FAR uint8_t *blk)
{
  aes_decrypt((FAR AES_CTX *)key, blk, blk);
}

int aes_setkey_xform(FAR void *sched, FAR uint8_t *key, int len)
{
  return aes_setkey((FAR AES_CTX *)sched, key, len);
}

void aes_ctr_reinit(caddr_t key, FAR uint8_t *iv)
{
  FAR struct aes_ctr_ctx *ctx;

  ctx = (FAR struct aes_ctr_ctx *)key;
  bcopy(iv, ctx->ac_block + AESCTR_NONCESIZE, AESCTR_IVSIZE);

  /* reset counter */

  bzero(ctx->ac_block + AESCTR_NONCESIZE + AESCTR_IVSIZE, 4);
}

void aes_gcm_reinit(caddr_t key, FAR uint8_t *iv)
{
  FAR struct aes_ctr_ctx *ctx;

  ctx = (FAR struct aes_ctr_ctx *)key;
  bcopy(iv, ctx->ac_block + AESCTR_NONCESIZE, AESCTR_IVSIZE);

  /* reset counter */

  bzero(ctx->ac_block + AESCTR_NONCESIZE + AESCTR_IVSIZE, 4);
  ctx->ac_block[AESCTR_BLOCKSIZE - 1] = 1; /* GCM starts with 1 */
}

void aes_ctr_crypt(caddr_t key, FAR uint8_t *data)
{
  FAR struct aes_ctr_ctx *ctx;
  uint8_t keystream[AESCTR_BLOCKSIZE];
  int i;

  ctx = (FAR struct aes_ctr_ctx *)key;

  /* increment counter */

  for (i = AESCTR_BLOCKSIZE - 1;
        i >= AESCTR_NONCESIZE + AESCTR_IVSIZE; i--)
    {
      /* continue on overflow */

      if (++ctx->ac_block[i])
        {
          break;
        }
    }

  aes_encrypt(&ctx->ac_key, ctx->ac_block, keystream);
  for (i = 0; i < AESCTR_BLOCKSIZE; i++)
    {
      data[i] ^= keystream[i];
    }

  explicit_bzero(keystream, sizeof(keystream));
}

int aes_ctr_setkey(FAR void *sched, FAR uint8_t *key, int len)
{
  FAR struct aes_ctr_ctx *ctx;

  if (len < AESCTR_NONCESIZE)
    {
      return -1;
    }

  ctx = (FAR struct aes_ctr_ctx *)sched;
  if (aes_setkey(&ctx->ac_key, key, len - AESCTR_NONCESIZE) != 0)
    {
      return -1;
    }

  bcopy(key + len - AESCTR_NONCESIZE, ctx->ac_block, AESCTR_NONCESIZE);
  return 0;
}

void aes_xts_reinit(caddr_t key, FAR uint8_t *iv)
{
  FAR struct aes_xts_ctx *ctx = (FAR struct aes_xts_ctx *)key;
  uint64_t blocknum;
  u_int i;

  /* Prepare tweak as E_k2(IV). IV is specified as LE representation
   * of a 64-bit block number which we allow to be passed in directly.
   */

  memcpy(&blocknum, iv, AES_XTS_IVSIZE);
  for (i = 0; i < AES_XTS_IVSIZE; i++)
    {
      ctx->tweak[i] = blocknum & 0xff;
      blocknum >>= 8;
    }

  /* Last 64 bits of IV are always zero */

  bzero(ctx->tweak + AES_XTS_IVSIZE, AES_XTS_IVSIZE);

  rijndael_encrypt(&ctx->key2, ctx->tweak, ctx->tweak);
}

void aes_xts_crypt(FAR struct aes_xts_ctx *ctx,
                   FAR uint8_t *data,
                   u_int do_encrypt)
{
  uint8_t block[AES_XTS_BLOCKSIZE];
  u_int i;
  u_int carry_in;
  u_int carry_out;

  for (i = 0; i < AES_XTS_BLOCKSIZE; i++)
    {
      block[i] = data[i] ^ ctx->tweak[i];
    }

  if (do_encrypt)
    {
      rijndael_encrypt(&ctx->key1, block, data);
    }
  else
    {
      rijndael_decrypt(&ctx->key1, block, data);
    }

  for (i = 0; i < AES_XTS_BLOCKSIZE; i++)
    {
      data[i] ^= ctx->tweak[i];
    }

  /* Exponentiate tweak */

  carry_in = 0;
  for (i = 0; i < AES_XTS_BLOCKSIZE; i++)
    {
      carry_out = ctx->tweak[i] & 0x80;
      ctx->tweak[i] = (ctx->tweak[i] << 1) | carry_in;
      carry_in = carry_out >> 7;
    }

  ctx->tweak[0] ^= (AES_XTS_ALPHA & -carry_in);
  explicit_bzero(block, sizeof(block));
}

void aes_xts_encrypt(caddr_t key, FAR uint8_t *data)
{
  aes_xts_crypt((FAR struct aes_xts_ctx *)key, data, 1);
}

void aes_xts_decrypt(caddr_t key, FAR uint8_t *data)
{
  aes_xts_crypt((FAR struct aes_xts_ctx *)key, data, 0);
}

int aes_xts_setkey(FAR void *sched, FAR uint8_t *key, int len)
{
  FAR struct aes_xts_ctx *ctx;

  if (len != 32 && len != 64)
    {
      return -1;
    }

  ctx = (FAR struct aes_xts_ctx *)sched;

  rijndael_set_key(&ctx->key1, key, len * 4);
  rijndael_set_key(&ctx->key2, key + (len / 2), len * 4);

  return 0;
}

void aes_ofb_encrypt(caddr_t key, FAR uint8_t *data)
{
  FAR struct aes_ofb_ctx *ctx;
  int i;

  ctx = (FAR struct aes_ofb_ctx *)key;

  aes_encrypt(&ctx->ac_key, ctx->iv, ctx->iv);
  for (i = 0; i < AESOFB_IVSIZE; i++)
    {
      data[i] ^= ctx->iv[i];
    }
}

int aes_ofb_setkey(FAR void *sched, FAR uint8_t *key, int len)
{
  FAR struct aes_ofb_ctx *ctx;

  ctx = (FAR struct aes_ofb_ctx *)sched;
  if (aes_setkey(&ctx->ac_key, key, len) != 0)
    {
      return -1;
    }

  return 0;
}

void aes_ofb_reinit(caddr_t key, FAR uint8_t *iv)
{
  FAR struct aes_ofb_ctx *ctx;

  ctx = (FAR struct aes_ofb_ctx *)key;
  ctx->iv = iv;
}

void aes_cfb8_encrypt(caddr_t key, FAR uint8_t *data)
{
  FAR struct aes_ofb_ctx *ctx;
  uint8_t ov[AESOFB_IVSIZE + 1];
  int i;

  ctx = (FAR struct aes_ofb_ctx *)key;

  for (i = 0; i < AESOFB_IVSIZE; i++)
    {
      bcopy(ctx->iv, ov, AESOFB_IVSIZE);
      aes_encrypt(&ctx->ac_key, ctx->iv, ctx->iv);
      data[i] ^= ctx->iv[0];
      ov[AESOFB_IVSIZE] = data[i];
      bcopy(ov + 1, ctx->iv, AESOFB_IVSIZE);
    }
}

void aes_cfb8_decrypt(caddr_t key, FAR uint8_t *data)
{
  FAR struct aes_ofb_ctx *ctx;
  uint8_t ov[AESOFB_IVSIZE + 1];
  int i;

  ctx = (FAR struct aes_ofb_ctx *)key;

  for (i = 0; i < AESOFB_IVSIZE; i++)
    {
      bcopy(ctx->iv, ov, AESOFB_IVSIZE);
      aes_encrypt(&ctx->ac_key, ctx->iv, ctx->iv);
      ov[AESOFB_IVSIZE] = data[i];
      data[i] ^= ctx->iv[0];
      bcopy(ov + 1, ctx->iv, AESOFB_IVSIZE);
    }
}

void aes_cfb128_encrypt(caddr_t key, FAR uint8_t *data)
{
  FAR struct aes_ofb_ctx *ctx;
  int i;

  ctx = (FAR struct aes_ofb_ctx *)key;

  aes_encrypt(&ctx->ac_key, ctx->iv, ctx->iv);
  for (i = 0; i < AESOFB_IVSIZE; i++)
    {
      data[i] ^= ctx->iv[i];
      ctx->iv[i] = data[i];
    }
}

void aes_cfb128_decrypt(caddr_t key, FAR uint8_t *data)
{
  FAR struct aes_ofb_ctx *ctx;
  uint8_t c;
  int i;

  ctx = (FAR struct aes_ofb_ctx *)key;

  aes_encrypt(&ctx->ac_key, ctx->iv, ctx->iv);
  for (i = 0; i < AESOFB_IVSIZE; i++)
    {
      c = data[i];
      data[i] ^= ctx->iv[i];
      ctx->iv[i] = c;
    }
}

/* And now for auth. */

void null_init(FAR void *ctx)
{
}

void poly1305_setkey(FAR void *sched, FAR const uint8_t *key, uint16_t len)
{
  FAR struct poly1305_state *ctx;

  ctx = (FAR struct poly1305_state *)sched;
  poly1305_begin(ctx, key);
}

int poly1305update_int(FAR void *ctx, FAR const uint8_t *buf, size_t len)
{
  poly1305_update(ctx, buf, len);
  return 0;
}

int poly1305_final(FAR uint8_t *digest, FAR void *ctx)
{
  poly1305_finish(ctx, digest);
  return 0;
}

int rmd160update_int(FAR void *ctx, FAR const uint8_t *buf, size_t len)
{
  rmd160update(ctx, buf, len);
  return 0;
}

int md5update_int(FAR void *ctx, FAR const uint8_t *buf, size_t len)
{
  md5update(ctx, buf, len);
  return 0;
}

int sha1update_int(FAR void *ctx, FAR const uint8_t *buf, size_t len)
{
  sha1update(ctx, buf, len);
  return 0;
}

int sha224update_int(FAR void *ctx, FAR const uint8_t *buf, size_t len)
{
  sha224update(ctx, buf, len);
  return 0;
}

int sha256update_int(FAR void *ctx, FAR const uint8_t *buf, size_t len)
{
  sha256update(ctx, buf, len);
  return 0;
}

int sha384update_int(FAR void *ctx, FAR const uint8_t *buf, size_t len)
{
  sha384update(ctx, buf, len);
  return 0;
}

int sha512update_int(FAR void *ctx, FAR const uint8_t *buf, size_t len)
{
  sha512update(ctx, buf, len);
  return 0;
}

void crc32setkey(FAR void *ctx, FAR const uint8_t *key, uint16_t len)
{
  FAR uint32_t *val = (FAR uint32_t *)key;
  uint32_t tmp = (*val) ^ CRC32_XOR_VALUE;
  memcpy(ctx, &tmp, len);
}

int crc32update(FAR void *ctx, FAR const uint8_t *buf, size_t len)
{
  FAR uint32_t *startval = (FAR uint32_t *)ctx;
  *startval = crc32part(buf, len, *startval);
  return 0;
}

void crc32final(FAR uint8_t *digest, FAR void *ctx)
{
  FAR uint32_t *val = (FAR uint32_t *)ctx;
  uint32_t result = (*val) ^ CRC32_XOR_VALUE;
  memcpy(digest, &result, sizeof(uint32_t));
}
