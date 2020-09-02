/* LibTomCrypt, modular cryptographic library -- Tom St Denis
 *
 * LibTomCrypt is a library that provides various cryptographic
 * algorithms in a highly modular and flexible manner.
 *
 * The library is free for all purposes without any express
 * guarantee it works.
 */
#include "tomcrypt_private.h"

/**
  @file ctr_setiv.c
  CTR implementation, set IV, Tom St Denis
*/

#ifdef LTC_CTR_MODE

/**
   Set an initialization vector
   @param IV   The initialization vector
   @param len  The length of the vector (in octets)
   @param ctr  The CTR state
   @return CRYPT_OK if successful
*/
int ctr_setiv(const unsigned char *IV, unsigned long len, symmetric_CTR *ctr)
{
   int err;

   LTC_ARGCHK(IV  != NULL);
   LTC_ARGCHK(ctr != NULL);

   /* bad param? */
   if ((err = cipher_is_valid(ctr->cipher)) != CRYPT_OK) {
      return err;
   }

   if (len != (unsigned long)ctr->blocklen) {
      return CRYPT_INVALID_ARG;
   }

   /* set IV */
   XMEMCPY(ctr->ctr, IV, len);

   /* force next block */
   ctr->padlen = 0;
   return cipher_descriptor[ctr->cipher].ecb_encrypt(IV, ctr->pad, &ctr->key);
}

#endif


/* ref:         HEAD -> develop */
/* git commit:  e8afa13d5c19d2757ff56537d34802c1dad2c507 */
/* commit time: 2019-04-10 17:05:59 +0200 */