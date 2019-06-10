/****************************************************************************
*  md5.h
*
****************************************************************************/

#ifndef _LIB_MD5_H
#define _LIB_MD5_H

#include "string.h"
#include "stdint.h"
#include "stdio.h"
#include "malloc.h"

/****************************************************************************
* Included Files
****************************************************************************/


/****************************************************************************
* Public Types
****************************************************************************/

struct MD5Context
{
	uint32_t buf[4];
	uint32_t bits[2];
	uint8_t in[64];
};

typedef struct MD5Context MD5_CTX;

/****************************************************************************
* Public Function Prototypes
****************************************************************************/

void MD5Init(struct MD5Context *context);

void MD5Update(struct MD5Context *context, unsigned char const *buf,
	unsigned len);

void MD5Final(unsigned char digest[16], struct MD5Context *context);

void MD5Transform(uint32_t buf[4], uint32_t const in[16]);

void md5_sum(const uint8_t *addr, const size_t len, uint8_t *mac);

char *md5_hash(const uint8_t *addr, const size_t len);



#endif /* _LIB_MD5_H */
