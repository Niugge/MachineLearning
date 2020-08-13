/**********************************************************
 *	@file dms_define.h
 *        data manage system , define middleware message format.
 *	
 *	author				data				ver
 *	niuhongfang			2020.07.23			v1.1
 **********************************************************/

#ifndef __MESSAGE_DEFINE__
#define __MESSAGE_DEFINE__

#include "stdint.h"
#include "stdbool.h"


/* ------ message define : message_XXX_s ------- */
#define	dms_define(name)	dms_##name##_s

#pragma pack(1)
typedef struct {
	int a;
	int b;
}dms_define(qwerty);

typedef struct {
	int c;
	int d;
}dms_define(asdfgh);


#pragma pack()

#endif

