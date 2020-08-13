/**********************************************************
 *	@file dms_middleware.h
 *        data manage system , message middleware handle head file.
 *	
 *	author				data				ver
 *	niuhongfang			2020.07.23			v1.1
 **********************************************************/
#ifndef __dms_MIDDLEWARE__
#define __dms_MIDDLEWARE__

#include "stdint.h"
#include "stdbool.h"
#include <rtthread.h>
#include "dms_define.h"


/* message middleware----------------------------------------------------------------  */
/* function caller */
#define	DMS_DECLARE(name)			dms_declare(#name)
#define dms_advertise(name)		dms_advertiser(#name, sizeof(dms_##name##_s))
#define	dms_publish(p,d)			dms_publisher(p,d)
#define	dms_subscrib(name)		dms_subscriber(#name)
#define	dms_check_update(p)		dms_check_updater(p)
#define dms_copy(p,d)					dms_copyer(p, d)
#define	dms_poll(p)						dms_poller(p)
#define	dms_unsubscrib(p)			dms_unsubscriber(p)

/* macro */
#define	LL_FOREACH(head,item)	\
	for(item=head;item;item=(item)->next)

#define LL_APPEND(head,add,format)                                                                    \
	do {                                                                                           \
	  format* _tmp;                                                                        \
	  (add)->next=NULL;                                                                            \
	  if (head) {                                                                                  \
		_tmp = head;                                                                               \
		while (_tmp->next) { _tmp = _tmp->next; }                                                  \
		_tmp->next=(add);                                                                          \
	  } else {                                                                                     \
		(head)=(add);                                                                              \
	  }                                                                                            \
	} while (0)

#define LL_DELETE(head,del,format)                                                                    \
do {                                                                                           \
  format* _tmp;                                                                        \
  if ((head) == (del)) {                                                                       \
    (head)=(head)->next;                                                                       \
  } else {                                                                                     \
    _tmp = head;                                                                               \
    while (_tmp->next && (_tmp->next != (del))) {                                              \
      _tmp = _tmp->next;                                                                       \
    }                                                                                          \
    if (_tmp->next) {                                                                          \
      _tmp->next = ((del)->next);                                                              \
    }                                                                                          \
  }                                                                                            \
} while (0)
	

/* ------ message format define ----- */
#pragma pack(1)
typedef struct dms_status {
	//TODO................
	rt_sem_t		sem;
	bool		update;
	struct dms_status *next;
}dms_status_s;

typedef struct {
	uint8_t obser_total;
	dms_status_s *msg_status;
}observer_status_s;

typedef struct dms_format {
	char *name;
	void *data;
	uint16_t size;
	observer_status_s observer;
	struct dms_format *next;
}dms_format_s, *pdms_format_s;

typedef struct {
	dms_format_s *message;
	dms_status_s *status;
}dms_handler_s;


/* function declare */
int dms_declare(char *name);
dms_handler_s* dms_advertiser(char *name, uint16_t len);
int dms_publisher(dms_handler_s *m, void *data);
dms_handler_s* dms_subscriber(char *name);
int dms_check_updater(dms_handler_s *m);
int dms_copyer(dms_handler_s *m, void *data);
int dms_poller(dms_handler_s *m);
int dms_unsubscriber(dms_handler_s **m);
void dms_dump(void);

#pragma pack()

#endif
