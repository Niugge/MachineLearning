/**********************************************************
 *	@file dms_middleware.c
 *        data manage system , message middleware handle c file.
 *	
 *	author				data				ver
 *	niuhongfang			2020.07.23			v1.1
 **********************************************************/
#include <stdio.h>
#include "stdlib.h"
#include "math.h"
#include "string.h"
#include "dms_middleware.h"

#ifdef __cplusplus
#define	mem_allocate(f,n)	(f*)(new char[n])
#define	mem_release(f)		delete f
#elif defined(__OS_USE)
#define	mem_allocate(f,n)	(f*)malloc(n)
#define	mem_release(f)		free(f)
#else
#define	mem_allocate(f,n)	(f*)malloc(n)
#define	mem_release(f)	    free(f)
#endif

//TODO......................
static rt_mutex_t mutex = RT_NULL;
static dms_format_s msg_first = { "DMS_HEAD" };

static int dms_lock()
{
	//TODO..........
	static bool inited = false;
	if(!inited){
		mutex = rt_mutex_create("dmutex", RT_IPC_FLAG_FIFO);
		inited = true;
	}
	
	rt_mutex_take(mutex, RT_WAITING_FOREVER);   
	
	return 0;
}

static int dms_unlock()
{
	//TODO..........
	
	rt_mutex_release(mutex);
	
	return 0;
}

int dms_declare(char *name)
{
	dms_format_s *p, *q;

	dms_lock();
	LL_FOREACH(&msg_first, p) {
		if (memcmp(p->name, name, strlen(name)) == 0) {
			return 0;
		}
		q = p;
	}
	//TODO.................
	dms_format_s *t = mem_allocate(dms_format_s, sizeof(dms_format_s));
	if (!t) {
		return -1;
	}

	memset(t, 0, sizeof(dms_format_s));
	t->name = name;
	t->next = NULL;
	q->next = t;
	dms_unlock();

	return 0;
}

dms_handler_s* dms_advertiser(char *name, uint16_t len)
{
	dms_format_s *p;
	dms_lock();
	LL_FOREACH(&msg_first, p) {
		if (memcmp(p->name, name, strlen(name)) == 0) {
			break;
		}
	}

	if (!p) {
		return NULL;
	}

	p->data = mem_allocate(char, len);
	if (!p->data) {
		return NULL;
	}
	p->size = len;
	dms_unlock();

	dms_handler_s *m = mem_allocate(dms_handler_s,sizeof(dms_handler_s));
	if (!m) {
		return NULL;
	}
	m->message = p;

	return m;
}

int dms_publisher(dms_handler_s *m, void *data)
{
	if (!m) {
		return -1;
	}

	dms_format_s *p = m->message;

	if (p->size) {
		dms_lock();
		memcpy(p->data, data, p->size);
		dms_unlock();
	}

	dms_status_s *s;
	LL_FOREACH(p->observer.msg_status, s) {
		dms_lock();
		//TODO...........................
		s->update = true;
		dms_unlock();
		if(!s->sem->value){
			rt_sem_release(s->sem);
		}
	}

	return 0;
}

dms_handler_s* dms_subscriber(char *name)
{
	dms_format_s *p;

	dms_lock();
	LL_FOREACH(&msg_first, p) {
		if (memcmp(p->name, name, strlen(name)) == 0) {
			break;
		}
	}

	if (!p) {
		return NULL;
	}

	//TODO......
	dms_status_s *status = mem_allocate(dms_status_s,sizeof(dms_status_s));
	if (!status) {
		return NULL;
	}
	memset(status, 0, sizeof(dms_status_s));
	
	++p->observer.obser_total;
	
	char sem_name[10];
	memset(sem_name, 0, sizeof(sem_name));
	snprintf(sem_name, sizeof(sem_name), "msg_sem%d", p->observer.obser_total);
	
	rt_sem_t sem = rt_sem_create(sem_name, 0, RT_IPC_FLAG_FIFO);
	if(sem == NULL){
		rt_kprintf("create semaphore failed.\n");
		return NULL;
	}
	status->sem = sem;
	LL_APPEND(p->observer.msg_status, status, dms_status_s);

	dms_handler_s *m = mem_allocate(dms_handler_s, sizeof(dms_handler_s));
	if (!m) {
		return NULL;
	}
	m->message = p;
	m->status = status;
	dms_unlock();

	return m;
}

int dms_check_updater(dms_handler_s *m)
{
	int ret = -1;

	if (!m) {
		return -1;
	}

	dms_status_s *s = m->status;
	
	if (!s) {
		return -1;
	}

	if (s->update) {
		ret = 0;
	}

	return ret;
}

int dms_copyer(dms_handler_s *m, void *data)
{
	int ret = -1;

	if (!m || !data) {
		return -1;
	}

	dms_format_s *p = m->message;
	dms_status_s *s = m->status;

	dms_lock();
	memcpy(data, p->data, p->size);
	//TODO..................
	s->update = false;
	dms_unlock();
	
	return ret;
}

int dms_poller(dms_handler_s *m)
{
	int ret = -1;

	if (!m) {
		return -1;
	}

	dms_status_s *s = m->status;
	
	if(!s){
		return -1;
	}
	
	//TODO.........................
	ret = rt_sem_take(s->sem, RT_WAITING_FOREVER);

	return ret;
}

int dms_unsubscriber(dms_handler_s **m)
{
	if (!m) {
		return -1;
	}
	
	dms_format_s *p = (*m)->message;
	dms_status_s *s = (*m)->status;
	
	dms_lock();
	
	LL_DELETE(p->observer.msg_status, s, dms_status_s);

	mem_release(s);
	mem_release(*m);
	*m = NULL;
	p->observer.obser_total--;
	
	dms_unlock();
	
	return 0;
}

void dms_dump()
{
	dms_format_s *p;

	LL_FOREACH(&msg_first, p) {
		rt_kprintf("Messaage %s:\r\n", p->name);
		rt_kprintf("\t observers total:%d\r\n", p->observer.obser_total);
		
		dms_status_s *s;
		uint16_t ownner = 0;
		LL_FOREACH(p->observer.msg_status, s) {
			rt_kprintf("\t\townner:%d update:%d\r\n", ownner++, s->update);
		}
	}
}
