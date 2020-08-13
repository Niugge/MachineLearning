/* 
 * Copyright (c) 2006-2018, RT-Thread Development Team 
 * 
 * SPDX-License-Identifier: Apache-2.0 
 * 
 * Change Logs: 
 * Date           Author       Notes 
 * 2018-08-24     yangjie      the first version 
 */ 

/*
 * 程序清单：信号量例程
 *
 * 该例程创建了一个动态信号量，初始化两个线程，线程1在count每计数10次时，
 * 发送一个信号量，线程2在接收信号量后，对number进行加1操作
 */
#include <rtthread.h>
#include "dms_middleware.h"

#define THREAD_PRIORITY         25
#define THREAD_TIMESLICE        5

/* 指向信号量的指针 */
static rt_sem_t dynamic_sem = RT_NULL;

ALIGN(RT_ALIGN_SIZE)
static char thread1_stack[1024];
static struct rt_thread thread1;
static void rt_thread1_entry(void *parameter)
{
    static rt_uint8_t count = 0;
  
		dms_handler_s *pub = dms_advertise(qwerty);
		dms_handler_s *pub2 = dms_advertise(asdfgh);
    while(1)
    {
        if(count <= 100)
        {
            count++;           
        }
        else
            return; 
        
        /* count每计数10次，就释放一次信号量 */
         if(0 == (count % 10))
        {
						static dms_qwerty_s data = {1,2};
						static dms_asdfgh_s data2 = {5,6};
						data.a++;
						data.b++;
						dms_publish(pub,&data);
						rt_thread_mdelay(100);
						dms_publish(pub2,&data2);
            rt_kprintf("t1 release a dynamic semaphore.\n" ); 
            //rt_sem_release(dynamic_sem);            
						rt_thread_mdelay(100);
        }
    }
}

ALIGN(RT_ALIGN_SIZE)
static char thread2_stack[1024];
static struct rt_thread thread2;
static void rt_thread2_entry(void *parameter)
{
    static rt_err_t result;
    static rt_uint8_t number = 0;
	
		dms_handler_s *m = dms_subscrib(qwerty);
	dms_qwerty_s data2;
	
			dms_handler_s *m2 = dms_subscrib(asdfgh);
	dms_asdfgh_s data3;
	
    while(1)
    {
        /* 永久方式等待信号量，获取到信号量，则执行number自加的操作 */
        //result = rt_sem_take(dynamic_sem, RT_WAITING_FOREVER);
			result = dms_poll(m);
        if (result != RT_EOK)
        {        
						//dms_QWERTY_s data2;
						
					
					rt_kprintf("t2 take a dynamic semaphore, failed. a:%d b:%d\n", data2.a, data2.b);
            //rt_sem_delete(dynamic_sem);
            return;
        }
        else
        {      
					dms_copy(m, &data2);
            number++;   
					if(number>=5){
						dms_dump();
						dms_unsubscrib(&m); 
					}
         
					rt_kprintf("t2 take a dynamic semaphore. number = %d a:%d b:%d\r\n" ,number, data2.a, data2.b);                        
        }
				
				if(dms_check_update(m2)==0){
						dms_copy(m2, &data3);
					rt_kprintf("t2 take a  c:%d d:%d\r\n" , data3.c, data3.d);
				}
				
    }   
}

ALIGN(RT_ALIGN_SIZE)
static char thread3_stack[1024];
static struct rt_thread thread3;
static void rt_thread3_entry(void *parameter)
{
    static rt_err_t result;
    static rt_uint8_t number = 0;
	
		dms_handler_s *m = dms_subscrib(qwerty);
	dms_qwerty_s data2;
    while(1)
    {
        /* 永久方式等待信号量，获取到信号量，则执行number自加的操作 */
        //result = rt_sem_take(dynamic_sem, RT_WAITING_FOREVER);
			result = dms_poll(m);
        if (result != RT_EOK)
        {        
						//dms_QWERTY_s data2;
						
					
					rt_kprintf("t3 take a dynamic semaphore, failed. a:%d b:%d\n", data2.a, data2.b);
            //rt_sem_delete(dynamic_sem);
            return;
        }
        else
        {      
					dms_copy(m, &data2);
            number++;             
					rt_kprintf("t3 take a dynamic semaphore. number = %d a:%d b:%d\r\n" ,number, data2.a, data2.b);                        
        }
    }   
}

ALIGN(RT_ALIGN_SIZE)
static char thread4_stack[1024];
static struct rt_thread thread4;
static void rt_thread4_entry(void *parameter)
{
    static rt_err_t result;
    static rt_uint8_t number = 0;
	
		dms_handler_s *m = dms_subscrib(asdfgh);
	dms_asdfgh_s data2;
    while(1)
    {
        /* 永久方式等待信号量，获取到信号量，则执行number自加的操作 */
        //result = rt_sem_take(dynamic_sem, RT_WAITING_FOREVER);
			result = dms_poll(m);
        if (result != RT_EOK)
        {        
						//dms_QWERTY_s data2;
						
					
					rt_kprintf("t4 take a dynamic semaphore, failed. a:%d b:%d\n", data2.c, data2.d);
            //rt_sem_delete(dynamic_sem);
            return;
        }
        else
        {      
					dms_copy(m, &data2);
            number++;             
					rt_kprintf("t4 take a dynamic semaphore. number = %d a:%d b:%d\r\n" ,number, data2.c, data2.d);                        
        }
    }   
}

/* 信号量示例的初始化 */
int semaphore_sample()
{
	
		DMS_DECLARE(qwerty);
		DMS_DECLARE(asdfgh);
		
	
	
    /* 创建一个动态信号量，初始值是0 */
    dynamic_sem = rt_sem_create("dsem", 0, RT_IPC_FLAG_FIFO);
    if (dynamic_sem == RT_NULL)
    {
        rt_kprintf("create dynamic semaphore failed.\n");
        return -1;
    }
    else
    {
        rt_kprintf("create done. dynamic semaphore value = 0.\n");
    }

    rt_thread_init(&thread1,
                   "thread1",
                   rt_thread1_entry,
                   RT_NULL,
                   &thread1_stack[0],
                   sizeof(thread1_stack), 
                   THREAD_PRIORITY, THREAD_TIMESLICE);
    rt_thread_startup(&thread1);
                   
    rt_thread_init(&thread2,
                   "thread2",
                   rt_thread2_entry,
                   RT_NULL,
                   &thread2_stack[0],
                   sizeof(thread2_stack), 
                   THREAD_PRIORITY-1, THREAD_TIMESLICE);
    rt_thread_startup(&thread2);
								 
		rt_thread_init(&thread3,
							 "thread3",
							 rt_thread3_entry,
							 RT_NULL,
							 &thread3_stack[0],
							 sizeof(thread3_stack), 
							 THREAD_PRIORITY-1, THREAD_TIMESLICE);
	rt_thread_startup(&thread3);

				rt_thread_init(&thread4,
							 "thread4",
							 rt_thread4_entry,
							 RT_NULL,
							 &thread4_stack[0],
							 sizeof(thread4_stack), 
							 THREAD_PRIORITY-1, THREAD_TIMESLICE);
	rt_thread_startup(&thread4);					 
							 
    return 0;
}
/* 导出到 msh 命令列表中 */
MSH_CMD_EXPORT(semaphore_sample, semaphore sample);

