// // #include <rtthread.h>
// // #include <rtdevice.h>
// // #include <micro_ros_rtt.h>
// // #include <rcl/rcl.h>
// // #include <rcl/error_handling.h>
// // #include <rclc/rclc.h>
// // #include <rclc/executor.h>
// // #include <std_msgs/msg/header.h>
// // #include <std_msgs/msg/string.h>

// // #include <stdio.h>
// // #include <unistd.h>
// // #include <time.h>
// // #include "ros_test.h"
// // #include <micro_ros_utilities/type_utilities.h>

// // #ifdef RT_USING_CAN
// // #define CAN_DEV_NAME       "rk_can2"
// // static rt_device_t can_dev;

// // #define STRING_BUFFER_LEN 100

// // #define RCCHECK(fn) { \
// // 	rcl_ret_t temp_rc = fn; \
// // 	if((temp_rc != RCL_RET_OK)){ \
// // 		rt_kprintf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);  \
// //     } \
// // }
// // // 		// return;\
// // // 	}\
// // // }
// // #define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){rt_kprintf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

// // static rcl_publisher_t ping_publisher;
// // static rcl_publisher_t pong_publisher;
// // static rcl_subscription_t ping_subscriber;
// // static rcl_subscription_t pong_subscriber;

// // static std_msgs__msg__String incoming_ping;
// // static std_msgs__msg__String outcoming_ping;
// // static std_msgs__msg__String incoming_pong;

// // static rcl_allocator_t allocator;
// // static rclc_support_t support;
// // static rcl_node_t node;
// // static rcl_timer_t timer;
// // static rclc_executor_t executor;

// // int seq_no;
// // int pong_count;
// // char outcoming_ping_buffer[STRING_BUFFER_LEN];
// // char incoming_ping_buffer[STRING_BUFFER_LEN];
// // char incoming_pong_buffer[STRING_BUFFER_LEN];

// // void ping_timer_callback(rcl_timer_t * atimer, int64_t last_call_time)
// // {
// // }

// // void pong_subscription_callback(const void * msgin)
// // {
// // 	const std_msgs__msg__String * ros_msg = (const std_msgs__msg__String *)msgin;
// //     struct rt_can_msg msg = {0};
// //     rt_size_t  size;
// // 	//if(strcmp(outcoming_ping.data.data, msg->data.data) == 0) {
// //     pong_count++;
// //     rt_kprintf("Pong for seq %s (%d)\n", ros_msg->data.data, pong_count);
// //     msg.id = 0x123;
// //     msg.ide = RT_CAN_STDID;
// //     msg.rtr = RT_CAN_DTR;
// //     msg.len = 8;
// //     msg.data[0] = 0x12;
// //     msg.data[1] = 0x34;
// //     msg.data[2] = 0x56;
// //     msg.data[3] = 0x78;
// //     msg.data[4] = 0x87;
// //     msg.data[5] = 0x65;
// //     msg.data[6] = 0x43;
// //     msg.data[7] = 0x21;
// //     size = rt_device_write(can_dev, 0, &msg, sizeof(msg));
// //     if (size == 0)
// //     {
// //         rt_kprintf("can dev write data failed!\n");
// //     }
// // }

// // //定时器
// // static void microros_ping_pong_thread_entry(void *parameter)
// // {
// //     while(1)
// //     {
// //         rt_thread_mdelay(100);
// //         rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
// //     }
// // }


// // void *  my_malloc(size_t size, void * state)
// // {
// // 	*state;
// // 	void *buff =  malloc(size);
// // 	return buff;
// // }


// // void * my_free(void * pointer, void * state)
// // {
// // 	*state;
// // 	free(pointer);

// // 	return NULL;
// // }

// // void * my_realloc(void * pointer, size_t size, void * state)
// // {
// // 	*state;
// // 	void *buff = realloc(pointer,size);

// // 	return buff;
// // }



// // void *  my_zero_calloc(size_t number_of_elements, size_t size_of_element, void * state)
// // {   
// // 	*state;
// //  	void * buf =  calloc(number_of_elements,size_of_element);
// // 	return buf;
// // }

// // void  microros_test_entry(void *args)
// // {
// //     unsigned int count = 0;
// //     while(1)
// //     {
// //         rt_kprintf("sed ping data,gap 100ms %d !\n",count++);
// //         seq_no = rand();
// //         rt_sprintf(outcoming_ping.data.data, "%d_123888", seq_no);
// //         outcoming_ping.data.size = strlen(outcoming_ping.data.data);
// //         RCSOFTCHECK(rcl_publish(&ping_publisher, (const void*)&outcoming_ping, NULL));
// //         rt_kprintf("Ping send seq %s\n", outcoming_ping.data.data);

// //         rt_thread_mdelay(100);
// //     }
// // }

// // void microros_init(void)
// // {
// //     // Serial setup
	    
// //     rt_kprintf("init serial \n");

// //     set_microros_transports();

// //     rt_kprintf("transport init \n");
// //     allocator = rcl_get_default_allocator();

// //     allocator.allocate = my_malloc;
// //     allocator.deallocate = my_free;
// //     allocator.reallocate = my_realloc;
// //     allocator.zero_allocate = my_zero_calloc;
// //     allocator.state = NULL;
    
// //     rt_kprintf("get default allocator init \n");
// //     // create init_options

// //     RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

// //     // create node
// //     RCCHECK(rclc_node_init_default(&node, "pingpong_node", "", &support));

// //     // Create a reliable ping publisher
// //     RCCHECK(rclc_publisher_init_default(&ping_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/can_feedback_frame"));

// //     // Create a best effort  pong subscriber
// //     RCCHECK(rclc_subscription_init_best_effort(&pong_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/can_control_frame"));///microROS/pong

// //     // Create a 2 seconds ping timer timer,
// //     RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(2000), ping_timer_callback));

// //     // Create executor
// //     executor = rclc_executor_get_zero_initialized_executor();

// //     RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
// //     RCCHECK(rclc_executor_add_timer(&executor, &timer));

// //     // RCCHECK(rclc_executor_add_subscription(&executor, &ping_subscriber, &incoming_ping, &ping_subscription_callback, ON_NEW_DATA));
// //     RCCHECK(rclc_executor_add_subscription(&executor, &pong_subscriber, &incoming_pong, &pong_subscription_callback, ON_NEW_DATA));
    

// //     // Create and allocate the pingpong messages
// //     outcoming_ping.data.data = outcoming_ping_buffer;
// //     outcoming_ping.data.capacity = STRING_BUFFER_LEN;

// //     incoming_ping.data.data = incoming_ping_buffer;
// //     incoming_ping.data.capacity = STRING_BUFFER_LEN;

// //     incoming_pong.data.data = incoming_pong_buffer;
// //     incoming_pong.data.capacity = STRING_BUFFER_LEN;

// //     rt_kprintf("[micro_ros] micro_ros init successful.\n");
// //     rt_thread_t thread = rt_thread_create("ping_pong", microros_ping_pong_thread_entry, RT_NULL, 8192, 10, 10);
// //     if(thread != RT_NULL)
// //     {
// //         rt_thread_startup(thread);
// //         rt_kprintf("[micro_ros] New thread mr_pubint32\n");
// //     }
// //     else
// //     {
// //         rt_kprintf("[micro_ros] Failed to create thread mr_pubint32\n");
// //     }
// //     rt_thread_t  microros_entry =  rt_thread_create("micoros entry", //线程名字
// //         microros_test_entry, //入口函数
// //         RT_NULL, //入口函数参数
// //         4096, //栈大小
// //         1, //线程优先级
// //         5000); //线程时间片大小

// //     if(microros_entry == NULL)
// //     {
// //         rt_kprintf("microros entry error !\n");
// //         return;
// //     }
// //     rt_thread_startup(microros_entry);
// // }
// // MSH_CMD_EXPORT(microros_init, can device sample);

// // static struct rt_semaphore rx_sem;

// // static rt_err_t can_rx_call(rt_device_t dev, rt_size_t size)
// // {
// //     rt_sem_release(&rx_sem);
// //     return RT_EOK;
// // }

// // static void can_rx_thread(void *parameter)
// // {
// //     struct rt_can_msg rxmsg = {0};

// //     rt_device_set_rx_indicate(can_dev, can_rx_call);

// // #ifdef RT_CAN_USING_HDR
// //     struct rt_can_filter_item items[4] =
// //     {
// //         RT_CAN_FILTER_ITEM_INIT(0x100, 0, 0, 1, 0x700, RT_NULL, RT_NULL),
// //         RT_CAN_FILTER_ITEM_INIT(0x300, 0, 0, 1, 0x700, RT_NULL, RT_NULL),
// //         RT_CAN_FILTER_ITEM_INIT(0x211, 0, 0, 1, 0x7ff, RT_NULL, RT_NULL),
// //         RT_CAN_FILTER_STD_INIT(0x486, RT_NULL, RT_NULL),
// //     };
// //     struct rt_can_filter_config cfg = {4, 1, items};
// //     rt_device_control(can_dev, RT_CAN_CMD_SET_FILTER, &cfg);
// // #endif
// //     while (1)
// //     {
// //         rxmsg.hdr = -1;
// //         rt_sem_take(&rx_sem, RT_WAITING_FOREVER);
// //         rt_device_read(can_dev, 0, &rxmsg, sizeof(rxmsg));
// //         rt_kprintf("ID:%x , LEN:%x DATA:\n", rxmsg.id, rxmsg.len);
// //         for (int i = 0; i < 8; i++)
// //         {
// //             rt_kprintf("%2x", rxmsg.data[i]);
// //         }
// //         rt_kprintf("\n");
// //         char oristr[] = "1000000000000000000000000000000000000000000000000000000000000";
// //         outcoming_ping.data.data = oristr;
// //         // rt_kprintf("ori: %s\n", outcoming_ping.data.data);
// //         char tmpstr[] = "1-0-0";
// //         if(rxmsg.rtr != 0) tmpstr[2] = '1';
// //         if(rxmsg.ide != 0) tmpstr[4] = '1';
// //         char id[20];
// //         rt_sprintf(id, "-%d-%d", rxmsg.id, rxmsg.len);

// //         int pos = 0;
// //         int i = 0;
// //         // rt_kprintf("tmp: %s\n", tmpstr);
// //         while(tmpstr[i] != '\0')
// //         {
// //             outcoming_ping.data.data[pos] = tmpstr[i];
// //             ++i;
// //             ++pos;
// //         }
// //         // rt_kprintf("tmpstr: %s\n", outcoming_ping.data.data);
// //         i = 0;
// //         while(id[i] != '\0')
// //         {
// //             outcoming_ping.data.data[pos] = id[i];
// //             ++i;
// //             ++pos;
// //         }
// //         // rt_kprintf("id: %s\n", outcoming_ping.data.data);
// //         for (int j = 0; j < rxmsg.len; ++j) 
// //         {
// //             rt_sprintf(tmpstr, "-%d", rxmsg.data[j]);
// //             i = 0;
// //             while(tmpstr[i] != '\0')
// //             {
// //                 outcoming_ping.data.data[pos] = tmpstr[i];
// //                 ++i;
// //                 ++pos;
// //             }
// //         }
// //         outcoming_ping.data.data[pos] = '\0';

// // 		outcoming_ping.data.size = strlen(outcoming_ping.data.data);
// //         rt_kprintf("Ping send seq %s\n", outcoming_ping.data.data); 
// //         // rt_kprintf("Ping send seq ori %s\n", oristr); 
// // 		RCSOFTCHECK(rcl_publish(&ping_publisher, (const void*)&outcoming_ping, NULL));
// //     }
// // }



// // int can_init(int argc, char *argv[])
// // {
// //     rt_err_t res = 0;
// //     rt_thread_t thread;
// //     char can_name[RT_NAME_MAX];

// //     if (argc == 2)
// //     {
// //         rt_strncpy(can_name, argv[1], RT_NAME_MAX);
// //     }
// //     else
// //     {
// //         rt_strncpy(can_name, CAN_DEV_NAME, RT_NAME_MAX);
// //     }
// //     can_dev = rt_device_find(can_name);
// //     if (!can_dev)
// //     {
// //         rt_kprintf("find %s failed!\n", can_name);
// //         return -RT_ERROR;
// //     }

// //     rt_sem_init(&rx_sem, "rx_sem", 0, RT_IPC_FLAG_FIFO);
// //     res = rt_device_open(can_dev, RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);
// //     RT_ASSERT(res == RT_EOK);
// //     rt_device_control(can_dev, RT_CAN_CMD_SET_MODE, (void *)RT_CAN_MODE_NORMAL);
// //     thread = rt_thread_create("can_rx", can_rx_thread, RT_NULL, 1024*8, 25, 10);
// //     if (thread != RT_NULL)
// //     {
// //         rt_thread_startup(thread);
// //     }
// //     else
// //     {
// //         rt_kprintf("create can_rx thread failed!\n");
// //     }
// //     return res;
// // }
// // MSH_CMD_EXPORT(can_init, can device sample);
// // #endif


// ////LIGHT 
// #include <rtthread.h>
// #include <rtdevice.h>
// #include <micro_ros_rtt.h>

// #include <rcl/rcl.h>
// #include <rcl/error_handling.h>
// #include <rclc/rclc.h>
// #include <rclc/executor.h>

// #include <std_msgs/msg/header.h>
// #include <std_msgs/msg/string.h>

// #include <stdio.h>
// #include <unistd.h>
// #include <time.h>
// #include "ros_test.h"
// #include <micro_ros_utilities/type_utilities.h>

// #define STRING_BUFFER_LEN 100

// #define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){rt_kprintf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return;}}
// #define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){rt_kprintf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

// static rcl_publisher_t ping_publisher;
// static rcl_publisher_t pong_publisher;
// static rcl_subscription_t ping_subscriber;
// static rcl_subscription_t pong_subscriber;

// // static std_msgs__msg__Header incoming_ping;
// // static std_msgs__msg__Header outcoming_ping;
// // static std_msgs__msg__Header incoming_pong;

// char outcoming_ping_buffer[STRING_BUFFER_LEN];
// char incoming_ping_buffer[STRING_BUFFER_LEN];
// char incoming_pong_buffer[STRING_BUFFER_LEN];

// static std_msgs__msg__String incoming_ping;
// static std_msgs__msg__String outcoming_ping;
// static std_msgs__msg__String incoming_pong;


// static rcl_allocator_t allocator;
// static rclc_support_t support;
// static rcl_node_t node;
// static rcl_timer_t timer;
// static rclc_executor_t executor;

// int device_id;
// int seq_no;
// int pong_count;

// #ifdef RT_USING_CAN
// #define CAN_DEV_NAME       "rk_can2"
// static rt_device_t can_dev;

// void ping_timer_callback(rcl_timer_t * atimer, int64_t last_call_time)
// {
// 	(void) last_call_time;

// 	// atomic_uint_least64_t period;
// 	// // This is a time in nanoseconds since an unspecified time.
// 	// atomic_int_least64_t last_call_time;
// 	// // This is a time in nanoseconds since an unspecified time.
// 	// atomic_int_least64_t next_call_time;
// 	// // Credit for time elapsed before ROS time is activated or deactivated.
// 	// atomic_int_least64_t time_credit;
// 	// // A flag which indicates if the timer is canceled.
// 	// atomic_bool canceled;
    
// 	// rt_kprintf("timer callback ! : %d, %d, %d, %d\n",atimer->impl->period,atimer->impl->last_call_time,atimer->impl->next_call_time,atimer->impl->time_credit);  


// 	//if (atimer != NULL) {
// 	//	seq_no = rand();
// 	//	rt_sprintf(outcoming_ping.data.data, "%d_%d", seq_no, device_id);
// 	//	outcoming_ping.data.size = strlen(outcoming_ping.data.data);
		
// 		// Fill the message timestamp
// 		//struct timespec ts;
// 		//clock_gettime(CLOCK_REALTIME, &ts);
// 		//outcoming_ping.stamp.sec = ts.tv_sec;
// 		//outcoming_ping.stamp.nanosec = ts.tv_nsec;

// 		// Reset the pong count and publish the ping message
// 		//pong_count = 0;
// 	//	RCSOFTCHECK(rcl_publish(&ping_publisher, (const void*)&outcoming_ping, NULL));
// 	//	rt_kprintf("Ping send seq %s\n", outcoming_ping.data.data);  
// 	//}
// }

// void ping_subscription_callback(const void * msgin)
// {
// 	const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;

// 	// Dont pong my own pings
// 	//if(strcmp(outcoming_ping.data.data, msg->data.data) != 0){
// 		rt_kprintf("Ping received with seq %s. Answering.\n", msg->data.data);
// 		RCSOFTCHECK(rcl_publish(&pong_publisher, (const void*)msg, NULL));
// 	//}
// }


// void pong_subscription_callback(const void * msgin)
// {
// 	const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;

// 	//if(strcmp(outcoming_ping.data.data, msg->data.data) == 0) {
// 			pong_count++;
// 			rt_kprintf("Pong for seq %s (%d)\n", msg->data.data, pong_count);
// 	//}
// }

// static void microros_ping_pong_thread_entry(void *parameter)
// {
//     while(1)
//     {
//         rt_thread_mdelay(100);
//         rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
//     }
// }


// void *  my_malloc(size_t size, void * state)
// {
// 	*state;
// 	void *buff =  malloc(size);
// 	return buff;
// }


// void * my_free(void * pointer, void * state)
// {
// 	*state;
// 	free(pointer);

// 	return NULL;
// }

// void * my_realloc(void * pointer, size_t size, void * state)
// {
// 	*state;
// 	void *buff = realloc(pointer,size);

// 	return buff;
// }



// void *  my_zero_calloc(size_t number_of_elements, size_t size_of_element, void * state)
// {   
// 	*state;
//  	void * buf =  calloc(number_of_elements,size_of_element);
// 	return buf;
// }


// void microros_init(void)
// {
//     // Serial setup
	    
//     rt_kprintf("init serial \n");

//     set_microros_transports();

//     rt_kprintf("transport init \n");
//     allocator = rcl_get_default_allocator();

//     allocator.allocate = my_malloc;
//     allocator.deallocate = my_free;
//     allocator.reallocate = my_realloc;
//     allocator.zero_allocate = my_zero_calloc;
//     allocator.state = NULL;
    
//     rt_kprintf("get default allocator init \n");
// // create init_options

    
//     // create init_options
//     RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

//     // create node
//     RCCHECK(rclc_node_init_default(&node, "pingpong_node", "", &support));

//     // Create a reliable ping publisher
//     RCCHECK(rclc_publisher_init_default(&ping_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/can_feedback_frame"));

//     // Create a best effort  pong subscriber
//     RCCHECK(rclc_subscription_init_best_effort(&pong_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/can_control_frame"));


//     // Create a 2 seconds ping timer timer,
//     // RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(100), ping_timer_callback));


//     // Create executor
//     executor = rclc_executor_get_zero_initialized_executor();


//     RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
//     RCCHECK(rclc_executor_add_timer(&executor, &timer));

//     // RCCHECK(rclc_executor_add_subscription(&executor, &ping_subscriber, &incoming_ping, &ping_subscription_callback, ON_NEW_DATA));
//         RCCHECK(rclc_executor_add_subscription(&executor, &pong_subscriber, &incoming_pong, &pong_subscription_callback, ON_NEW_DATA));

//     // Create and allocate the pingpong messages
//     outcoming_ping.data.data = outcoming_ping_buffer;
//     outcoming_ping.data.capacity = STRING_BUFFER_LEN;

//     incoming_ping.data.data = incoming_ping_buffer;
//     incoming_ping.data.capacity = STRING_BUFFER_LEN;

//     incoming_pong.data.data = incoming_pong_buffer;
//     incoming_pong.data.capacity = STRING_BUFFER_LEN;

//     device_id = rand();

//     rt_kprintf("[micro_ros] micro_ros init successful.\n");
//     rt_thread_t thread = rt_thread_create("ping_pong", microros_ping_pong_thread_entry, RT_NULL, 8192, 10, 10);
//     if(thread != RT_NULL)
//     {
//         rt_thread_startup(thread);
//         rt_kprintf("[micro_ros] New thread mr_pubint32\n");
//     }
//     else
//     {
//         rt_kprintf("[micro_ros] Failed to create thread mr_pubint32\n");
//     }
// }
// MSH_CMD_EXPORT(microros_init, can device sample);

// static struct rt_semaphore rx_sem;

// static rt_err_t can_rx_call(rt_device_t dev, rt_size_t size)
// {
//     rt_sem_release(&rx_sem);
//     return RT_EOK;
// }

// static void can_rx_thread(void *parameter)
// {
//     struct rt_can_msg rxmsg = {0};

//     rt_device_set_rx_indicate(can_dev, can_rx_call);

// #ifdef RT_CAN_USING_HDR
//     struct rt_can_filter_item items[4] =
//     {
//         RT_CAN_FILTER_ITEM_INIT(0x100, 0, 0, 1, 0x700, RT_NULL, RT_NULL),
//         RT_CAN_FILTER_ITEM_INIT(0x300, 0, 0, 1, 0x700, RT_NULL, RT_NULL),
//         RT_CAN_FILTER_ITEM_INIT(0x211, 0, 0, 1, 0x7ff, RT_NULL, RT_NULL),
//         RT_CAN_FILTER_STD_INIT(0x486, RT_NULL, RT_NULL),
//     };
//     struct rt_can_filter_config cfg = {4, 1, items};
//     rt_device_control(can_dev, RT_CAN_CMD_SET_FILTER, &cfg);
// #endif
//     while (1)
//     {
//         rxmsg.hdr = -1;
//         rt_sem_take(&rx_sem, RT_WAITING_FOREVER);
//         rt_device_read(can_dev, 0, &rxmsg, sizeof(rxmsg));
//         rt_kprintf("ID:%x , LEN:%x DATA:\n", rxmsg.id, rxmsg.len);
//         for (int i = 0; i < 8; i++)
//         {
//             rt_kprintf("%2x", rxmsg.data[i]);
//         }
//         rt_kprintf("\n");
//         char oristr[] = "1000000000000000000000000000000000000000000000000000000000000";
//         outcoming_ping.data.data = oristr;
//         // rt_kprintf("ori: %s\n", outcoming_ping.data.data);
//         char tmpstr[] = "1-0-0";
//         if(rxmsg.rtr != 0) tmpstr[2] = '1';
//         if(rxmsg.ide != 0) tmpstr[4] = '1';
//         char id[20];
//         rt_sprintf(id, "-%d-%d", rxmsg.id, rxmsg.len);

//         int pos = 0;
//         int i = 0;
//         // rt_kprintf("tmp: %s\n", tmpstr);
//         while(tmpstr[i] != '\0')
//         {
//             outcoming_ping.data.data[pos] = tmpstr[i];
//             ++i;
//             ++pos;
//         }
//         // rt_kprintf("tmpstr: %s\n", outcoming_ping.data.data);
//         i = 0;
//         while(id[i] != '\0')
//         {
//             outcoming_ping.data.data[pos] = id[i];
//             ++i;
//             ++pos;
//         }
//         // rt_kprintf("id: %s\n", outcoming_ping.data.data);
//         for (int j = 0; j < rxmsg.len; ++j) 
//         {
//             rt_sprintf(tmpstr, "-%d", rxmsg.data[j]);
//             i = 0;
//             while(tmpstr[i] != '\0')
//             {
//                 outcoming_ping.data.data[pos] = tmpstr[i];
//                 ++i;
//                 ++pos;
//             }
//         }
//         outcoming_ping.data.data[pos] = '\0';

// 		outcoming_ping.data.size = strlen(outcoming_ping.data.data);
//         rt_kprintf("Ping send seq %s\n", outcoming_ping.data.data); 
//         // rt_kprintf("Ping send seq ori %s\n", oristr); 
// 		RCSOFTCHECK(rcl_publish(&ping_publisher, (const void*)&outcoming_ping, NULL));
//     }
// }



// int can_init(int argc, char *argv[])
// {
//     rt_err_t res = 0;
//     rt_thread_t thread;
//     char can_name[RT_NAME_MAX];

//     if (argc == 2)
//     {
//         rt_strncpy(can_name, argv[1], RT_NAME_MAX);
//     }
//     else
//     {
//         rt_strncpy(can_name, CAN_DEV_NAME, RT_NAME_MAX);
//     }
//     can_dev = rt_device_find(can_name);
//     if (!can_dev)
//     {
//         rt_kprintf("find %s failed!\n", can_name);
//         return -RT_ERROR;
//     }

//     rt_sem_init(&rx_sem, "rx_sem", 0, RT_IPC_FLAG_FIFO);
//     res = rt_device_open(can_dev, RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);
//     RT_ASSERT(res == RT_EOK);
//     rt_device_control(can_dev, RT_CAN_CMD_SET_MODE, (void *)RT_CAN_MODE_NORMAL);
//     thread = rt_thread_create("can_rx", can_rx_thread, RT_NULL, 1024*8, 25, 10);
//     if (thread != RT_NULL)
//     {
//         rt_thread_startup(thread);
//     }
//     else
//     {
//         rt_kprintf("create can_rx thread failed!\n");
//     }
//     return res;
// }
// MSH_CMD_EXPORT(can_init, can device sample);
// #endif

