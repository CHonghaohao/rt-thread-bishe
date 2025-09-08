#include <rtthread.h>
#include <rtdevice.h>
#include <micro_ros_rtt.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/header.h>
#include <std_msgs/msg/string.h>

#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include "ros_test.h"
#include <micro_ros_utilities/type_utilities.h>

#define STRING_BUFFER_LEN 100

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){rt_kprintf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){rt_kprintf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

static rcl_publisher_t ping_publisher;
static rcl_publisher_t pong_publisher;
static rcl_subscription_t ping_subscriber;
static rcl_subscription_t pong_subscriber;

// static std_msgs__msg__Header incoming_ping;
// static std_msgs__msg__Header outcoming_ping;
// static std_msgs__msg__Header incoming_pong;




static std_msgs__msg__String incoming_ping;
static std_msgs__msg__String outcoming_ping;
static std_msgs__msg__String incoming_pong;


static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rcl_timer_t timer;
static rclc_executor_t executor;

int device_id;
int seq_no;
int pong_count;



void ping_timer_callback(rcl_timer_t * atimer, int64_t last_call_time)
{
	(void) last_call_time;

	// atomic_uint_least64_t period;
	// // This is a time in nanoseconds since an unspecified time.
	// atomic_int_least64_t last_call_time;
	// // This is a time in nanoseconds since an unspecified time.
	// atomic_int_least64_t next_call_time;
	// // Credit for time elapsed before ROS time is activated or deactivated.
	// atomic_int_least64_t time_credit;
	// // A flag which indicates if the timer is canceled.
	// atomic_bool canceled;
    
	// rt_kprintf("timer callback ! : %d, %d, %d, %d\n",atimer->impl->period,atimer->impl->last_call_time,atimer->impl->next_call_time,atimer->impl->time_credit);  


	if (atimer != NULL) {
		seq_no = rand();
		rt_sprintf(outcoming_ping.data.data, "%d_%d", seq_no, device_id);
		outcoming_ping.data.size = strlen(outcoming_ping.data.data);
		
		// Fill the message timestamp
		//struct timespec ts;
		//clock_gettime(CLOCK_REALTIME, &ts);
		//outcoming_ping.stamp.sec = ts.tv_sec;
		//outcoming_ping.stamp.nanosec = ts.tv_nsec;

		// Reset the pong count and publish the ping message
		//pong_count = 0;
		RCSOFTCHECK(rcl_publish(&ping_publisher, (const void*)&outcoming_ping, NULL));
		rt_kprintf("Ping send seq %s\n", outcoming_ping.data.data);  
	}
}

void ping_subscription_callback(const void * msgin)
{
	const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;

	// Dont pong my own pings
	//if(strcmp(outcoming_ping.data.data, msg->data.data) != 0){
		rt_kprintf("Ping received with seq %s. Answering.\n", msg->data.data);
		RCSOFTCHECK(rcl_publish(&pong_publisher, (const void*)msg, NULL));
	//}
}


void pong_subscription_callback(const void * msgin)
{
	const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;

	//if(strcmp(outcoming_ping.data.data, msg->data.data) == 0) {
			pong_count++;
			rt_kprintf("Pong for seq %s (%d)\n", msg->data.data, pong_count);
	//}
}

static void microros_ping_pong_thread_entry(void *parameter)
{
    while(1)
    {
        rt_thread_mdelay(100);
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
}


void *  my_malloc(size_t size, void * state)
{
	*state;
	void *buff =  malloc(size);
	return buff;
}


void * my_free(void * pointer, void * state)
{
	*state;
	free(pointer);

	return NULL;
}

void * my_realloc(void * pointer, size_t size, void * state)
{
	*state;
	void *buff = realloc(pointer,size);

	return buff;
}



void *  my_zero_calloc(size_t number_of_elements, size_t size_of_element, void * state)
{   
	*state;
 	void * buf =  calloc(number_of_elements,size_of_element);
	return buf;
}


void microros_ping_pong(void)
{
    // Serial setup
	    
        rt_kprintf("init serial \n");

        set_microros_transports();

        rt_kprintf("transport init \n");
        allocator = rcl_get_default_allocator();

		allocator.allocate = my_malloc;
		allocator.deallocate = my_free;
		allocator.reallocate = my_realloc;
		allocator.zero_allocate = my_zero_calloc;
		allocator.state = NULL;
       
        rt_kprintf("get default allocator init \n");
	// create init_options

	  
		// create init_options
		RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
		rt_kprintf("rclc support init \n");
		// create node
		RCCHECK(rclc_node_init_default(&node, "pingpong_node", "", &support));
		rt_kprintf("rclc node init\n");
		// Create a reliable ping publisher
		RCCHECK(rclc_publisher_init_default(&ping_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/microROS/ping"));

		// Create a best effort pong publisher
		//RCCHECK(rclc_publisher_init_best_effort(&pong_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/microROS/pong"));

		// // Create a best effort ping subscriber
		//RCCHECK(rclc_subscription_init_best_effort(&ping_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/microROS/ping"));

		// Create a best effort  pong subscriber
		RCCHECK(rclc_subscription_init_best_effort(&pong_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/microROS/pong"));


		// Create a 2 seconds ping timer timer,
		RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(2000), ping_timer_callback));


		// Create executor
		executor = rclc_executor_get_zero_initialized_executor();


		RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
		RCCHECK(rclc_executor_add_timer(&executor, &timer));

		// RCCHECK(rclc_executor_add_subscription(&executor, &ping_subscriber, &incoming_ping, &ping_subscription_callback, ON_NEW_DATA));
		 RCCHECK(rclc_executor_add_subscription(&executor, &pong_subscriber, &incoming_pong, &pong_subscription_callback, ON_NEW_DATA));

		// Create and allocate the pingpong messages
		char outcoming_ping_buffer[STRING_BUFFER_LEN];
		outcoming_ping.data.data = outcoming_ping_buffer;
		outcoming_ping.data.capacity = STRING_BUFFER_LEN;

		char incoming_ping_buffer[STRING_BUFFER_LEN];
		incoming_ping.data.data = incoming_ping_buffer;
		incoming_ping.data.capacity = STRING_BUFFER_LEN;

		char incoming_pong_buffer[STRING_BUFFER_LEN];
		incoming_pong.data.data = incoming_pong_buffer;
		incoming_pong.data.capacity = STRING_BUFFER_LEN;

		device_id = rand();

		rt_kprintf("[micro_ros] micro_ros init successful.\n");
		rt_thread_t thread = rt_thread_create("ping_pong", microros_ping_pong_thread_entry, RT_NULL, 8192, 1, 10);
		if(thread != RT_NULL)
		{
			rt_thread_startup(thread);
			rt_kprintf("[micro_ros] New thread mr_pubint32\n");
		}
		else
		{
			rt_kprintf("[micro_ros] Failed to create thread mr_pubint32\n");
		}
}
MSH_CMD_EXPORT(microros_ping_pong, microros_ping_pong);
