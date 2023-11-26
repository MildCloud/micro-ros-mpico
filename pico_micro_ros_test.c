#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <rmw_microros/rmw_microros.h>
#include <geometry_msgs/msg/twist.h>

// Control board library
#include <hardware/adc.h>
#include <rc/motor/motor.h>
#include <rc/encoder/encoder.h>


#include "pico/stdlib.h"
#include "pico_uart_transports.h"

const uint LED_PIN = 25;

rcl_publisher_t publisher;
std_msgs__msg__Int32 pico_publish_msg;
std_msgs__msg__Int32MultiArray int_array_msg;

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist sub_msg;

void twist_callback(const void *msg_in) {
    const geometry_msgs__msg__Twist *twist_msg = (const geometry_msgs__msg__Twist *)msg_in;
    float linear_x = twist_msg->linear.x;
    float angular_z = twist_msg->angular.z;
    printf("recv spped(%f,%f)\n", linear_x, angular_z);
    adc_init();
    adc_gpio_init(26);
    adc_gpio_init(27);
    adc_gpio_init(28);
    rc_motor_init();
    if (linear_x > 0 && angular_z == 0) {
        gpio_put(LED_PIN, 1);
        rc_motor_set(3, 10000);
        rc_motor_set(1, -10000);
        sleep_ms(400);
        rc_motor_set(3, 0);
        rc_motor_set(1, 0);
        gpio_put(LED_PIN, 0);
    }
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    int_array_msg.data.data[0] = rc_encoder_read_delta(1);
    int_array_msg.data.data[1] = rc_encoder_read_delta(3);
    int_array_msg.data.data[2] = rc_encoder_read_count(1);
    int_array_msg.data.data[3] = rc_encoder_read_count(3);
    rcl_ret_t ret = rcl_publish(&publisher, &int_array_msg, NULL);
    // pico_publish_msg.data++;
}

int main()
{
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    rcl_timer_t timer;
    rcl_node_t node;
    rcl_node_t motor_node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "pico_motor_node", "", &support);

    // init subscription
    rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel");

    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &twist_callback, ON_NEW_DATA);
    
    // init publisher
    rclc_publisher_init_default(
			&publisher,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
			"pico_publisher");

    // init timer
    rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(1000),
		timer_callback);

    rclc_executor_add_timer(&executor, &timer);
    

    // spin node some
    rc_encoder_init();

    pico_publish_msg.data = 0;

    int_array_msg.data.capacity = 5;
    int_array_msg.data.size = 4;
    int_array_msg.data.data = (int32_t *)malloc(sizeof(int32_t) * int_array_msg.data.capacity);

    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }

    return 0;
}
