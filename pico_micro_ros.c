#include <stdio.h>
#include <math.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <micro_ros_utilities/string_utilities.h>

// include msgs
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>

// Control board library
#include <hardware/adc.h>
#include <rc/motor/motor.h>
#include <rc/encoder/encoder.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"

#include "pico_micro_ros.h"

const uint LED_PIN = 25;

float enc2meters = ((2.0 * PI * WHEEL_RADIUS) / (GEAR_RATIO * ENCODER_RES));

rcl_publisher_t publisher;
rcl_publisher_t odom_publisher;
std_msgs__msg__Int32 pico_publish_msg;
std_msgs__msg__Float32MultiArray array_msg;
nav_msgs__msg__Odometry odom_msg;

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist sub_msg;

odom_t current_odom;


quaternion_t Euler2Quaternion(float roll, float pitch, float yaw) {
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    quaternion_t q;
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;
    return q;
}

float clamp_duty(float duty)
{
    if (duty > 1.0)
    {
        return 1.0;
    }
    else if (duty < -1.0)
    {
        return -1.0;
    }
    return duty;
}

void twist_callback(const void *msg_in) {
    gpio_put(LED_PIN, 1);
    int16_t l_cmd, r_cmd;
    float left_sp, right_sp, l_duty, r_duty;
    const geometry_msgs__msg__Twist *twist_msg = (const geometry_msgs__msg__Twist *)msg_in;
    float linear_vel = twist_msg->linear.x;
    float angular_vel = twist_msg->angular.z;

    left_sp = linear_vel - WHEEL_BASE * angular_vel / 2;
    right_sp = linear_vel + WHEEL_BASE * angular_vel / 2;

    if (left_sp > 0.05) {
        l_duty = SLOPE_L * left_sp + INTERCEPT_L;
    }
    else if (left_sp < -0.05) {
        l_duty = SLOPE_L * left_sp - INTERCEPT_L;
    }
    else {
        l_duty = 0;
    }

    if (right_sp > 0.05) {
        r_duty = SLOPE_R * right_sp + INTERCEPT_R;
    }
    else if (right_sp < -0.05) {
        r_duty = SLOPE_R * right_sp - INTERCEPT_R;
    }
    else {
        r_duty = 0;
    }

    // Clamp duty cycle to [-1, 1]
    l_duty = clamp_duty(l_duty);
    r_duty = clamp_duty(r_duty);

    // duty to motor command
    l_cmd = LEFT_MOTOR_POL * (int)(l_duty * 0.95 * pow(2, 15));
    r_cmd = RIGHT_MOTOR_POL * (int)(r_duty * 0.95 * pow(2, 15));

    // set left and right motor command
    rc_motor_set(LEFT_MOTOR_CHANNEL, l_cmd);
    rc_motor_set(RIGHT_MOTOR_CHANNEL, r_cmd);
    // sleep_ms(500);
    // rc_motor_set(LEFT_MOTOR_CHANNEL, 0);
    // rc_motor_set(RIGHT_MOTOR_CHANNEL, 0);

    gpio_put(LED_PIN, 0);


}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    static float current_time = 0;
    static float last_time = 0;
    current_time = rmw_uros_epoch_millis();
    float dt = (current_time - last_time) / 1000.0;
    float left_delta, right_delta, delta_d, delta_x, delta_y, delta_yaw;
    // left delta
    left_delta = LEFT_ENC_POL * enc2meters * rc_encoder_read_delta(1);
    // right delta
    right_delta = RIGHT_ENC_POL * enc2meters * rc_encoder_read_delta(3);

    delta_d = (left_delta + right_delta) / 2;
    delta_yaw = (right_delta - left_delta) / WHEEL_BASE;
    delta_x = delta_d * cos(current_odom.yaw + delta_yaw / 2);
    delta_y = delta_d * sin(current_odom.yaw + delta_yaw / 2);
    current_odom.x += delta_x;
    current_odom.y += delta_y;
    current_odom.yaw += delta_yaw;

    array_msg.data.data[0] = left_delta;
    array_msg.data.data[1] = right_delta;
    array_msg.data.data[2] = LEFT_ENC_POL * enc2meters * rc_encoder_read_count(1);
    array_msg.data.data[3] = RIGHT_ENC_POL * enc2meters * rc_encoder_read_count(3);
    array_msg.data.data[4] = current_odom.x;
    array_msg.data.data[5] = current_odom.y;
    array_msg.data.data[6] = current_odom.yaw;
    rcl_ret_t ret = rcl_publish(&publisher, &array_msg, NULL);

    int64_t stamp = rmw_uros_epoch_millis();
    odom_msg.header.stamp.sec = (int32_t)(stamp / 1000);
    odom_msg.header.stamp.nanosec = (uint32_t)((stamp % 1000) * 1e6);
    odom_msg.pose.pose.position.x = current_odom.x;
    odom_msg.pose.pose.position.y = current_odom.y;
    quaternion_t q = Euler2Quaternion(0.0, 0.0, current_odom.yaw);
    odom_msg.pose.pose.orientation.w = q.w;
    odom_msg.pose.pose.orientation.x = q.x;
    odom_msg.pose.pose.orientation.y = q.y;
    odom_msg.pose.pose.orientation.z = q.z;
    odom_msg.twist.twist.angular.z = delta_yaw / dt;
    odom_msg.twist.twist.linear.x = delta_d / dt;
    rcl_publish(&odom_publisher, &odom_msg, NULL);
    last_time = current_time;
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
    rc_encoder_init();
    adc_init();
    adc_gpio_init(26);
    adc_gpio_init(27);
    adc_gpio_init(28);
    rc_motor_init();

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

    rclc_executor_init(&executor, &support.context, 3, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &twist_callback, ON_NEW_DATA);
    
    // init publisher
    rclc_publisher_init_default(
			&publisher,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
			"/pico_publisher");

    rclc_publisher_init_default(
			&odom_publisher,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
			"/odom");

    // init timer
    rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(1000),
		timer_callback);

    rclc_executor_add_timer(&executor, &timer);
    

    // spin node some

    pico_publish_msg.data = 0;

    array_msg.data.capacity = 8;
    array_msg.data.size = 7;
    array_msg.data.data = (float *)malloc(sizeof(float) * array_msg.data.capacity);

    micro_ros_string_utilities_set(odom_msg.header.frame_id, "odom");
    micro_ros_string_utilities_set(odom_msg.child_frame_id, "base_link");

    current_odom.x = 0;
    current_odom.y = 0;
    current_odom.yaw = 0;
    while (true)
    {
        // TODO should we put the update of odometry here?
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }

    return 0;
}
