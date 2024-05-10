#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

// % means student can set whatever they want 

// defining constants 
#define BASE_LIN_VEL 0.6
#define BASE_ANG_VEL 0.6
#define STEP_VEL 0.1
#define BASE_DUTY_CYCLE 0.6 // 60% duty cycle %
#define STEP_DUTY_CYCLE 0.1 // 10% change %
#define MAX_DUTY_CYCLE 1.0f
#define MIN_DUTY_CYCLE 0.6f
#define BASE_DIFF 0.2 // base diff between duty cycle of left and right wheel for turning


ros::Publisher duty_cycle_pub;

void calculatePWM(float lin_vel, float ang_vel)
{
    // duty_cycle_command: "left_wheel_duty_cycle,right_wheel_duty_cycle"
    // +ve duty cycle: forward; -ve duty cycle: backward
    std_msgs::String duty_cycle_command; 
    float left_wheel_duty_cycle, right_wheel_duty_cycle;
    float diff_duty_cycle_for_turn; // diff between duty cycle of left and right wheel for turning
    int num_step_lin = std::round((std::abs(lin_vel)-BASE_LIN_VEL)/STEP_VEL);
    int num_step_ang = std::round((std::abs(ang_vel)-BASE_ANG_VEL)/STEP_VEL);
    ROS_INFO("num_step_lin: %d", num_step_lin);

    if (lin_vel==0 && ang_vel==0) // STOP
    {
        left_wheel_duty_cycle = 0;
        right_wheel_duty_cycle = 0;
        ROS_INFO("Command: [ STOP ]");
    }
    else if (lin_vel>0 && ang_vel==0) // FORWARD
    {
        left_wheel_duty_cycle = BASE_DUTY_CYCLE + (num_step_lin*STEP_DUTY_CYCLE);
        left_wheel_duty_cycle = std::max(std::min(left_wheel_duty_cycle, MAX_DUTY_CYCLE), MIN_DUTY_CYCLE);
        right_wheel_duty_cycle = left_wheel_duty_cycle;
        ROS_INFO("Command: [ FORWARD ]");
    }
    else if (lin_vel<0 && ang_vel==0) // BACKWARD
    {
        left_wheel_duty_cycle = BASE_DUTY_CYCLE + (num_step_lin*STEP_DUTY_CYCLE);
        left_wheel_duty_cycle = - std::max(std::min(left_wheel_duty_cycle, MAX_DUTY_CYCLE), MIN_DUTY_CYCLE);
        right_wheel_duty_cycle = left_wheel_duty_cycle;
        ROS_INFO("Command: [ BACKWARD ]");
    }
    else if (lin_vel==0 && ang_vel>0) // LEFT 
    {
        // right wheel turn;  left wheel stop
        left_wheel_duty_cycle = 0;
        diff_duty_cycle_for_turn = BASE_DUTY_CYCLE + (num_step_ang*STEP_DUTY_CYCLE);
        right_wheel_duty_cycle = diff_duty_cycle_for_turn;
        right_wheel_duty_cycle = std::max(std::min(right_wheel_duty_cycle, MAX_DUTY_CYCLE), MIN_DUTY_CYCLE);
        ROS_INFO("Command: [ LEFT ]");
    }
    else if (lin_vel==0 && ang_vel<0) // RIGHT 
    {
        // left wheel turn; right wheel stop
        right_wheel_duty_cycle = 0;
        diff_duty_cycle_for_turn = BASE_DUTY_CYCLE + (num_step_ang*STEP_DUTY_CYCLE);
        left_wheel_duty_cycle = diff_duty_cycle_for_turn;
        left_wheel_duty_cycle = std::max(std::min(left_wheel_duty_cycle, MAX_DUTY_CYCLE), MIN_DUTY_CYCLE);
        ROS_INFO("Command: [ RIGHT ]");
    }
    else if (lin_vel>0 && ang_vel>0) // FORWARD LEFT 
    {
        // right wheel faster than left wheel 
        left_wheel_duty_cycle = BASE_DUTY_CYCLE + (num_step_lin+num_step_ang)*STEP_DUTY_CYCLE;
        left_wheel_duty_cycle = std::max(std::min(left_wheel_duty_cycle, MAX_DUTY_CYCLE), MIN_DUTY_CYCLE);
        diff_duty_cycle_for_turn = std::max(BASE_DIFF + (num_step_ang*STEP_DUTY_CYCLE), 0.0); //prevent diff<0 so won't turn wrong dir
        right_wheel_duty_cycle = left_wheel_duty_cycle + diff_duty_cycle_for_turn;
        if (right_wheel_duty_cycle > MAX_DUTY_CYCLE)
        {
            ROS_INFO("Duty cycle exceeds max value, adjusting...");
            right_wheel_duty_cycle = MAX_DUTY_CYCLE;
            left_wheel_duty_cycle = right_wheel_duty_cycle - diff_duty_cycle_for_turn;
            left_wheel_duty_cycle = std::max(left_wheel_duty_cycle, MIN_DUTY_CYCLE);
        }
        
        //right_wheel_duty_cycle = std::max(std::min(left_wheel_duty_cycle, MAX_DUTY_CYCLE), MIN_DUTY_CYCLE);
        ROS_INFO("Command: [ FORWARD LEFT ]");
    }
    else if (lin_vel>0 && ang_vel<0) // FORWARD RIGHT 
    {
        // left wheel faster than right wheel 
        right_wheel_duty_cycle = BASE_DUTY_CYCLE + (num_step_lin+num_step_ang)*STEP_DUTY_CYCLE;
        right_wheel_duty_cycle = std::max(std::min(right_wheel_duty_cycle, MAX_DUTY_CYCLE), MIN_DUTY_CYCLE);
        diff_duty_cycle_for_turn = std::max(BASE_DIFF + (num_step_ang*STEP_DUTY_CYCLE), 0.0);
        left_wheel_duty_cycle = right_wheel_duty_cycle + diff_duty_cycle_for_turn;
        if (left_wheel_duty_cycle > MAX_DUTY_CYCLE)
        {
            ROS_INFO("Duty cycle exceeds max value, adjusting...");
            left_wheel_duty_cycle = MAX_DUTY_CYCLE;
            right_wheel_duty_cycle = left_wheel_duty_cycle - diff_duty_cycle_for_turn;
            right_wheel_duty_cycle = std::max(right_wheel_duty_cycle, MIN_DUTY_CYCLE);
        }
        //left_wheel_duty_cycle = std::max(std::min(right_wheel_duty_cycle, MAX_DUTY_CYCLE), MIN_DUTY_CYCLE);
        ROS_INFO("Command: [ FORWARD RIGHT ]");
    }
    else if (lin_vel<0 && ang_vel<0) // BACKWARD LEFT 
    {
        // right wheel faster than left wheel 
        left_wheel_duty_cycle = BASE_DUTY_CYCLE + (num_step_lin+num_step_ang)*STEP_DUTY_CYCLE;
        left_wheel_duty_cycle = std::max(std::min(left_wheel_duty_cycle, MAX_DUTY_CYCLE), MIN_DUTY_CYCLE);
        diff_duty_cycle_for_turn = std::max(BASE_DIFF + (num_step_ang*STEP_DUTY_CYCLE), 0.0); //prevent diff<0 so won't turn wrong dir
        right_wheel_duty_cycle = left_wheel_duty_cycle + diff_duty_cycle_for_turn;
        if (right_wheel_duty_cycle > MAX_DUTY_CYCLE)
        {
            ROS_INFO("Duty cycle exceeds max value, adjusting...");
            right_wheel_duty_cycle = MAX_DUTY_CYCLE;
            left_wheel_duty_cycle = right_wheel_duty_cycle - diff_duty_cycle_for_turn;
            left_wheel_duty_cycle = std::max(left_wheel_duty_cycle, MIN_DUTY_CYCLE);
        }
        right_wheel_duty_cycle = - right_wheel_duty_cycle;
        left_wheel_duty_cycle = - left_wheel_duty_cycle;
        ROS_INFO("Command: [ BACKWARD LEFT ]");
    }
    else if (lin_vel<0 && ang_vel>0) // BACKWARD RIGHT 
    {
        // left wheel faster than right wheel 
        right_wheel_duty_cycle = BASE_DUTY_CYCLE + (num_step_lin+num_step_ang)*STEP_DUTY_CYCLE;
        right_wheel_duty_cycle = std::max(std::min(right_wheel_duty_cycle, MAX_DUTY_CYCLE), MIN_DUTY_CYCLE);
        diff_duty_cycle_for_turn = std::max(BASE_DIFF + (num_step_ang*STEP_DUTY_CYCLE), 0.0);
        left_wheel_duty_cycle = right_wheel_duty_cycle + diff_duty_cycle_for_turn;
        if (left_wheel_duty_cycle > MAX_DUTY_CYCLE)
        {
            ROS_INFO("Duty cycle exceeds max value, adjusting...");
            left_wheel_duty_cycle = MAX_DUTY_CYCLE;
            right_wheel_duty_cycle = left_wheel_duty_cycle - diff_duty_cycle_for_turn;
            right_wheel_duty_cycle = std::max(right_wheel_duty_cycle, MIN_DUTY_CYCLE);
        }
        right_wheel_duty_cycle = - right_wheel_duty_cycle;
        left_wheel_duty_cycle = - left_wheel_duty_cycle;
        ROS_INFO("Command: [ BACKWARD RIGHT ]");
    }
    else 
    {
        ROS_INFO("Keyboard commands have errors!");
        return;
    }
 
    // publish desired duty cycle 
    duty_cycle_command.data = std::to_string(left_wheel_duty_cycle) + "," + std::to_string(right_wheel_duty_cycle);
    //std::cout << "duty cycle pub: " << left_wheel_duty_cycle << ", " << right_wheel_duty_cycle << std::endl;
    duty_cycle_pub.publish(duty_cycle_command);
    ROS_INFO("Published duty cycle (L,R): %s", duty_cycle_command.data.c_str());
}

void teleopCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    // only the following 2 fields of the msg are used 
    // the rest are all 0
    float linear_vel = msg->linear.x;
    float angular_vel = msg->angular.z;

    ROS_INFO("Received velocity command: [lin_vel = %f] [ang_vel = %f]", linear_vel, angular_vel);
    ROS_INFO("Received keyboard command!");

    calculatePWM(linear_vel, angular_vel);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "locomotion_controller_node");
    ros::NodeHandle nh;
    ros::Subscriber teleop_sub = nh.subscribe("/cmd_vel", 10, teleopCallback);
    duty_cycle_pub = nh.advertise<std_msgs::String>("/duty_cycle", 10);
    ROS_INFO("Started locomotion_controller_node!");
    ros::spin();

    return 0;
}
