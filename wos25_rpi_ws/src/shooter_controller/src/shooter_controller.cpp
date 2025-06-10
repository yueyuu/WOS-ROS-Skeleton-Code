#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int8.h>

#define PITCH_STEP 10 ///degrees 
#define MAX_PITCH_ANGLE 90 //degrees
 

// global variables 
enum ShooterCommand
{
    shoot = 1, 
    pitch_down = 9,
    pitch_up = 0
};

ShooterCommand shooter_command;
bool new_data;

//====================================================================================================================

std::string shooterCommandToString(ShooterCommand command)
{
    std::string command_name;

    switch(command)
    {
        case ShooterCommand::shoot:
            command_name = "shoot";
            break;
        case ShooterCommand::pitch_down:
            command_name = "pitch_down";
            break;
        case ShooterCommand::pitch_up:
            command_name = "pitch_up";
            break;
        default:
            ROS_INFO("No such shooter command!");
            command_name = "fail";
    }

    return command_name;
}

void shooterCommandCallback(const std_msgs::UInt8::ConstPtr& msg)
{

    // switch case is to check validity of input command
    switch(msg->data)
    {
        case ShooterCommand::shoot:
        case ShooterCommand::pitch_down:
        case ShooterCommand::pitch_up:
            shooter_command = static_cast<ShooterCommand>(msg->data);
            ROS_INFO("Received shooter command [%s]", shooterCommandToString(shooter_command).c_str());
            new_data = 1;
            break;
        default:
            ROS_INFO("No such shooter command! Not moving shooter...");
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "shooter_controller_node");
    ros::NodeHandle nh;

    // setting up publishers and subscribers
    // TODO: set up publishers and subscribers 
    ros::Subscriber shooter_command_sub = 
    ros::Publisher shooter_fire_driver_command_pub = 
    ros::Publisher shooter_pitch_driver_command_pub = 

    ROS_INFO("Started shooter_controller_node!");

    std_msgs::UInt8 shooter_fire_driver_cmd;
    std_msgs::Int8 shooter_pitch_driver_cmd;

    new_data = 0;

    // pitch angle [-90, 90] degrees
    float curr_angle = 0; // down is -ve, up is +ve

    // main loop
    while (ros::ok())
    {
        ros::spinOnce();

        if (new_data)
        {
            // received new command data from laptop 
    
            // TODO: write the switch case
            // publish the shooter fire and pitch commands
            // you can look at robot_controller.py to see what kind of data should be send out
            // to fire: data = 1 should be published
            // to pitch: data = desired angle should be published (take note there is a max pitch angle)

            new_data = 0;
        }
    }

    return 0;
}
