#include <ros/ros.h>
#include <std_msgs/UInt8.h>

// % means student can set whatever they want 

// global variables 
enum FlipperState 
{
    open = 0,
    closed = 1
};

enum FlipperMode
{
    manual = 0,
    autonomous = 1
};

enum IRState
{
    off = 0,
    on = 1
};


FlipperMode flipper_mode_command;
FlipperState flipper_movement_command;
IRState ir_state;

//====================================================================================================================

std::string flipperModeToString(FlipperMode mode)
{
    std::string mode_name;

    switch(mode)
    {
        case FlipperMode::manual:
            mode_name = "manual";
            break;
        case FlipperMode::autonomous:
            mode_name = "autonomous";
            break;
        default:
            ROS_INFO("No such flipper mode!");
            mode_name = "fail";
    }

    return mode_name;
}

std::string flipperStateToString(FlipperState state)
{
    std::string state_name;

    switch(state)
    {
        case FlipperState::open:
            state_name = "open";
            break;
        case FlipperState::closed:
            state_name = "closed";
            break;
        default:
            ROS_INFO("No such flipper state!");
            state_name = "fail";
    }

    return state_name;
}

std::string IRStateToString(IRState state)
{
    std::string state_name;

    switch(state)
    {
        case IRState::off:
            state_name = "off";
            break;
        case IRState::on:
            state_name = "on";
            break;
        default:
            ROS_INFO("No such IR state!");
            state_name = "fail";
    }

    return state_name;
}

void flipperModeCallback(const std_msgs::UInt8::ConstPtr& msg)
{
    // switch case is to check validity of input command
    switch(msg->data)
    {
        case FlipperMode::manual:
        case FlipperMode::autonomous:
            flipper_mode_command = static_cast<FlipperMode>(msg->data);
            ROS_INFO("Received flipper mode command [%s]", flipperModeToString(flipper_mode_command).c_str());
            break;
        default:
            ROS_INFO("No such flipper mode! Not changing flipper mode...");
    }
}

void flipperMovementCommandCallback(const std_msgs::UInt8::ConstPtr& msg)
{

    // switch case is to check validity of input command
    switch(msg->data)
    {
        case FlipperState::open:
        case FlipperState::closed:
            flipper_movement_command = static_cast<FlipperState>(msg->data);
            ROS_INFO("Received flipper movement command [%s]", flipperStateToString(flipper_movement_command).c_str());
            break;
        default:
            ROS_INFO("No such flipper movement command! Not changing flipper state...");
    }
}

void IRStateCallback(const std_msgs::UInt8::ConstPtr& msg)
{
    // switch case is to check validity of input command
    switch(msg->data)
    {
        case IRState::off:
        case IRState::on:
            ir_state = static_cast<IRState>(msg->data);
            ROS_INFO("Received IR state [%s]", IRStateToString(ir_state).c_str());
            break;
         default:
            ROS_INFO("No such IR state! Not changing IR state...");   
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "flipper_controller_node");
    ros::NodeHandle nh;

    // setting up publishers and subscribers
    ros::Subscriber flipper_mode_sub = nh.subscribe("flipper_mode", 1, flipperModeCallback);
    ros::Subscriber flipper_movement_command_sub = nh.subscribe("flipper_movement_command", 1, flipperMovementCommandCallback);
    ros::Subscriber IR_state_sub = nh.subscribe("IR_state", 1, IRStateCallback);
    ros::Publisher flipper_driver_command_pub = nh.advertise<std_msgs::UInt8>("flipper_driver_command", 1);

    ROS_INFO("Started flipper_controller_node!");

    //init vars
    flipper_mode_command = FlipperMode::manual;
    flipper_movement_command = FlipperState::closed;
    ir_state = IRState::off;

    FlipperMode flipper_mode = FlipperMode::manual;
    FlipperState flipper_state = FlipperState::closed;

    std_msgs::UInt8 flipper_driver_command;

    // main loop
    while (ros::ok())
    {
        ros::spinOnce();

        //check for flipper mode change (redundant???--------------------------------------------*****)
        if (flipper_mode != flipper_mode_command)
        {
            ROS_INFO("Flipper mode change: %s --> %s", flipperModeToString(flipper_mode).c_str(), flipperModeToString(flipper_mode_command).c_str());
            flipper_mode = flipper_mode_command;
        }

        switch (flipper_mode)
        {
            case FlipperMode::manual:
                // check for new flipper movement command
                if (flipper_state != flipper_movement_command)
                {
                    ROS_INFO("Flipper state change: %s --> %s", flipperStateToString(flipper_state).c_str(), flipperStateToString(flipper_movement_command).c_str());
                    flipper_state = flipper_movement_command;
                    flipper_driver_command.data = flipper_state;
                    flipper_driver_command_pub.publish(flipper_driver_command);
                }
                break;
            case FlipperMode::autonomous:
                if (ir_state == IRState::on)
                {
                    // open flipper to flick
                    flipper_state = FlipperState::open;
                    flipper_driver_command.data = flipper_state;
                    flipper_driver_command_pub.publish(flipper_driver_command);
                    ros::Duration(2).sleep(); //seconds
                    //close flipper
                    flipper_state = FlipperState::closed;
                    flipper_driver_command.data = flipper_state;
                    flipper_driver_command_pub.publish(flipper_driver_command);
                    ros::Duration(3).sleep(); //seconds
                }
                // if IR is off, no need do anything
                
                flipper_movement_command = flipper_state; // for if anyhow press command to move flipper in auto mode
                break;
            default:
                ROS_ERROR("Code has bug in managing flipper mode!!!");  
        }

    }

    return 0;
}
