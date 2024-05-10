#include <ros/ros.h>
#include <std_msgs/UInt8.h>

enum FlipperCommand
{
    open_ = 1, 
    close_ = 2,
    manual_ = 3,
    autonomous_ = 4
};

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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "flipper_keyboard_node");
    ros::NodeHandle nh;

    ros::Publisher flipper_mode_pub = nh.advertise<std_msgs::UInt8>("flipper_mode", 1);
    ros::Publisher flipper_movement_command_pub = nh.advertise<std_msgs::UInt8>("flipper_movement_command", 1);

    std::string input_line;
    int cmd;
    std_msgs::UInt8 flipper_movement_command;
    std_msgs::UInt8 flipper_mode;

    while (ros::ok())
    {
        std::cout << "Please choose flipper command: \n" 
                     << "1. open flipper\n" 
                     << "2. close flipper\n" 
                     << "3. manual mode\n" 
                     << "4. autonomous mode\n";

        getline(std::cin, input_line); // read in input from terminal

        try { cmd = stoi(input_line); }
        catch (...) { cmd = 6; }

        switch (cmd)
        {
            case FlipperCommand::open_:
                // open flipper
                std::cout << "Received command: [ 1. open flipper ]\n\n";
                flipper_movement_command.data = FlipperState::open;
                flipper_movement_command_pub.publish(flipper_movement_command);
                break;
            case FlipperCommand::close_:
                //close flipper
                std::cout << "Received command: [ 2. close flipper ]\n\n";
                flipper_movement_command.data = FlipperState::closed;
                flipper_movement_command_pub.publish(flipper_movement_command);
                break;
            case FlipperCommand::manual_:
                // manual mode
                std::cout << "Received command: [ 3. manual mode ]\n\n";
                flipper_mode.data = FlipperMode::manual;
                flipper_mode_pub.publish(flipper_mode);
                break;
            case FlipperCommand::autonomous_:
                // autonomous mode
                std::cout << "Received command: [ 4. autonomous mode ]\n\n";
                flipper_mode.data = FlipperMode::autonomous;
                flipper_mode_pub.publish(flipper_mode);
                break;
            default:
                std::cout << "No such command! Ignoring command...\n\n";
        }

    }

    return 0;
}
