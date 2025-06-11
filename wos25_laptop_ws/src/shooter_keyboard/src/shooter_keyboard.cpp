#include <ros/ros.h>
#include <std_msgs/UInt8.h>

enum ShooterCommand
{
    // the numbers here correspond to the key to press
    shoot = 1, 
    pitch_down = 9,
    pitch_up = 0
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "shooter_keyboard_node");
    ros::NodeHandle nh;

    // TODO: set up publisher
    ros::Publisher shooter_movement_command_pub = 

    std::string input_line;
    int cmd;
    std_msgs::UInt8 shooter_movement_command;

    while (ros::ok())
    {
        std::cout << "Please choose shooter command: \n" 
                     << "Shoot -------[ 1 ]\n" 
                     << "Pitch down --[ 9 ]\n"
                     << "Pitch up ----[ 0 ]\n\n";

        getline(std::cin, input_line); // read in input from terminal

        try { cmd = stoi(input_line); }
        catch (...) { cmd = 6; }

        std::cout << "\n";

        switch (cmd)
        {
            case ShooterCommand::shoot:
                // fire the rubber band gun
                std::cout << "Received command: [ Shoot ]\n\n";
                shooter_movement_command.data = ShooterCommand::shoot;
                shooter_movement_command_pub.publish(shooter_movement_command);
                break;
            // TODO: complete the switch case
            default:
                std::cout << "No such command! Ignoring command...\n\n";
        }

    }

    return 0;
}
