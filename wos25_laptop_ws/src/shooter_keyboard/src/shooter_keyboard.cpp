#include <ros/ros.h>
#include <std_msgs/UInt8.h>

enum ShooterCommand
{
    shoot = 1, 
    pitch_down = 9,
    pitch_up = 0

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "shooter_keyboard_node");
    ros::NodeHandle nh;

    //ros::Publisher shooter_mode_pub = nh.advertise<std_msgs::UInt8>("shooter_mode", 1);
    ros::Publisher shooter_movement_command_pub = nh.advertise<std_msgs::UInt8>("shooter_command", 1);

    std::string input_line;
    int cmd;
    std_msgs::UInt8 shooter_movement_command;
    //std_msgs::UInt8 shooter_mode;

    while (ros::ok())
    {
        std::cout << "Please choose shooter command: \n" 
                     << "Shoot -------[ 1 ]\n" 
                     << "Pitch down ----[ 9 ]\n"
                     << "Pitch up ---[ 0 ]\n\n";

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
            case ShooterCommand::pitch_down:
                //pitch rubber band gun down
                std::cout << "Received command: [ Pitch down ]\n\n";
                shooter_movement_command.data = ShooterCommand::pitch_down;
                shooter_movement_command_pub.publish(shooter_movement_command);
                break;
            case ShooterCommand::pitch_up:
                //pitch rubber band gun up
                std::cout << "Received command: [ Pitch up ]\n\n";
                shooter_movement_command.data = ShooterCommand::pitch_up;
                shooter_movement_command_pub.publish(shooter_movement_command);
                break;
            // case ShooterCommand::manual_:
            //     // manual mode
            //     std::cout << "Received command: [ 3. manual mode ]\n\n";
            //     flipper_mode.data = FlipperMode::manual;
            //     flipper_mode_pub.publish(flipper_mode);
            //     break;
            // case ShooterCommand::autonomous_:
            //     // autonomous mode
            //     std::cout << "Received command: [ 4. autonomous mode ]\n\n";
            //     flipper_mode.data = FlipperMode::autonomous;
            //     flipper_mode_pub.publish(flipper_mode);
            //     break;
            default:
                std::cout << "No such command! Ignoring command...\n\n";
        }

    }

    return 0;
}
