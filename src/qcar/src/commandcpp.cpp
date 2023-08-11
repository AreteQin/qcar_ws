#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Vector3Stamped.h"
#include <sstream>
#include <termios.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <string>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_A 0x61
#define KEYCODE_Q 0x71
#define KEYCODE_SPACE 0x20

class Teleop {
public:
    Teleop();

    void keyLoop();

private:
    ros::NodeHandle nh_;
    double throttle, steering;
    ros::Publisher command_pub;
};

Teleop::Teleop() : throttle(0), steering(0) {
    ros::NodeHandle n;
    command_pub = n.advertise<geometry_msgs::Vector3Stamped>("/qcar/user_command", 1000);
}

int kfd = 0;
double steering_bias = -0.06; // bias to modify the steering angle to make the car go straight
struct termios cooked, raw;

void quit(int sig) {
    tcsetattr(kfd, TCSANOW, &cooked);
    ros::shutdown();
    exit(0);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "command_cpp_node");
    Teleop teleop;
    ros::Rate loop_rate(1000);
    signal(SIGINT, quit);
    teleop.keyLoop();
    return (0);
}

void Teleop::keyLoop() {
    char c;
    bool dirty = false;
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move the robot.");
    puts("Press the space bar to stop the robot.");
//    puts("a/z - Increase/decrease linear velocity");
//    puts("s/x - Increase/decrease angular velocity");
    puts("Press q to quit");
    throttle = 0;
    steering = steering_bias;
    geometry_msgs::Vector3Stamped command_msgs;
    while (ros::ok()) {
        // get the next event from the keyboard
        if (read(kfd, &c, 1) < 0) {
            perror("read():");
            exit(-1);
        }
//        char printable[100];
        ROS_DEBUG("value: 0x%02X\n", c);
        switch (c) {
            case KEYCODE_L:
                ROS_DEBUG("LEFT");
                throttle = throttle;
                if (steering < 0.3) {
                    steering = steering + 0.1;
                } else {
                    steering = steering;
                }
                dirty = true;
                break;
            case KEYCODE_R:
                ROS_DEBUG("RIGHT");
                throttle = throttle;
                if (steering > -0.5) {
                    steering = steering - 0.1;
                } else {
                    steering = steering;
                }
                dirty = true;
                break;
            case KEYCODE_U:
                ROS_DEBUG("UP");
                if (throttle == 0) {
                    throttle = 0.05;
                } else if (throttle < 0.25) {
                    throttle = throttle + 0.03;
                } else { throttle = throttle; }
                steering = steering_bias;
                dirty = true;
                break;
            case KEYCODE_D:
                ROS_DEBUG("DOWN");
                if (throttle == 0) {
                    throttle = -0.05;
                } else if (throttle > -0.25) {
                    throttle = throttle - 0.03;
                } else { throttle = throttle; }
                steering = steering_bias;
                dirty = true;
                break;
            case KEYCODE_SPACE:
                throttle = 0;
                steering = steering_bias;
                ROS_DEBUG("STOP");
                dirty = true;
                break;
            case KEYCODE_Q:
                ROS_DEBUG("QUIT");
                ROS_INFO_STREAM("You quit the teleop successfully");
                quit(0);
                return;
        }
        std_msgs::String keyboard_input;
        if (dirty == true) {
            command_msgs.header.stamp = ros::Time::now();
            command_msgs.header.frame_id = std::string("command_input");
            command_msgs.vector.x = throttle;
            command_msgs.vector.y = steering;
            command_pub.publish(command_msgs);
            dirty = false;
        }
    }
    return;
}