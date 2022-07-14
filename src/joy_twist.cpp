#include <geometry_msgs/Twist.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>


using namespace std;

class Car_control {
    private:
    static int gear;
    static bool ready;
   public:
    geometry_msgs::Twist control(float angular, float vol_forward, float vol_backward, int lb, int rb) {
        geometry_msgs::Twist twist;
        if (lb == 0 && rb == 0) {
            ready = true;
        }

        if (ready) {
            if (lb ^ rb) {
                if (lb == 1) {
                    gear--;
                } else {
                    gear++;
                }

                ready = false;
            }
        }

        if (gear < 0) {
            gear = 0;
        } else if (gear >= 3) {
            gear = 2;
        }

        twist.linear.x = -(vol_forward - vol_backward) * 2.5 * (gear + 1) / 3;
        twist.angular.z = angular * 2.5 * (gear + 1) / 3;
        return twist;
    }
};

int Car_control::gear = 0;
bool Car_control::ready = false;

class Joy_rosky {
   private:
    int RT_index, LT_index, axis_index, LB_index, RB_index;

   public:
    Joy_rosky() {
        ros::NodeHandle nodePtr;
        nodePtr.param<int>("RT_index", RT_index, 2);
        nodePtr.param<int>("LT_index", LT_index, 5);
        nodePtr.param<int>("axis_index", axis_index, 0);
        nodePtr.param<int>("LB_index", LB_index, 4);
        nodePtr.param<int>("RB_index", RB_index, 5);

        // create a publisher that will advertise on the command_velocity topic of the turtle
        publisher = nodePtr.advertise<geometry_msgs::Twist>("/rosky/cmd_vel", 1);

        // subscribe to the joystick topic for the input to drive the turtle
        subscriber = nodePtr.subscribe<sensor_msgs::Joy>("joy", 10, &Joy_rosky::joyCallback, this);
    }

   private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
        Car_control controller;
        publisher.publish(controller.control(joy->axes[axis_index], (-joy->axes[RT_index] + 1) / 2, (-joy->axes[LT_index] + 1) / 2, joy->buttons[LB_index], joy->buttons[RB_index]));
    }

    ros::Publisher publisher;
    ros::Subscriber subscriber;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "joy_rosky");
    Joy_rosky joy_rosky;

    ros::spin();
}