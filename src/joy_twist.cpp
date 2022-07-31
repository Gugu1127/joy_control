#include <geometry_msgs/Twist.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>

#include <thread>
#include <vector>
#include <string>

using namespace std;

class Trigger {
   private:
    int previousValue;

   public:
    Trigger() {
        previousValue = 0;
    }

    bool isActive(int currentValue) {
        bool returnValue = false;
        if (previousValue == 1 && currentValue == 0) {
            returnValue = true;
        }

        previousValue = currentValue;
        return returnValue;
    }
};

class Argument {
   public:
    int gear;
    bool ready;
    vector<pair<float, float> > record;
    Trigger trigger_L_bottom, trigger_R_bottom, trigger_B_bottom;
    bool marco_REC;
    bool back;
    Argument() {
        marco_REC = false;
        back = false;
    }
};

class Car_control {
   private:
    static Argument argument;
    static int gearNum;

   public:
    Car_control() {
        gearNum = 5;
    }

    geometry_msgs::Twist control(ros::Publisher publisher, float angular, float vol_forward, float vol_backward, int lb, int rb, int LB, int RB, int B) {
        geometry_msgs::Twist twist;
        if (lb == 0 && rb == 0) {
            argument.ready = true;
        }

        if (argument.ready) {
            if (lb ^ rb) {
                if (lb == 1) {
                    argument.gear--;
                } else {
                    argument.gear++;
                }

                argument.ready = false;
            }
        }

        if (argument.gear < 0) {
            argument.gear = 0;
        } else if (argument.gear >= gearNum) {
            argument.gear = gearNum - 1;
        }

        twist.linear.x = -(vol_forward - vol_backward) * 0.11 * (argument.gear + 1) / gearNum;
        twist.angular.z = angular * 0.45 * (argument.gear + 1) / gearNum;

        bool L_bottom_state = argument.trigger_L_bottom.isActive(LB);
        bool R_bottom_state = argument.trigger_R_bottom.isActive(RB);
        bool B_bottom_state = argument.trigger_B_bottom.isActive(B);
        if (L_bottom_state) {
            if (argument.marco_REC)
                argument.marco_REC = false;
            else
                argument.marco_REC = true;
        }

        if (R_bottom_state) {
            argument.record.clear();
        }

        if (B_bottom_state) {
            argument.back = true;
        }

        if (argument.back) {
            if (argument.record.size() > 0) {
                int index = argument.record.size() - 1;
                twist.linear.x = argument.record[index].first;
                twist.angular.z = argument.record[index].second;
                argument.record.erase(argument.record.begin() + index);
                return twist;
            } else {
                argument.back = false;
            }
        } else {
            if (abs(twist.linear.x) > 0.01 || abs(twist.angular.z) > 0.1)
                argument.record.push_back(move(pair<float, float>(-twist.linear.x, -twist.angular.z)));
        }

        return twist;
    }
};

int Car_control::gearNum;

Argument Car_control::argument;
class Joy_rosky {
   private:
    int RT_index, LT_index, axis_index, LB_index, RB_index;

    int L_bottom, R_bottom, B;

    string ifCollision;

   public:
    Joy_rosky() {
        ros::NodeHandle nodePtr;
        nodePtr.param<int>("RT_index", RT_index, 2);
        nodePtr.param<int>("LT_index", LT_index, 5);
        nodePtr.param<int>("axis_index", axis_index, 0);
        nodePtr.param<int>("LB_index", LB_index, 4);
        nodePtr.param<int>("RB_index", RB_index, 5);
        nodePtr.param<int>("L_bottom", L_bottom, 6);
        nodePtr.param<int>("R_bottom", R_bottom, 7);
        nodePtr.param<int>("B", B, 1);
        // create a publisher that will advertise on the command_velocity topic of the turtle
        publisher = nodePtr.advertise<geometry_msgs::Twist>("/rosky/cmd_vel", 1);

        // subscribe to the joystick topic for the input to drive the turtle
        subscriber = nodePtr.subscribe<sensor_msgs::Joy>("joy", 1, &Joy_rosky::joyCallback, this);

        subscriber_collision = nodePtr.subscribe("/collision", 1, &Joy_rosky::collisionCallback, this);
    }

   private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
        Car_control controller;
        geometry_msgs::Twist twist = move(controller.control(publisher, joy->axes[axis_index], (-joy->axes[RT_index] + 1) / 2, (-joy->axes[LT_index] + 1) / 2, joy->buttons[LB_index], joy->buttons[RB_index], joy->buttons[L_bottom], joy->buttons[R_bottom], joy->buttons[B]));
        if (ifCollision == "true" && twist.linear.x > 0) {
            twist.linear.x = 0;
        }

        publisher.publish(twist);
    }

    void collisionCallback(const std_msgs::String& msg) {
        ifCollision = msg.data;
    }

    ros::Publisher publisher;
    ros::Subscriber subscriber;
    ros::Subscriber subscriber_collision;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "joy_rosky");
    Joy_rosky joy_rosky;
    ros::spin();
}