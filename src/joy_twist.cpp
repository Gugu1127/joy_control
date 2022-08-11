#include <geometry_msgs/Twist.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int8.h>

#include <string>
#include <thread>
#include <vector>

using namespace std;

enum Position {
    top,
    top_R,
    top_L,
    bottom,
    bottom_R,
    bottom_L
};

class Trigger {
   protected:
    int previousValue;

   public:
    Trigger() {
        previousValue = 0;
    }

    bool isActive(int currentValue) {
        bool returnValue = false;
        if (previousValue != 0 && currentValue == 0) {
            returnValue = true;
        }

        previousValue = currentValue;
        return returnValue;
    }
};

class MultiTrigger : public Trigger {
   public:
    int isActive(int currentValue) {
        int returnValue = 0;
        if (previousValue < 0 && currentValue == 0) {
            returnValue = -1;
        } else if (previousValue > 0 && currentValue == 0) {
            returnValue = 1;
        }

        previousValue = currentValue;
        return returnValue;
    }
};

class Argument {
   public:
    int gear;
    int window_horizon_num,window_vertical_num;

    bool ready;
    vector<pair<float, float> > record;
    Trigger trigger_L_bottom, trigger_R_bottom, trigger_B_bottom;
    MultiTrigger trigger_horizon_axis, trigger_vertical_axis;
    bool marco_REC;
    bool back;
    Argument() {
        window_horizon_num = 0;
        window_vertical_num = 0;
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

    geometry_msgs::Twist control(ros::Publisher publisher, ros::Publisher publisher_sliding_window, float angular, float vol_forward, float vol_backward, int lb, int rb, int LB, int RB, int B, int horizon_axis_index, int vertical_axis_index) {
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
        int horizon_axis_state = argument.trigger_horizon_axis.isActive(horizon_axis_index);
        int vertical_axis_state = argument.trigger_vertical_axis.isActive(vertical_axis_index);

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
        if (horizon_axis_state < 0) {
            argument.window_horizon_num = 10;
        } else if (horizon_axis_state > 0) {
            argument.window_horizon_num = -10;
        } else {
            argument.window_horizon_num = 0;
        }

        if (vertical_axis_state > 0) {
            argument.window_vertical_num = 1;
        } else if (vertical_axis_state < 0) {
            argument.window_vertical_num = -1;
        } else {
            argument.window_vertical_num = 0;
        }
        std_msgs::Int8 window_size;
        window_size.data = argument.window_vertical_num + argument.window_horizon_num;
        publisher_sliding_window.publish(window_size);
        return twist;
    }
};

int Car_control::gearNum;

Argument Car_control::argument;
class Joy_rosky {
   private:
    int RT_index, LT_index, axis_index, LB_index, RB_index, vertical_axis_index, horizon_axis_index;

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
        nodePtr.param<int>("horizon_axis_index", horizon_axis_index, 6);
        nodePtr.param<int>("vertical_axis_index", vertical_axis_index, 7);

        // create a publisher that will advertise on the command_velocity topic of the turtle
        publisher = nodePtr.advertise<geometry_msgs::Twist>("/rosky/cmd_vel", 1);
        publisher_sliding_window = nodePtr.advertise<std_msgs::Int8>("/sliding_window/set", 1);

        // subscribe to the joystick topic for the input to drive the turtle
        subscriber = nodePtr.subscribe<sensor_msgs::Joy>("joy", 1, &Joy_rosky::joyCallback, this);

        subscriber_collision = nodePtr.subscribe("/collision", 1, &Joy_rosky::collisionCallback, this);
    }

   private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
        Car_control controller;
        geometry_msgs::Twist twist = move(controller.control(publisher, publisher_sliding_window, joy->axes[axis_index], (-joy->axes[RT_index] + 1) / 2, (-joy->axes[LT_index] + 1) / 2, joy->buttons[LB_index], joy->buttons[RB_index], joy->buttons[L_bottom], joy->buttons[R_bottom], joy->buttons[B], joy->axes[horizon_axis_index], joy->axes[vertical_axis_index]));
        if (twist.linear.x > 0) {
            if (collisionState[top] == 0) {
                twist.linear.x = 0;
            } else if (collisionState[top] == 2 && twist.linear.x > 0.05) {
                twist.linear.x = 0.05;
            }

        } else {
            if (collisionState[bottom] == 0) {
                twist.linear.x = 0;
            } else if (collisionState[bottom] == 2 && twist.linear.x < -0.05) {
                twist.linear.x = -0.05;
            }
        }

        if (twist.angular.z < 0) {
            if (collisionState[top_R] == 0 || collisionState[bottom_L] == 0) {
                if (twist.angular.z = 0)
                    ;
            }
        } else {
            if (collisionState[top_L] == 0 || collisionState[bottom_R] == 0) {
                twist.angular.z = 0;
            }
        }

        publisher.publish(twist);
    }

    int collisionState[6];
    void collisionCallback(const std_msgs::Int16MultiArray& msg) {
        for (int i = 0; i < 6; i++) {
            collisionState[i] = msg.data[i];
        }
    }

    ros::Publisher publisher;
    ros::Publisher publisher_sliding_window;
    ros::Subscriber subscriber;
    ros::Subscriber subscriber_collision;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "joy_rosky");
    Joy_rosky joy_rosky;
    ros::spin();
}
