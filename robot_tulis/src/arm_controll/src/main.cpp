#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include <cmath>
using namespace std;
class RobotArm {
public:
    RobotArm() {
        joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
        loop_rate = new ros::Rate(100);
        joint_positions = {0.0, 0.0, 0.0, 0.0, 0.0};
    }

    void executeMovements() {
        starting();
        yy();
        moveToHomePosition();
        ros::Duration(0.5).sleep();
        starting();
        uu();
        moveToHomePosition();
        ros::Duration(0.5).sleep();
        starting();
        aa();
        moveToHomePosition();
        ros::Duration(0.5).sleep();
        starting();
        nn();
        moveToHomePosition();
        ros::Duration(0.5).sleep();
        starting();
        aa();
        moveToHomePosition();
        ros::Duration(0.5).sleep();
        starting();
        bb();
        moveToHomePosition();
        ros::Duration(0.5).sleep();
        starting();
        ii();
        moveToHomePosition();
        ros::Duration(0.5).sleep();
        starting();
        nn();
        moveToHomePosition();
        ros::Duration(0.5).sleep();
        starting();
        aa();
        moveToHomePosition();
        ros::Duration(0.5).sleep();
        starting();
        rr();
        moveToHomePosition();
        ros::Duration(0.5).sleep();
        starting();
        aa();
        moveToHomePosition();


    }

private:
    ros::NodeHandle nh;
    ros::Publisher joint_state_pub;
    ros::Rate* loop_rate;
    vector<double> joint_positions;

    void updateAndPublish() {
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();
        joint_state.name = {"hip", "shoulder", "elbow", "gelang", "tulis"};
        joint_state.position = joint_positions;

        joint_state_pub.publish(joint_state);

        ros::spinOnce();
        loop_rate->sleep();
    }

    void moveToHomePosition() {
        vector<double> target_positions = {0, 0, 0, 0, 0};
        moveToTargetPosition(target_positions);
    }

    void starting() {
        vector<double> target_positions = {0.0, -1.08, 0.44, 0.67, 0};
        moveToTargetPosition(target_positions);
    }

    void moveDiagonalLeftUp() {
        vector<double> target_positions = {0.5, -1.26, 0.01, 0.7, 0.35};
        moveToTargetPosition(target_positions);
    }

    void moveDiagonalRightUp() {
        vector<double> target_positions = {-0.5, -1.26, 0.01, 0.7, 0.35};
        moveToTargetPosition(target_positions);
    }

    void moveDiagonalRightDown() {
        vector<double> target_positions = {-0.5, 0.6, -0.55, -0.03, 0.01};
        moveToTargetPosition(target_positions);
    }

    void moveDiagonalLeftDown() {
        vector<double> target_positions = {0.5, 0.6, -0.55, -0.03, 0.01};
        moveToTargetPosition(target_positions);
    }

    void moveVerticalDown() {
        vector<double> target_positions = {joint_positions[0], 0.6, -0.55, -0.03, 0.07};
        moveToTargetPosition(target_positions);
    }

    void moveHorizontalLeft() {
        vector<double> target_positions = {0.5, joint_positions[1], joint_positions[2], joint_positions[3], 0.1};
        moveToTargetPosition(target_positions);
    }

    void moveVerticalUp() {
        vector<double> target_positions = {joint_positions[0], -1.39, 0.03, 1.38, 0.0};
        moveToTargetPosition(target_positions);
    }

    void moveHorizontalRight() {
        vector<double> target_positions = {-0.5, joint_positions[1], joint_positions[2], joint_positions[3], 0.1};
        moveToTargetPosition(target_positions);
    }

    void yy() {
        ROS_INFO("Y");
        moveDiagonalRightUp();
        starting();
        moveDiagonalLeftUp();
        starting();
        moveVerticalDown();
        starting();
    }

    void uu() {
        ROS_INFO("U");
        moveVerticalUp();
        moveVerticalDown();
        moveHorizontalLeft();
        moveVerticalUp();
        moveVerticalDown();
        starting();
    }

    void aa(){
        ROS_INFO("A");
        moveVerticalDown();
        moveVerticalUp();
        moveDiagonalLeftDown();
        starting();
        moveHorizontalLeft();
    }

    void nn(){
        ROS_INFO("N");
        moveVerticalDown();
        moveVerticalUp();
        moveDiagonalLeftDown();
        moveDiagonalLeftUp();
        moveVerticalDown();
        starting();
    }

    void bb(){
        ROS_INFO("B");
        moveVerticalUp();
        moveHorizontalLeft();
        starting();
        moveDiagonalLeftDown();
        moveVerticalDown();
        starting();
    }

    void ii(){
        ROS_INFO("I");
        moveVerticalUp();
        moveVerticalDown();
        starting();
    }

    void rr(){
        ROS_INFO("R");
        moveVerticalUp();
        moveHorizontalLeft();
        starting();
        moveDiagonalLeftDown();
        starting();
    }

    void moveToTargetPosition(const vector<double>& target_positions) {
        int max_steps = 0;
        for (size_t i = 0; i < joint_positions.size() - 1; ++i) {
            max_steps = max(max_steps, static_cast<int>(abs(joint_positions[i] - target_positions[i]) * 100));
        }

        for (int step = 0; step <= max_steps; ++step) {
            bool all_reached = true;
            for (size_t i = 0; i < joint_positions.size() - 1; ++i) {
                if (abs(joint_positions[i] - target_positions[i]) > 0.001) {
                    joint_positions[i] += (target_positions[i] > joint_positions[i]) ? 0.01 : -0.01;
                    all_reached = false;
                }
            }

            updateAndPublish();

            if (all_reached) {
                break;
            }
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_arm_controller");
    
    RobotArm robot_arm;
    robot_arm.executeMovements();

    return 0;
}

// . . .
// . . .
// . . .


// ___________
