#include <common/lcm_config.h>
#include <mbot/mbot_channels.h>
#include <lcmtypes/mbot_arm_block_t.hpp>
#include <lcmtypes/mbot_arm_status_t.hpp>
#include <lcmtypes/robot_path_t.hpp>
#include <lcmtypes/odometry_t.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <lcmtypes/path_status_t.hpp>
#include <lcmtypes/mbot_motor_command_t.hpp>
#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <unistd.h>
#include <cmath>
#include <signal.h>

#define SLAM_POSE_CHANNEL "SLAM_POSE"

class GrabBlock
{
public:
    GrabBlock(lcm::LCM * instance) : lcmInstance(instance) {
        grabCommand.status = ATTEMPT_GRASP;
    };
    ~GrabBlock() {};
    void handleBlock(const lcm::ReceiveBuffer* buf, const std::string& channel, const mbot_arm_status_t* arm_state) {
        std::cout << "arm_state:" << unsigned(arm_state->status) << std::endl;
        robot_path_t path;
        if (arm_state->status == DETECT_BLOCKS) {
            //publish the path (the destination will be the position of the block)
            path.path.resize(2);
            path.path[0].x = cur_pose_.x;
            path.path[0].y = cur_pose_.y;
            path.path[0].theta = cur_pose_.theta;

            float x = arm_state->block.pose.x;
            float y = arm_state->block.pose.y;
            float r = sqrtf(x*x + y*y);
            float d = r - 0.18;
            x = d*sin(arm_state->block.pose.theta);
            y = d*cos(arm_state->block.pose.theta);

            path.path[1].x = cur_pose_.x+x;
            path.path[1].y = cur_pose_.y+y;
            path.path[1].theta = cur_pose_.theta+arm_state->block.pose.theta;

            path.path_length = 2;
            path.path_type = 5;
            lcmInstance->publish(CONTROLLER_PATH_CHANNEL, &path);
        }
        else if (arm_state->status == DETECT_BLOCK_IN_RANGE) {
            //make sure the robot is stopped
            mbot_motor_command_t cmd;
            cmd.trans_v = 0;
            cmd.angular_v = 0;
            lcmInstance->publish(MBOT_MOTOR_COMMAND_CHANNEL, &cmd);
            path.path.resize(1);
            path.path[0].x = cur_pose_.x;
            path.path[0].y = cur_pose_.y;
            path.path[0].theta = cur_pose_.theta;
            path.path_length = 1;
            path.path_type = 5;
            lcmInstance->publish(CONTROLLER_PATH_CHANNEL, &path);
            sleep(0.5);
            //publish the command to arm
            lcmInstance->publish(ARM_COMMAND_CHANNEL, &grabCommand);
        }
    }

    void handlePathUpdate(const lcm::ReceiveBuffer* buf, const std::string& channel, const path_status_t* state)
    {
        path_state = state->completed;
        if (path_state == 1)
            lcmInstance->publish(ARM_COMMAND_CHANNEL, &grabCommand);
    }

    void handlePose(const lcm::ReceiveBuffer* buf, const std::string& channel, const pose_xyt_t* pose)
    {
        cur_pose_.x = pose->x;
        cur_pose_.y = pose->y;
        cur_pose_.theta = pose->theta;
    }
private:
    lcm::LCM * lcmInstance;
    int8_t path_state = 0;
    mbot_arm_status_t grabCommand;
    odometry_t cur_pose_;
    const int8_t SYSTEM_IDLE = 0;
    const int8_t DETECT_BLOCKS = 1;
    const int8_t DETECT_BLOCK_IN_RANGE = 2;
    const int8_t ATTEMPT_GRASP = 3;
};

int main(int argc, char** argv) {
    lcm::LCM lcmInstance(MULTICAST_URL);

    GrabBlock grabblock(&lcmInstance);
    lcmInstance.subscribe(ARM_STATE_CHANNEL, &GrabBlock::handleBlock, &grabblock);
    lcmInstance.subscribe(PATH_STATUS_CHANNEL, &GrabBlock::handlePathUpdate, &grabblock);
    lcmInstance.subscribe(SLAM_POSE_CHANNEL, &GrabBlock::handlePose, &grabblock);

    signal(SIGINT, exit);

    while(true)
    {
        lcmInstance.handleTimeout(50);  // update at 20Hz minimum
    }

    return 0;
}
