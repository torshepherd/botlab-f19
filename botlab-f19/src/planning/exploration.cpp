#include <planning/exploration.hpp>
#include <planning/frontiers.hpp>
#include <planning/planning_channels.h>
#include <common/grid_utils.hpp>
#include <common/timestamp.h>
#include <mbot/mbot_channels.h>
#include <slam/slam_channels.h>
#include <fstream>
#include <iostream>
#include <queue>
#include <unistd.h>
#include <cassert>
#include <common/angle_functions.hpp>

const float kReachedPositionThreshold = 0.05f;  // must get within this distance of a position for it to be explored

// Define an equality operator for poses to allow direct comparison of two paths
bool operator==(const pose_xyt_t& lhs, const pose_xyt_t& rhs)
{
    return (lhs.x == rhs.x) && (lhs.y == rhs.y) && (lhs.theta == rhs.theta);
}


Exploration::Exploration(int32_t teamNumber,
                         lcm::LCM* lcmInstance)
: teamNumber_(teamNumber)
, state_(exploration_status_t::STATE_INITIALIZING)
, currFrontier_(-1)
, haveNewPose_(false)
, haveNewMap_(false)
, haveHomePose_(false)
, lcmInstance_(lcmInstance)
, pathReceived_(false)
, planningTime_(0)
, flagGrab_(false)
, haveMap_(false)
, flagGrabFinish_(false)
, grabBlockNum_(0)
, maxGrabBlockNum_(6)
, blockId_(-1)
, haveTargetBlockOrPose_(false)
, flagNewBlockRequestMessage_(false)
, flagGrabStatusUpdateMessage_(false)
, flagPoseStackInitialized_(false)
{

    if (flagGrab_) {
        state_ = exploration_status_t::GRAB_INITIALIZING;
    }
    assert(lcmInstance_);   // confirm a nullptr wasn't passed in

    lcmInstance_->subscribe(SLAM_MAP_CHANNEL, &Exploration::handleMap, this);
    lcmInstance_->subscribe(SLAM_POSE_CHANNEL, &Exploration::handlePose, this);
    lcmInstance_->subscribe(MESSAGE_CONFIRMATION_CHANNEL, &Exploration::handleConfirmation, this);
    lcmInstance_->subscribe(NEXT_BLOCK_RETURN_CHANNEL, &Exploration::handleBlockRequest, this);
    lcmInstance_->subscribe(ARM_BLOCK_STATUS_CHANNEL, &Exploration::handleGrabStatusUpdate, this);

    // Send an initial message indicating that the exploration module is initializing. Once the first map and pose are
    // received, then it will change to the exploring map state.
    exploration_status_t status;
    status.utime = utime_now();
    status.team_number = teamNumber_;
    status.state = exploration_status_t::STATE_INITIALIZING;
    status.status = exploration_status_t::STATUS_IN_PROGRESS;

    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);

    block_request_.new_block = 0;

    MotionPlannerParams params;
    params.robotRadius = 0.15;
    planner_.setParams(params);
}


bool Exploration::exploreEnvironment()
{
    // (state_ != exploration_status_t::STATE_COMPLETED_EXPLORATION)
    //     && (state_ != exploration_status_t::STATE_FAILED_EXPLORATION)
    while(true)
    {
        // If data is ready, then run an update of the exploration routine
        if(isReadyToUpdate())
        {
            runExploration();
        }
        // Otherwise wait a bit for data to arrive
        else
        {
            usleep(10000);
        }

        if(flagGrab_) {
            if( (state_ == exploration_status_t::GRAB_COMPLETED)
            || (state_ == exploration_status_t::GRAB_FAILED)) {
                break;
            }
        }
        else {
            if( (state_ == exploration_status_t::STATE_COMPLETED_EXPLORATION)
            || (state_ == exploration_status_t::STATE_FAILED_EXPLORATION)) {
                break;
            }
        }
    }

    // If the state is completed, then we didn't fail
    return state_ == exploration_status_t::STATE_COMPLETED_EXPLORATION;
}

void Exploration::handleBlockRequest(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const block_request_t* block_request_back)
{
    block_request_back_.new_block = block_request_back->new_block;
    block_request_back_.block = block_request_back->block;
    flagNewBlockRequestMessage_ = true;
}

void Exploration::handleGrabStatusUpdate(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const mbot_picking_status_t* grab_status)
{
    if (nextStateFromArm_ != grab_status->status) {
        nextStateFromArm_ = grab_status->status;
        flagGrabStatusUpdateMessage_ = true;
    }
}

void Exploration::handleMap(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const occupancy_grid_t* map)
{

    std::lock_guard<std::mutex> autoLock(dataLock_);
    incomingMap_.fromLCM(*map);
    haveNewMap_ = true;
    haveMap_ = true;
}


void Exploration::handlePose(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const pose_xyt_t* pose)
{

    std::lock_guard<std::mutex> autoLock(dataLock_);
    incomingPose_ = *pose;
    haveNewPose_ = true;
}

void Exploration::handleConfirmation(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const message_received_t* confirm)
{
    std::lock_guard<std::mutex> autoLock(dataLock_);
    if(confirm->channel == CONTROLLER_PATH_CHANNEL && confirm->creation_time == most_recent_path_time) pathReceived_ = true;
}

bool Exploration::isReadyToUpdate(void)
{
    std::lock_guard<std::mutex> autoLock(dataLock_);
    if (flagGrab_) {
        return haveMap_ && haveNewPose_;
    }
    return haveNewMap_ && haveNewPose_;
}


void Exploration::runExploration(void)
{
    assert(isReadyToUpdate());

    copyDataForUpdate();

    if(flagGrab_) {
        grabStateMachine();
    }
    else {
        executeStateMachine();
    }
}


void Exploration::copyDataForUpdate(void)
{
    std::lock_guard<std::mutex> autoLock(dataLock_);

    // Only copy the map if a new one has arrived because it is a costly operation
    if(haveNewMap_)
    {
        currentMap_ = incomingMap_;
        haveNewMap_ = false;
    }

    // Always copy the pose because it is a cheap copy
    currentPose_ = incomingPose_;
    haveNewPose_ = false;

    // The first pose received is considered to be the home pose
    if(!haveHomePose_)
    {
        homePose_ = incomingPose_;
        putPose_ = homePose_;
        putPose_.theta = wrap_to_pi(homePose_.theta + MY_PI);
        putPose_.x -= 0.20;
        haveHomePose_ = true;
        std::cout << "INFO: Exploration: Set home pose:" << homePose_.x << ',' << homePose_.y << ','
            << homePose_.theta << '\n';
    }
}


void Exploration::executeStateMachine(void)
{
    bool stateChanged = false;
    int8_t nextState = state_;

    // Save the path from the previous iteration to determine if a new path was created and needs to be published
    robot_path_t previousPath = currentPath_;

    // Run the state machine until the state remains the same after an iteration of the loop
    do
    {
        switch(state_)
        {
            case exploration_status_t::STATE_INITIALIZING:
                nextState = executeInitializing();
                break;
            case exploration_status_t::STATE_EXPLORING_MAP:
                nextState = executeExploringMap(stateChanged);
                break;

            case exploration_status_t::STATE_RETURNING_HOME:
                nextState = executeReturningHome(stateChanged);
                break;

            case exploration_status_t::STATE_COMPLETED_EXPLORATION:
                nextState = executeCompleted(stateChanged);
                break;

            case exploration_status_t::STATE_FAILED_EXPLORATION:
                nextState = executeFailed(stateChanged);
                break;
        }

        stateChanged = nextState != state_;
        state_ = nextState;

    } while(stateChanged);

    //if path confirmation was not received, resend path
    if(!pathReceived_)
    {
        std::cout << "the current path was not received by motion_controller, attempting to send again:\n";

        std::cout << "timestamp: " << currentPath_.utime << "\n";

        for(auto pose : currentPath_.path){
            std::cout << "(" << pose.x << "," << pose.y << "," << pose.theta << "); ";
        }std::cout << "\n";

        lcmInstance_->publish(CONTROLLER_PATH_CHANNEL, &currentPath_);
    }

    //if path changed, send current path
    if(previousPath.path != currentPath_.path)
    {

        std::cout << "INFO: Exploration: A new path was created on this iteration. Sending to Mbot:\n";

        std::cout << "path timestamp: " << currentPath_.utime << "\npath: ";

        for(auto pose : currentPath_.path){
            std::cout << "(" << pose.x << "," << pose.y << "," << pose.theta << "); ";
        }std::cout << "\n";

        lcmInstance_->publish(CONTROLLER_PATH_CHANNEL, &currentPath_);

        pathReceived_ = false;
        most_recent_path_time = currentPath_.utime;
    }

}

//abcd

void Exploration::readPoseStack(void) {
    std::vector<std::vector<int>> intPoseStack;
    std::vector<int> a_int_pose;
    a_int_pose.push_back(138);
    a_int_pose.push_back(114);
    intPoseStack.push_back(a_int_pose);

    planner_.setMap(currentMap_);

    poseStack_.clear();
    // for(size_t i = 0; i < intPoseStack.size(); i++) {
    //     pose_xyt_t a_pose = planner_.poseFromIntXY(intPoseStack[i][0], intPoseStack[i][1]);
    //     poseStack_.push_back(a_pose);
    //     printf("a_pose = %f, %f", a_pose.x, a_pose.y);
    // }    

}

bool Exploration::nextBlockOrPose(int16_t& blockId, pose_xyt_t& blockPose) {
    // TODOTHIS
    bool flagNext = false;
    if (grabBlockNum_ >= maxGrabBlockNum_) {
        return false;
    }
    block_request_.new_block++;
     
    while(true) {
        lcmInstance_->publish(NEXT_BLOCK_REQUEST_CHANNEL, &block_request_);
        // printf("block_request_ = %d, %d, %d, sizeof(tag_id) = %d\n",  (int) block_request_.utime, (int) block_request_.new_block, (int) block_request_.block.tag_id, sizeof(block_request_.block.tag_id));
        usleep(10000);
        if(flagNewBlockRequestMessage_ && block_request_back_.new_block == block_request_.new_block) {
        // if(flagNewBlockRequestMessage_) {
            break;
        }

    }
    blockId = block_request_back_.block.tag_id;
    blockPose = block_request_back_.block.pose;
    printf("blockId = %d, blockPose = %f, %f\n", (int)blockId, blockPose.x, blockPose.y);
    if(blockId >= 0) {
        flagNext = true;
    }
    else {
        if( !poseStack_.empty() ) {
            blockPose = poseStack_.back();
            blockId = -1;
            poseStack_.pop_back();
            flagNext = true;
        }
    }
    printf("blockId after = %d, blockPose = %f, %f\n", (int)blockId, blockPose.x, blockPose.y);
    return flagNext;
}

bool Exploration::publishGrabState(int8_t nextState, int publishTime) {
    // TODOTHIS
    grab_status_.status = nextState;
    for (int i = 0; i < publishTime; i++) {
        lcmInstance_->publish(BOT_BLOCK_STATUS_CHANNEL, &grab_status_);
        usleep(10000);
    }
    return true;
}
bool Exploration::subscribeGrabState(int8_t& nextState) {
    // TODOTHIS
    if(flagGrabStatusUpdateMessage_) {
        nextState = nextStateFromArm_;
        flagGrabStatusUpdateMessage_ = false;
        return true;
    }
    return false;
}

bool Exploration::waitForArmMessage(int8_t& nextState, int64_t waitTime) {
    int64_t startTime = utime_now();
    while(utime_now() - startTime < waitTime) {
        bool flagJump = subscribeGrabState(nextState);
        if(flagJump) {
            return true;
        }
    }
    return false;
}

void Exploration::grabStateMachine(void)
{
    bool stateChanged = false;
    int8_t nextState = state_;

    // Save the path from the previous iteration to determine if a new path was created and needs to be published
    robot_path_t previousPath = currentPath_;
    bool flagPublishState = false;
    // Run the state machine until the state remains the same after an iteration of the loop
    do
    {
        switch(state_)
        {
            case exploration_status_t::GRAB_INITIALIZING:
                nextState = grabInitializing();
                flagPublishState = true;
                break;

            case exploration_status_t::GRAB_READY_FOR_NEXT:
                nextState = grabReadyForNext();
                flagPublishState = true;
                break;

            case exploration_status_t::GRAB_DRIVE_TO_POSE:
                nextState = grabDriveToPose();
                flagPublishState = true;
                break;

            case exploration_status_t::GRAB_PICKING_UP:
                nextState = grabPickingUp();
                break;

            case exploration_status_t::GRAB_RETURN_HOME:
                nextState = grabReturnHome(stateChanged);
                flagPublishState = true;
                break;

            case exploration_status_t::GRAB_PUTTING_DOWN:
                nextState = grabPuttingDown();
                break;

            case exploration_status_t::GRAB_COMPLETED:
                nextState = grabCompleted();
                flagPublishState = true;
                break;

            case exploration_status_t::GRAB_FAILED:
                nextState = grabFailed();
                break;
        }

        stateChanged = nextState != state_;
        state_ = nextState;

    } while(stateChanged);

    //if path confirmation was not received, resend path
    if(!pathReceived_ && flagPublishState)
    {
        std::cout << "the current path was not received by motion_controller, attempting to send again:\n";

        std::cout << "timestamp: " << currentPath_.utime << "\n";

        for(auto pose : currentPath_.path){
            std::cout << "(" << pose.x << "," << pose.y << "," << pose.theta << "); ";
        }std::cout << "\n";

        lcmInstance_->publish(CONTROLLER_PATH_CHANNEL, &currentPath_);
    }

    //if path changed, send current path
    if(previousPath.path != currentPath_.path)
    {

        std::cout << "INFO: Exploration: A new path was created on this iteration. Sending to Mbot:\n";

        std::cout << "path timestamp: " << currentPath_.utime << "\npath: ";

        for(auto pose : currentPath_.path){
            std::cout << "(" << pose.x << "," << pose.y << "," << pose.theta << "); ";
        }std::cout << "\n";

        lcmInstance_->publish(CONTROLLER_PATH_CHANNEL, &currentPath_);

        pathReceived_ = false;
        most_recent_path_time = currentPath_.utime;
    }

}

int8_t Exploration::grabInitializing(void)
{
std::cout << "Function: " << __FUNCTION__ << "  File: " << __FILE__ << "  Line: " << __LINE__ << "\n";
    /////////////////////////   Create the status message    //////////////////////////
    // Immediately transition to exploring once the first bit of data has arrived
    exploration_status_t status;
    status.utime = utime_now();
    status.team_number = teamNumber_;
    status.state = exploration_status_t::STATE_INITIALIZING;
    status.status = exploration_status_t::STATUS_COMPLETE;

    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);

    while(true) {
        if(haveMap_) {
            break;
        }
        usleep(10000);
    }
    readPoseStack();
    printf("stack size = %d\n", (int) poseStack_.size());

    // planner_.setMap(currentMap_);
    // currFrontier_ = planner_.newFrontier(currentPose_);
std::cout << "Function: " << __FUNCTION__ << "  File: " << __FILE__ << "  Line: " << __LINE__ << "\n";

    return exploration_status_t::GRAB_READY_FOR_NEXT;
}

int8_t Exploration::grabReadyForNext(void) {
std::cout << "Function: " << __FUNCTION__ << "  File: " << __FILE__ << "  Line: " << __LINE__ << "\n";
    int8_t nextState = state_;

    haveTargetBlockOrPose_ = nextBlockOrPose(blockId_, blockPose_);

    printf("haveTargetBlockOrPose_ = %d\n", (int)haveTargetBlockOrPose_);

    exploration_status_t status;
    status.utime = utime_now();
    status.team_number = teamNumber_;
    status.state = exploration_status_t::STATE_EXPLORING_MAP;
    if(haveTargetBlockOrPose_) {
        status.status = exploration_status_t::STATUS_IN_PROGRESS;
        nextState = exploration_status_t::GRAB_DRIVE_TO_POSE;
        publishGrabState(nextState);

        planner_.setMap(currentMap_);
        bool flagPushGoal = true;
        currentPath_ = planner_.directlyPlanPath(currentPose_, blockPose_, flagPushGoal);
        currentPath_.path.back().theta = blockPose_.theta;
        currentPath_.path_type = 2;
    }
    else {
        status.status = exploration_status_t::STATUS_COMPLETE;
        nextState = exploration_status_t::GRAB_RETURN_HOME;
        publishGrabState(nextState);
        flagGrabFinish_ = true;

        planner_.setMap(currentMap_);
        bool flagPushGoal = true;
        currentPath_ = planner_.directlyPlanPath(currentPose_, homePose_, flagPushGoal);
        currentPath_.path_type = 1;
    }
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);
std::cout << "Function: " << __FUNCTION__ << "  File: " << __FILE__ << "  Line: " << __LINE__ << "\n";

    return nextState;
}

int8_t Exploration::grabDriveToPose(void) {
std::cout << "Function: " << __FUNCTION__ << "  File: " << __FILE__ << "  Line: " << __LINE__ << "\n";

    int8_t nextState = state_;

    planner_.setMap(currentMap_);
    bool flagValidPath = planner_.checkPath(currentPath_);
    // if ( (!flagValidPath) || (utime_now() - planningTime_ > 5000000) ) {
    if ( !flagValidPath ) {
        bool flagPushGoal = true;
        printf("drive to pose replan %d\n", (int)utime_now());
        currentPath_ = planner_.directlyPlanPath(currentPose_, blockPose_, flagPushGoal);
        currentPath_.path.back().theta = blockPose_.theta;
        currentPath_.path_type = 2;
        planningTime_ = utime_now();
        printPath(currentPath_);
    }

    if (currentPath_.path.size() == 0) {
        if(!planner_.isValidPose(currentPose_) ) {
            currentPath_ = planner_.pathToFree(currentPose_);
            currentPath_.path_type = 0;
            printf("new path size = %d \n", (int) currentPath_.path.size());
        }
    }

    /////////////////////////   Create the status message    //////////////////////////
    exploration_status_t status;
    status.utime = utime_now();
    status.team_number = teamNumber_;
    status.state = exploration_status_t::STATE_EXPLORING_MAP;
    status.status = exploration_status_t::STATUS_IN_PROGRESS;

    if ( subscribeGrabState(nextState) ) {
        lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);
        return nextState;
    }

    float distToBlock = distance_between_points(Point<float>(blockPose_.x, blockPose_.y),
                                                Point<float>(currentPose_.x, currentPose_.y));
    printf("block = %f, %f, %d, %d, %f\n", blockPose_.x, blockPose_.y, (int)currentPath_.path.size(), (int)flagValidPath, distToBlock);
    // If we're within the threshold of home, then we're done.
    bool flagAngleOk = true;
    if(currentPath_.path_type == 2) {
        float angleDifference = angle_diff(blockPose_.theta, currentPose_.theta);
        flagAngleOk = (std::abs(angleDifference) <= 0.07);
    }
    if( (distToBlock <= kReachedPositionThreshold) && flagAngleOk)
    {
        int64_t waitTime = 3000000;
        bool flagJump = waitForArmMessage(nextState, waitTime);
        if(flagJump) {
            return nextState;
        }

        status.status = exploration_status_t::STATUS_COMPLETE;
        nextState = exploration_status_t::GRAB_READY_FOR_NEXT;
        publishGrabState(nextState);
    }
    // Otherwise, if there's a path, then keep following it
    else if(currentPath_.path.size() > 1)
    {
        status.status = exploration_status_t::STATUS_IN_PROGRESS;
    }
    // Else, there's no valid path to follow and we aren't home, so we have failed.
    else
    {
        status.status = exploration_status_t::STATUS_FAILED;
        nextState = exploration_status_t::GRAB_FAILED;
        publishGrabState(nextState);
    }

    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);

    ////////////////////////////   Determine the next state    ////////////////////////
    return nextState;
}

int8_t Exploration::grabReturnHome(bool stateChanged)
{
std::cout << "Function: " << __FUNCTION__ << "  File: " << __FILE__ << "  Line: " << __LINE__ << "\n";
    //////////////////////// TODO: Implement your method for returning to the home pose ///////////////////////////
    /*
    * NOTES:
    *   - At the end of each iteration, then (1) or (2) must hold, otherwise exploration is considered to have failed:
    *       (1) dist(currentPose_, targetPose_) < kReachedPositionThreshold  :  reached the home pose
    *       (2) currentPath_.path_length > 1  :  currently following a path to the home pose
    */

    int8_t nextState = state_;

    planner_.setMap(currentMap_);
    bool flagValidPath = planner_.checkPath(currentPath_);
        printf("time = %d\n", (int)utime_now());
    // if ( (!flagValidPath) || (utime_now() - planningTime_ > 5000000) ) {
    if(flagGrabFinish_) {
        grabTarPose_ = homePose_;
    }
    else{
        grabTarPose_ = putPose_;
    }
    if ( !flagValidPath || stateChanged ) {
        bool flagPushGoal = true;
        printf("home replan %d\n", (int)utime_now());
        currentPath_ = planner_.directlyPlanPath(currentPose_, grabTarPose_, flagPushGoal);
        currentPath_.path.back().theta = grabTarPose_.theta;
        currentPath_.path_type = 2;
        planningTime_ = utime_now();
        printPath(currentPath_);
    }

    if (currentPath_.path.size() == 0) {
        if(!planner_.isValidPose(currentPose_) ) {
            currentPath_ = planner_.pathToFree(currentPose_);
            currentPath_.path_type = 0;
            printf("new path size = %d \n", (int) currentPath_.path.size());
        }
    }

    /////////////////////////   Create the status message    //////////////////////////
    exploration_status_t status;
    status.utime = utime_now();
    status.team_number = teamNumber_;
    status.state = exploration_status_t::STATE_RETURNING_HOME;

    double distToHome = distance_between_points(Point<float>(grabTarPose_.x, grabTarPose_.y),
                                                Point<float>(currentPose_.x, currentPose_.y));
    printf("home = %f, %f, %d, %d, %f\n", grabTarPose_.x, grabTarPose_.y, (int)currentPath_.path.size(), (int)flagValidPath, distToHome);
    // If we're within the threshold of home, then we're done.
    bool flagAngleOk = true;
    if(currentPath_.path_type == 2) {
        float angleDifference = angle_diff(grabTarPose_.theta, currentPose_.theta);
        flagAngleOk = (std::abs(angleDifference) <= 0.07);
        printf("angleDifference = %f\n", angleDifference);
    }
    if( (distToHome <= kReachedPositionThreshold) && flagAngleOk)
    // if(distToHome <= kReachedPositionThreshold)
    {
        printf("at home!\n");
        if(flagGrabFinish_) {
            status.status = exploration_status_t::STATUS_COMPLETE;
            nextState = exploration_status_t::GRAB_COMPLETED;
            publishGrabState(nextState);
        }
        else {
            status.status = exploration_status_t::STATUS_IN_PROGRESS;
            nextState = exploration_status_t::GRAB_PUTTING_DOWN;
            publishGrabState(nextState, 1);
        }
    }
    // Otherwise, if there's a path, then keep following it
    else if(currentPath_.path.size() > 1)
    {
        status.status = exploration_status_t::STATUS_IN_PROGRESS;
    }
    // Else, there's no valid path to follow and we aren't home, so we have failed.
    else
    {
        status.status = exploration_status_t::STATUS_FAILED;
        nextState = exploration_status_t::GRAB_FAILED;
    }

    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);
std::cout << "Function: " << __FUNCTION__ << "  File: " << __FILE__ << "  Line: " << __LINE__ << "\n";

    ////////////////////////////   Determine the next state    ////////////////////////
    return nextState;
}

int8_t Exploration::grabPickingUp(void)
{
std::cout << "Function: " << __FUNCTION__ << "  File: " << __FILE__ << "  Line: " << __LINE__ << "\n";
    // Just wait
    int8_t nextState = state_;

    exploration_status_t status;
    status.utime = utime_now();
    status.team_number = teamNumber_;
    status.state = exploration_status_t::STATE_EXPLORING_MAP;
    status.status = exploration_status_t::STATUS_IN_PROGRESS;

    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);

    if ( subscribeGrabState(nextState) ) {
        return nextState;
    }

std::cout << "Function: " << __FUNCTION__ << "  File: " << __FILE__ << "  Line: " << __LINE__ << "\n";
    return nextState;
}

int8_t Exploration::grabPuttingDown(void)
{
std::cout << "Function: " << __FUNCTION__ << "  File: " << __FILE__ << "  Line: " << __LINE__ << "\n";
    // Just wait
    int8_t nextState = state_;

    exploration_status_t status;
    status.utime = utime_now();
    status.team_number = teamNumber_;
    status.state = exploration_status_t::STATE_EXPLORING_MAP;
    status.status = exploration_status_t::STATUS_IN_PROGRESS;

    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);

    if ( subscribeGrabState(nextState) ) {
        grabBlockNum_++;
        return nextState;
    }

std::cout << "Function: " << __FUNCTION__ << "  File: " << __FILE__ << "  Line: " << __LINE__ << "\n";
    return nextState;
}


int8_t Exploration::grabFailed(void)
{
    // Send the execute failed forever. There is no way to recover.
    exploration_status_t msg;
    msg.utime = utime_now();
    msg.team_number = teamNumber_;
    msg.state = exploration_status_t::STATE_FAILED_EXPLORATION;
    msg.status = exploration_status_t::STATUS_FAILED;

    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &msg);

    return exploration_status_t::GRAB_FAILED;
}

int8_t Exploration::grabCompleted(void)
{
    printf("completed hahaha!\n");
    // Stay in the completed state forever because exploration only explores a single map.
    exploration_status_t status;
    status.utime = utime_now();
    status.team_number = teamNumber_;
    status.state = exploration_status_t::STATE_COMPLETED_EXPLORATION;
    status.status = exploration_status_t::STATUS_COMPLETE;

    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);

    return exploration_status_t::GRAB_COMPLETED;
}

int8_t Exploration::executeInitializing(void)
{
    /////////////////////////   Create the status message    //////////////////////////
    // Immediately transition to exploring once the first bit of data has arrived
    exploration_status_t status;
    status.utime = utime_now();
    status.team_number = teamNumber_;
    status.state = exploration_status_t::STATE_INITIALIZING;
    status.status = exploration_status_t::STATUS_COMPLETE;

    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);

    planner_.setMap(currentMap_);
    currFrontier_ = planner_.newFrontier(currentPose_);

    return exploration_status_t::STATE_EXPLORING_MAP;
}

float getDistance(const pose_xyt_t& pose1, const pose_xyt_t& pose2) {
    float dx = pose1.x - pose2.x;
    float dy = pose1.y - pose2.y;
    return sqrtf(dx*dx + dy*dy);
}

int8_t Exploration::executeExploringMap(bool initialize)
{
    //////////////////////// TODO: Implement your method for exploring the map ///////////////////////////
    /*
    * NOTES:
    *   - At the end of each iteration, then (1) or (2) must hold, otherwise exploration is considered to have failed:
    *       (1) frontiers_.empty() == true      : all frontiers have been explored as determined by find_map_frontiers()
    *       (2) currentPath_.path_length > 1 : currently following a path to the next frontier
    *
    *   - Use the provided function find_map_frontiers() to find all frontiers in the map.
    *   - You will need to implement logic to select which frontier to explore.
    *   - You will need to implement logic to decide when to select a new frontier to explore. Take into consideration:
    *       -- The map is evolving as you drive, so what previously looked like a safe path might not be once you have
    *           explored more of the map.
    *       -- You will likely be able to see the frontier before actually reaching the end of the path leading to it.
    */

printf("currentPath_.size() = %d\t", (int)currentPath_.path.size());
printf("currFrontier_ = %d\t", currFrontier_);

    // Exploration
    planner_.setMap(currentMap_);
    bool flagInitialized = planner_.isInitialized(currFrontier_);

    float distToGoal = 0.0;
    if (currentPath_.path.size() >= 1) {
        distToGoal = getDistance(currentPose_, currentPath_.path.back());
    }

    int newFontier = planner_.newFrontier(currentPose_);

    if ( (flagInitialized && (distToGoal < 0.25 ) ) || (utime_now() - planningTime_ > 10000000) ) {
        currFrontier_ = newFontier;
        if (currFrontier_ >= 0) {
            pose_xyt_t goalPose = planner_.poseFromIndex(currFrontier_);
            bool flagPushGoal = true;
            currentPath_ = planner_.directlyPlanPath(currentPose_, goalPose, flagPushGoal);
            currentPath_.path_type = 0;
            planningTime_ = utime_now();
        }
    }
    else {
        bool flagValidPath = planner_.checkPath(currentPath_);
        printf("flagValidPath = %d\t", (int) flagValidPath);
        if (!flagValidPath) {
            currFrontier_ = newFontier;
            pose_xyt_t goalPose = planner_.poseFromIndex(currFrontier_);
            bool flagPushGoal = true;
            currentPath_ = planner_.directlyPlanPath(currentPose_, goalPose, flagPushGoal);
            currentPath_.path_type = 0;
            planningTime_ = utime_now();
        }
        printf("flagValidPath = %d\t", (int) flagValidPath);
    }
printf("currFrontier_ = %d, %d\t", currFrontier_, newFontier);
printf("currentPath_.size() = %d\n", (int)currentPath_.path.size());

    if (currentPath_.path.size() == 0) {
        if(!planner_.isValidPose(currentPose_) ) {
            currentPath_ = planner_.pathToFree(currentPose_);
            currentPath_.path_type = 0;
            printf("new path size = %d \n", (int) currentPath_.path.size());
        }
    }

    /////////////////////////////// End student code ///////////////////////////////

    /////////////////////////   Create the status message    //////////////////////////
    exploration_status_t status;
    status.utime = utime_now();
    status.team_number = teamNumber_;
    status.state = exploration_status_t::STATE_EXPLORING_MAP;

    // If no frontiers remain, then exploration is complete
    if(newFontier == -1)
    {
        planner_.saveMap("./saved_distance_map.map");

        status.status = exploration_status_t::STATUS_COMPLETE;
        bool flagPushGoal = true;
        currentPath_ = planner_.directlyPlanPath(currentPose_, homePose_, flagPushGoal);
        currentPath_.path_type = 0;
        planningTime_ = utime_now();
printf("currentPose_ = %f, %f\t homePose_ = %f, %f\n", currentPose_.x, currentPose_.y, homePose_.x, homePose_.y);
        printPath(currentPath_);
    }
    // Else if there's a path to follow, then we're still in the process of exploring
    else if(currentPath_.path.size() > 1)
    {
        status.status = exploration_status_t::STATUS_IN_PROGRESS;
    }
    // Otherwise, there are frontiers, but no valid path exists, so exploration has failed
    else
    {
        status.status = exploration_status_t::STATUS_FAILED;
    }

    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);

    ////////////////////////////   Determine the next state    ////////////////////////
    switch(status.status)
    {
        // Don't change states if we're still a work-in-progress
        case exploration_status_t::STATUS_IN_PROGRESS:
            return exploration_status_t::STATE_EXPLORING_MAP;

        // If exploration is completed, then head home
        case exploration_status_t::STATUS_COMPLETE:
            return exploration_status_t::STATE_RETURNING_HOME;

        // If something has gone wrong and we can't reach all frontiers, then fail the exploration.
        case exploration_status_t::STATUS_FAILED:
            return exploration_status_t::STATE_FAILED_EXPLORATION;

        default:
            std::cerr << "ERROR: Exploration::executeExploringMap: Set an invalid exploration status. Exploration failed!";
            return exploration_status_t::STATE_FAILED_EXPLORATION;
    }
}

void Exploration::printPath(const robot_path_t& thePath) {
    printf("path begin\n");
    for(size_t i = 0; i < thePath.path.size(); i++) {
        printf("(%d, %f, %f)\t", (int)i, thePath.path[i].x, thePath.path[i].y);
    }
    printf("\npath end\n");
}

int8_t Exploration::executeReturningHome(bool initialize)
{
    //////////////////////// TODO: Implement your method for returning to the home pose ///////////////////////////
    /*
    * NOTES:
    *   - At the end of each iteration, then (1) or (2) must hold, otherwise exploration is considered to have failed:
    *       (1) dist(currentPose_, targetPose_) < kReachedPositionThreshold  :  reached the home pose
    *       (2) currentPath_.path_length > 1  :  currently following a path to the home pose
    */

    planner_.setMap(currentMap_);
    bool flagValidPath = planner_.checkPath(currentPath_);
        printf("time = %d\n", (int)utime_now());
    // if ( (!flagValidPath) || (utime_now() - planningTime_ > 5000000) ) {
    if ( !flagValidPath ) {
        bool flagPushGoal = true;
        printf("home replan %d\n", (int)utime_now());
        currentPath_ = planner_.directlyPlanPath(currentPose_, homePose_, flagPushGoal);
        currentPath_.path_type = 0;
        planningTime_ = utime_now();
        printPath(currentPath_);
    }

    if (currentPath_.path.size() == 0) {
        if(!planner_.isValidPose(currentPose_) ) {
            currentPath_ = planner_.pathToFree(currentPose_);
            currentPath_.path_type = 0;
            printf("new path size = %d \n", (int) currentPath_.path.size());
        }
    }

    /////////////////////////////// End student code ///////////////////////////////

    /////////////////////////   Create the status message    //////////////////////////
    exploration_status_t status;
    status.utime = utime_now();
    status.team_number = teamNumber_;
    status.state = exploration_status_t::STATE_RETURNING_HOME;

    double distToHome = distance_between_points(Point<float>(homePose_.x, homePose_.y),
                                                Point<float>(currentPose_.x, currentPose_.y));
   printf("home = %f, %f, %d, %d, %f\n", homePose_.x, homePose_.y, (int)currentPath_.path.size(), (int)flagValidPath, distToHome);
    // If we're within the threshold of home, then we're done.
    if(distToHome <= kReachedPositionThreshold)
    {
printf("completed nice!\n");
        status.status = exploration_status_t::STATUS_COMPLETE;
    }
    // Otherwise, if there's a path, then keep following it
    else if(currentPath_.path.size() > 1)
    {
        status.status = exploration_status_t::STATUS_IN_PROGRESS;
    }
    // Else, there's no valid path to follow and we aren't home, so we have failed.
    else
    {
        status.status = exploration_status_t::STATUS_FAILED;
    }

    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);

    ////////////////////////////   Determine the next state    ////////////////////////
    if(status.status == exploration_status_t::STATUS_IN_PROGRESS)
    {
        return exploration_status_t::STATE_RETURNING_HOME;
    }
    else if(status.status == exploration_status_t::STATUS_COMPLETE) {
        return exploration_status_t::STATE_COMPLETED_EXPLORATION;
    }
    else // if(status.status == exploration_status_t::STATUS_FAILED)
    {
        return exploration_status_t::STATE_FAILED_EXPLORATION;
    }
}

int8_t Exploration::executeCompleted(bool initialize)
{
printf("completed hahaha!\n");
    // Stay in the completed state forever because exploration only explores a single map.
    exploration_status_t msg;
    msg.utime = utime_now();
    msg.team_number = teamNumber_;
    msg.state = exploration_status_t::STATE_COMPLETED_EXPLORATION;
    msg.status = exploration_status_t::STATUS_COMPLETE;

    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &msg);

    return exploration_status_t::STATE_COMPLETED_EXPLORATION;
}


int8_t Exploration::executeFailed(bool initialize)
{
    // Send the execute failed forever. There is no way to recover.
    exploration_status_t msg;
    msg.utime = utime_now();
    msg.team_number = teamNumber_;
    msg.state = exploration_status_t::STATE_FAILED_EXPLORATION;
    msg.status = exploration_status_t::STATUS_FAILED;

    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &msg);

    return exploration_status_t::STATE_FAILED_EXPLORATION;
}
