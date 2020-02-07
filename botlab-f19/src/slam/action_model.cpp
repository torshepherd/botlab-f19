#include <slam/action_model.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>

inline float myMinF(float a, float b) {
    if(a < b) {
        return a;
    }
    else {
        return b;
    }
}

inline float myMaxF(float a, float b) {
    if(a > b) {
        return a;
    }
    else {
        return b;
    }
}

ActionModel::ActionModel(void)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
    // alpha1_ = 0.2f; // alpha1: angle to angle
    // alpha2_ = 0.2f; // alpha2: trans to angle
    // alpha3_ = 0.2f; // alpha3: trans to trans
    // alpha4_ = 0.05f; // alpha4: angle to trans
    alpha1_ = 0.3f; // alpha1: angle to angle
    alpha2_ = 0.3f; // alpha2: trans to angle
    alpha3_ = 0.2f; // alpha3: trans to trans
    alpha4_ = 0.05f; // alpha4: angle to trans

    // alpha1_ = 0.4f; // alpha1: angle to angle
    // alpha2_ = 0.4f; // alpha2: trans to angle
    // alpha3_ = 0.3f; // alpha3: trans to trans
    // alpha4_ = 0.05f; // alpha4: angle to trans

    // initialized_ = false;

    transEps_ = 0.001f;
    rotEps_ = 0.001f;

    flagNormal_ = true;

    // prevOdometry_.utime = 0;
    // prevOdometry_.x = 0.0f;
    // prevOdometry_.y = 0.0f;
    // prevOdometry_.theta = 0.0f;

    // diffOdometry_.utime = 0;
    // diffOdometry_.x = 0.0f;
    // diffOdometry_.y = 0.0f;
    // diffOdometry_.theta = 0.0f;
    
    // std::default_random_engine generator_;
}

bool ActionModel::updateAction(const pose_xyt_t& currOdometry, const pose_xyt_t& prevOdometry, const pose_xyt_t& diffOdometry, bool flagNormal)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////

    flagNormal_ = flagNormal;
    currUTime_ = currOdometry.utime;

    float maxLinearVel = 0.07;
    float maxAngularVel = 0.25;
    float maxLinearMove = maxLinearVel;
    float maxAngularMove = maxAngularVel;

    float trans = translationFromOdometry(diffOdometry);
    float rot1_ = 0.0f;
    float rot2_ = 0.0f;

    if(flagNormal_) {
        if ( trans < transEps_) {
            if (diffOdometry.theta < rotEps_) {
                // Not translate and not rotate --> not move
                return false;
            }
            rot1_ = 0.0f;
            rot2_ = diffOdometry.theta;
        }
        else {
            rot1_ = atan2f(diffOdometry.y, diffOdometry.x) - prevOdometry.theta;
            rot1_ = wrap_to_pi(rot1_);
            rot2_ = wrap_to_pi(currOdometry.theta - prevOdometry.theta - rot1_);
        }

        float rotLimit = fabsf(diffOdometry.theta) * 2;
        float rot1Limited = myMinF(fabsf(rot1_), rotLimit);
        float rot2Limited = myMinF(fabsf(rot2_), rotLimit);


        float varRot1 = wrap_to_pi(alpha1_ * rot1Limited + alpha2_ * trans);
        // float varTrans = alpha3_ * trans + alpha4_ * fabsf(rot1_ + rot2_);
        float varTrans = alpha3_ * trans + alpha4_ * fabsf(diffOdometry.theta);
        float varRot2 = wrap_to_pi(alpha1_ * rot2Limited + alpha2_ * trans);

        distributionRot1_ = std::normal_distribution<float>(rot1_, varRot1);
        distributionTrans_ = std::normal_distribution<float>(trans, varTrans);
        distributionRot2_ = std::normal_distribution<float>(rot2_, varRot2);        
    }
    else {
        uniformTrans_ = std::uniform_real_distribution<float>(-maxLinearMove, maxLinearMove);
        uniformRot_ = std::uniform_real_distribution<float>(-maxAngularMove, maxAngularMove);
    }

    // prevOdometry.utime = currOdometry.utime;        
    // prevOdometry.x = currOdometry.x;        
    // prevOdometry.y = currOdometry.y;        
    // prevOdometry.theta = currOdometry.theta;        

    return true;
}


particle_t ActionModel::applyAction(const particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.

    particle_t newSample;
    newSample.weight = sample.weight;
    newSample.parent_pose = sample.pose;
    newSample.pose.utime = currUTime_;

    if(flagNormal_) {
        float rot1Sample = distributionRot1_(generator_);
        float rot2Sample = distributionRot2_(generator_);
        float transSample = distributionTrans_(generator_);

        newSample.pose.x = sample.pose.x + transSample * cosf(sample.pose.theta + rot1Sample);
        newSample.pose.y = sample.pose.y + transSample * sinf(sample.pose.theta + rot1Sample);
        newSample.pose.theta = wrap_to_pi( sample.pose.theta + rot1Sample + rot2Sample );
    }
    else {
        float dX = uniformTrans_(generator_);
        float dY = uniformTrans_(generator_);
        float dTheta = uniformRot_(generator_);

        newSample.pose.x = sample.pose.x + dX;
        newSample.pose.y = sample.pose.y + dY;
        newSample.pose.theta = sample.pose.theta + dTheta;
    }
    // printf("nice = (%f, %f, %f)\n", newSample.pose.x, newSample.pose.y, newSample.pose.theta);

    return newSample;
}
