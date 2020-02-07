#ifndef SLAM_ACTION_MODEL_HPP
#define SLAM_ACTION_MODEL_HPP

#include <lcmtypes/pose_xyt_t.hpp>
#include <random>

struct particle_t;

/**
* ActionModel implements the sampling-based odometry action model for estimating the motion of the robot between
* time t and t'.
* 
* An action model is used to propagate a sample from the prior distribution, x, into
* the proposal distribution, x', based on the supplied motion estimate of the robot
* in the time interval [t, t'].
*
* To use the ActionModel, a two methods exist:
*
*   - bool updateAction(const pose_xyt_t& odometry);
*   - particle_t applyAction(const particle_t& sample);
*
* updateAction() provides the most recent odometry data so the action model can update the distributions from
* which it will sample.
*
* applyAction() applies the action to the provided sample and returns a new sample that can be part of the proposal 
* distribution for the particle filter.
*/
class ActionModel
{
public:
    
    /**
    * Constructor for ActionModel.
    */
    ActionModel(void);
    
    /**
    * updateAction sets up the motion model for the current update for the localization.
    * After initialization, calls to applyAction() will be made, so all distributions based on sensor data
    * should be created here.
    *
    * \param    odometry            Current odometry data from the robot
    * \return   The pose transform distribution representing the uncertainty of the robot's motion.
    */
    bool updateAction(const pose_xyt_t& currOdometry, const pose_xyt_t& prevOdometry, const pose_xyt_t& diffOdometry, bool flagNormal);
    
    /**
    * applyAction applies the motion to the provided sample and returns a new sample that
    * can be part of the proposal distribution for the particle filter.
    *
    * \param    sample          Sample to be moved
    * \return   New sample based on distribution from the motion model at the current update.
    */
    particle_t applyAction(const particle_t& sample);

    float inline translationFromOdometry(const pose_xyt_t& diffOdometry) const {
        return sqrtf( diffOdometry.y* diffOdometry.y + diffOdometry.x*diffOdometry.x);
    }

private:
    
    ////////// TODO: Add private member variables needed for you implementation ///////////////////
    float alpha1_; // alpha1: angle to angle
    float alpha2_; // alpha2: trans to angle
    float alpha3_; // alpha3: trans to trans
    float alpha4_; // alpha4: angle to trans
    // bool initialized_;
    float transEps_;
    float rotEps_;
    // pose_xyt_t prevOdometry_;
    // pose_xyt_t diffOdometry_;
    int64_t currUTime_;
    bool flagNormal_;
    std::default_random_engine generator_;
    std::normal_distribution<float> distributionRot1_;
    std::normal_distribution<float> distributionTrans_;
    std::normal_distribution<float> distributionRot2_;
    std::uniform_real_distribution<float> uniformRot_;
    std::uniform_real_distribution<float> uniformTrans_;
};

#endif // SLAM_ACTION_MODEL_HPP
