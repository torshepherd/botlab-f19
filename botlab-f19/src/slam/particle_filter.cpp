#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <cassert>
#include <algorithm>
#include <functional>
// #include <stdlib.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <common/angle_functions.hpp>

// #include <planning/obstacle_distance_grid.hpp>

ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles),
flagPoseStrategy_(POSESTRATEGY_HIGHEST_N),
ratioParticles_(0.98),
bestWeight_(-1.0),
bestWeightThres_(2.4),
odometryVelThres_(0.75),
prevFlagNormalDistribution_(true),
initialized_(false),
flagGlobalLocalization_(false),
kNumParticlesOriginal_(numParticles)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
    for (particle_t& a_particle : posterior_ ) {
        a_particle.weight = 1.0 / (static_cast<double>(kNumParticles_));
        a_particle.pose.x = 0.0f;
        a_particle.pose.y = 0.0f;
        a_particle.pose.theta = 0.0f;
        a_particle.pose.utime = 0;
    }
}


void ParticleFilter::initializeFilterAtPose(const pose_xyt_t& pose)
{
    printf("Initial pose: (%f, %f, %f)\n", pose.x, pose.y, pose.theta);
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    for (particle_t& a_particle : posterior_ ) {
        a_particle.weight = 1.0 / (static_cast<double>(kNumParticles_));
        a_particle.pose = pose;
        a_particle.parent_pose = pose;
    }
}

float ParticleFilter::getDiffOdometry(const pose_xyt_t& currOdometry, const pose_xyt_t& prevOdometry, pose_xyt_t& diffOdometry) const {
    diffOdometry.utime = currOdometry.utime;
    diffOdometry.x = currOdometry.x - prevOdometry.x;
    diffOdometry.y = currOdometry.y - prevOdometry.y;
    diffOdometry.theta = wrap_to_pi(currOdometry.theta - prevOdometry.theta);

    float diffTime = static_cast<float>(currOdometry.utime - prevOdometry.utime) / 1e6;
    float odometryVel = sqrtf( diffOdometry.y* diffOdometry.y + diffOdometry.x*diffOdometry.x) / diffTime;
    return odometryVel;
}

pose_xyt_t ParticleFilter::estimateSlamOdometry(const pose_xyt_t& odometry, pose_xyt_t& diffOdometry, const lidar_t& laser, const OccupancyGrid& map) {
    const bool flagNormalDistributionFalse = false;
    actionModel_.updateAction(odometry, prevOdometry_, diffOdometry, flagNormalDistributionFalse);
    std::vector<particle_t> posteriorOdom;
    auto priorOdom = sameDistribution();
    auto proposalOdom = computeProposalDistribution(priorOdom);
    computeNormalizedPosterior(proposalOdom, laser, map, posteriorOdom);
    pose_xyt_t slamOdometry = estimatePosteriorPose(posteriorOdom, flagNormalDistributionFalse);
    getDiffOdometry(slamOdometry, posteriorPose_, diffOdometry);
    return slamOdometry;
}


pose_xyt_t ParticleFilter::updateFilter(const pose_xyt_t&      odometry,
                                        const lidar_t& laser,
                                        const OccupancyGrid&   map)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.

    //abcd

    if (initialized_) {
        const bool flagNormalDistributionTrue = true;

        pose_xyt_t slamOdometry;
        pose_xyt_t diffOdometry;
        float odometryVel = getDiffOdometry(odometry, prevOdometry_, diffOdometry);
        bool flagSlamOdometry = false;

        if( (odometryVel > odometryVelThres_) && (!flagGlobalLocalization_) ) {
            // There must be wheel slip happening
            printf("Bad odometry! odometryVel = %f\t", odometryVel);
            slamOdometry = estimateSlamOdometry(odometry, diffOdometry, laser, map);
            flagSlamOdometry = true;
        }

        bool hasRobotMoved = true;
        if(flagSlamOdometry) {
            hasRobotMoved = actionModel_.updateAction(slamOdometry, posteriorPose_, diffOdometry, flagNormalDistributionTrue);
        }
        else {
            hasRobotMoved = actionModel_.updateAction(odometry, prevOdometry_, diffOdometry, flagNormalDistributionTrue);
        }
        
        if (hasRobotMoved) {
            // Remove these deep copies later, e.g. "posterior_ = posterior"
            
            std::vector<particle_t> posterior;
            auto prior = resamplePosteriorDistribution();
            auto proposal = computeProposalDistribution(prior);
            bool flagGoodPosterior = computeNormalizedPosterior(proposal, laser, map, posterior);

            if ( (!flagGoodPosterior) && (!flagGlobalLocalization_) ) {
                // Bad weight, use SLAM odometry
                if(!flagSlamOdometry) {
                    printf("Localize again! w1 = %f\t", bestWeight_);
                    slamOdometry = estimateSlamOdometry(odometry, diffOdometry, laser, map);
                    flagSlamOdometry = true;
                    actionModel_.updateAction(slamOdometry, posteriorPose_, diffOdometry, flagNormalDistributionTrue);
                    prior = resamplePosteriorDistribution();
                    proposal = computeProposalDistribution(prior);
                    computeNormalizedPosterior(proposal, laser, map, posterior);
                    printf("w2 = %f, flagGoodPosterior = %d\n", bestWeight_, (int)flagGoodPosterior );
                }
            }

            posterior_ = posterior;
            posteriorPose_ = estimatePosteriorPose(posterior_, flagNormalDistributionTrue);

            // if(false) {
            //     auto prior = resamplePosteriorDistribution();
            //     auto proposal = computeProposalDistribution(prior);
            //     computeNormalizedPosterior(proposal, laser, map);
            //     posteriorPose_ = estimatePosteriorPose(posterior_);
            // }

            printf("pose = %f, \t %d, %f, %f, %f, \t %d, %f, %f, %f\n", bestWeight_,
                        (int) posteriorPose_.utime, posteriorPose_.x, posteriorPose_.y, posteriorPose_.theta,
                        (int) odometry.utime, odometry.x, odometry.y, odometry.theta);
            // if(true) {
            //     std::ofstream particleFile;
            //     particleFile.open ("./pose_log.csv");
            //     particleFile << std::fixed << std::setprecision(8) << odometry.utime << ", ";
            //     particleFile << std::fixed << std::setprecision(8) << odometry.x << ", ";
            //     particleFile << std::fixed << std::setprecision(8) << odometry.y << ", ";
            //     particleFile << std::fixed << std::setprecision(8) << odometry.theta << "\n";

            //     particleFile << std::fixed << std::setprecision(8) << posteriorPose_.utime << ", ";
            //     particleFile << std::fixed << std::setprecision(8) << posteriorPose_.x << ", ";
            //     particleFile << std::fixed << std::setprecision(8) << posteriorPose_.y << ", ";
            //     particleFile << std::fixed << std::setprecision(8) << posteriorPose_.theta << "\n";

            //     particleFile << std::fixed << std::setprecision(8) << bestWeight_ << "\n";

            //     particleFile.close();
            // }
        }
    }
    prevOdometry_.utime = odometry.utime;        
    prevOdometry_.x = odometry.x;        
    prevOdometry_.y = odometry.y;        
    prevOdometry_.theta = odometry.theta;
    initialized_ = true;
    
    posteriorPose_.utime = odometry.utime;
    
    return posteriorPose_;
}


pose_xyt_t ParticleFilter::poseEstimate(void) const
{
    return posteriorPose_;
}


particles_t ParticleFilter::particles(void) const
{
    particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    particles.utime = posteriorPose_.utime;
    // printf("particles.utime = %d\n", (int)particles.utime);
    return particles;
}

inline float myRand() {
	return ((float) rand() / (((float) RAND_MAX) + 1) ); 
}

// void ParticleFilter::uniformDistributeParticles(const OccupancyGrid&   map) {

//     flagGlobalLocalization_ = true;

//     ObstacleDistanceGrid distanceMap;
//     distanceMap.setDistances(map);

//     float xMaxMeters;
//     float xMinMeters;
//     float yMaxMeters;
//     float yMinMeters;
//     map.copyToSlam(xMaxMeters, xMinMeters, yMaxMeters, yMinMeters);

//     std::uniform_real_distribution<float> xDistribution(xMinMeters, xMaxMeters);
//     std::uniform_real_distribution<float> yDistribution(yMinMeters, yMaxMeters);
//     std::uniform_real_distribution<float> thetaDistribution(-MY_PI, MY_PI);

//     const int kNumParticlesGlobal = 30000;
//     kNumParticles_ = kNumParticlesGlobal;
//     posterior_.clear();
//     posterior_.reserve(kNumParticles_);
//     double weightStep = 1.0 / static_cast<double>(kNumParticles_);
//     particle_t a_particle;
//     a_particle.parent_pose = posteriorPose_;
//     a_particle.pose = posteriorPose_;
//     a_particle.weight = weightStep;

//     for(int j = 0; j < kNumParticles_; j++) {
//         float xRandom, yRandom, thetaRandom;
//         while (true) {
//             xRandom = xDistribution(generator_);
//             yRandom = yDistribution(generator_);
//             thetaRandom = thetaDistribution(generator_);
//             int xInt, yInt;
//             distanceMap.toInt(xRandom, yRandom, xInt, yInt);
//             if(!distanceMap.isCellInGrid(xInt, yInt) ) {
//                 continue;
//             }
//             int index = distanceMap.cellIndex(xInt, yInt);
//             if(distanceMap.isObstacle(index, 0.15, true)) {
//                 continue;
//             }
//             break;
//         }
//         a_particle.parent_pose.x = xRandom;
//         a_particle.parent_pose.y = yRandom;
//         a_particle.parent_pose.theta = thetaRandom;
//         a_particle.pose.x = xRandom;
//         a_particle.pose.y = yRandom;
//         a_particle.pose.theta = thetaRandom;
//         posterior_.push_back(a_particle);
//     }

// }

std::vector<particle_t> ParticleFilter::resamplePosteriorDistribution(void)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    if (flagGlobalLocalization_) {
        kNumParticles_ -= 1000;
        if(kNumParticles_ < kNumParticlesOriginal_) {
            kNumParticles_ = kNumParticlesOriginal_;
        }
    }

    std::vector<particle_t> prior;
    prior.reserve(kNumParticles_);

    double weightStep = 1.0 / static_cast<double>(kNumParticles_);
    double cumSumWeight = posterior_[0].weight;
    double randStart = myRand() * weightStep;
    int i = 0;
    for (int j = 0; j < kNumParticles_; j++) {
        double currWeight = randStart + (static_cast<double>(j) * weightStep);
        while (currWeight > cumSumWeight) {
            i++;
            cumSumWeight += posterior_[i].weight;
        }
        prior.push_back(posterior_[i]);
        prior.back().weight = weightStep;
    }

    return prior;
}

std::vector<particle_t> ParticleFilter::sameDistribution(void)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    
    double weightStep = 1.0 / static_cast<double>(kNumParticles_);
    particle_t a_particle;
    a_particle.parent_pose = posteriorPose_;
    a_particle.pose = posteriorPose_;
    a_particle.weight = weightStep;

    std::vector<particle_t> prior;
    prior.reserve(kNumParticles_);
    for (int j = 0; j < kNumParticles_; j++) {
        prior.push_back(a_particle);
        prior.back().weight = weightStep;
    }

    return prior;
}


std::vector<particle_t> ParticleFilter::computeProposalDistribution(const std::vector<particle_t>& prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    std::vector<particle_t> proposal;
    proposal.reserve(prior.size());
    for(const particle_t& a_particle : prior) {
        // Action model
        // proposal.push_back( a_particle );
        proposal.push_back( actionModel_.applyAction(a_particle) );
    }
    return proposal;
}


bool ParticleFilter::computeNormalizedPosterior(const std::vector<particle_t>& proposal,
                                                                   const lidar_t& laser,
                                                                   const OccupancyGrid&   map,
                                                std::vector<particle_t>& posterior)
{
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the 
    ///////////       particles in the proposal distribution
    // std::vector<particle_t> posterior;
    posterior = proposal;

    bestWeight_ = -1.0;
    for(size_t i = 0; i < proposal.size(); i++) {
        posterior[i].weight = sensorModel_.likelihood(proposal[i], laser, map);

        // For debugging
        if (posterior[i].weight > bestWeight_) {
            bestWeight_ = posterior[i].weight;
        }
    }

    double sumWeight = 0.0;
    for(const particle_t& a_particle : posterior) {
        sumWeight += a_particle.weight;
    }
    for(particle_t& a_particle : posterior) {
        a_particle.weight /= sumWeight;
    }

    if(bestWeight_ < bestWeightThres_) {
        return false;
    }

    return true;
}

// std::vector<particle_t> ParticleFilter::computeNormalizedPosterior(const std::vector<particle_t>& proposal,
//                                                                    const lidar_t& laser,
//                                                                    const OccupancyGrid&   map)
// {
//     /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the 
//     ///////////       particles in the proposal distribution
//     std::vector<particle_t> posterior;
//     // This is a wrong implementation: no sensor model
//     posterior = proposal;

//     for(size_t i = 0; i < proposal.size(); i++) {
//         posterior[i].weight = sensorModel_.likelihood(proposal[i], laser, map);
//     }

//     float sumWeight = 0.0;
//     for(const particle_t& a_particle : posterior) {
//         sumWeight += a_particle.weight;
//     }
//     for(particle_t& a_particle : posterior) {
//         a_particle.weight /= sumWeight;
//     }
//     return posterior;
// }

pose_xyt_t ParticleFilter::estimatePosteriorPose(const std::vector<particle_t>& posterior, bool flagNormalDistribution)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    pose_xyt_t pose;
    if ( (flagPoseStrategy_ == POSESTRATEGY_HIGHEST_N)) {
        if (flagNormalDistribution) {
            pose = weightedAverage(posterior, ratioParticles_);
        }
        else {
            pose = weightedAverage(posterior, 0.995);
        }
        
        // printf("haha\n");
    }
    else {
        size_t bestId = 0;
        double bestWeight = 0.0;
        for(size_t i = 0; i < posterior.size(); i++) {
            if(posterior[i].weight > bestWeight) {
                bestWeight = posterior[i].weight;
                bestId = i;
            }
        }
        pose = posterior[bestId].pose;
    }

    return pose;
}

pose_xyt_t ParticleFilter::weightedAverage(const std::vector<particle_t>& posterior, float ratio) const {
    double ratioWeight = nthWeight(posterior, ratio);
    float sumX = 0.0f;
    float sumY = 0.0f;
    float sumCos = 0.0f;
    float sumSin = 0.0f;
    int particleUsed = 0;
    for(const particle_t& a_particle : posterior) {
        if(a_particle.weight >= ratioWeight) {
            sumX += a_particle.pose.x;
            sumY += a_particle.pose.y;
            sumCos += cosf( a_particle.pose.theta );
            sumSin += sinf( a_particle.pose.theta );
            particleUsed++;
        }
    }
    pose_xyt_t pose;
    float particleUsedFloat = static_cast<float>(particleUsed);
    if(particleUsed == 0) {
        printf("particleUsed = %d, %f\t", particleUsed, ratioWeight);

        // std::ofstream weightFile;
        // weightFile.open ("./weights_log.csv");
        // for(const particle_t& a_particle : posterior) {
        //     std::cout << std::fixed << std::setprecision(8) << a_particle.weight << ", ";
        // }
        // weightFile << "\n";
        // weightFile.close();

    }
    pose.utime = posterior[0].pose.utime;
    pose.x = sumX / particleUsedFloat;
    pose.y = sumY / particleUsedFloat;
    pose.theta = atan2f(sumSin, sumCos);

    return pose;
}

double ParticleFilter::nthWeight(const std::vector<particle_t>& posterior, float ratio) const {
    std::vector<double> weights;
    weights.reserve(posterior.size());
    for(const particle_t& a_particle : posterior) {
        weights.push_back(a_particle.weight);
    }
    int ratioIndex = ratio * weights.size();
    // printf("ratioIndex = %d, %d\n", ratioIndex, (int) weights.size());
    
    std::vector<double>::iterator ratioIterator = weights.begin();
    std::advance(ratioIterator, ratioIndex); 
    std::nth_element(weights.begin(), ratioIterator, weights.end());
    return weights[ratioIndex];
}
