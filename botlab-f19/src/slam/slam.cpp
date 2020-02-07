#include <slam/slam.hpp>
#include <slam/slam_channels.h>
#include <mbot/mbot_channels.h>
#include <optitrack/optitrack_channels.h>
#include <unistd.h>
#include <cassert>
#include <chrono>
#include <stdio.h>

#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include <time.h>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

int getch(void)
{
    struct termios oldattr, newattr;
    int ch;
    tcgetattr( STDIN_FILENO, &oldattr );
    newattr = oldattr;
    newattr.c_lflag &= ~( ICANON | ECHO );
    tcsetattr( STDIN_FILENO, TCSANOW, &newattr );
    ch = getchar();
    tcsetattr( STDIN_FILENO, TCSANOW, &oldattr );
    return ch;
}

int kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;
 
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
 
  ch = getchar();
 
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);
 
  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }
 
  return 0;
}

void OccupancyGridSLAM::checkInput() {
    int ch = 0;
    int flag_getchar = 0;
    if ( kbhit())
    {
        ch = getch();
        flag_getchar = 1;
    }
    if (flag_getchar == 1) {
        switch (ch)
        {
        case '1':
            printf("\nWe get your input!!!\n");
            break;
        // case '2':
            uniformDistributeParticles();
        //     mode_ = localization_only;
        //     printf("\nGlobal localization start!!!\n");
        //     break;
        case 'l':
            mode_ = localization_only;
            printf("\nLocalization only!!!\n");
            break;
        case 'f':
            mode_ = full_slam;
            printf("\nFull SLAM!!!\n");
            break;
        case 's':
            map_.saveToFile("./this_map.map");
            printf("\nMap saved!!!\n");
            break;
        default:
            break;
        }
    }
}

// void OccupancyGridSLAM::uniformDistributeParticles(void) {
//     filter_.uniformDistributeParticles(map_);
// }

OccupancyGridSLAM::OccupancyGridSLAM(int         numParticles,
                                     int8_t      hitOddsIncrease,
                                     int8_t      missOddsDecrease,
                                     lcm::LCM&   lcmComm,
                                     bool waitForOptitrack,
                                     bool mappingOnlyMode,
                                     const std::string localizationOnlyMap)
: mode_(full_slam)  // default is running full SLAM, unless user specifies otherwise on the command line
, haveInitializedPoses_(false)
, waitingForOptitrack_(waitForOptitrack)
, haveMap_(false)
, numIgnoredScans_(0)
, filter_(numParticles)
, map_(10.0f, 10.0f, 0.05f) //30,30,0.1  // create a 10m x 10m grid with 0.05m cells
, mapper_(5.0f, hitOddsIncrease, missOddsDecrease)
, lcm_(lcmComm)
, mapUpdateCount_(0)
{
    srand(time(NULL));

    // Confirm that the mode is valid -- mapping-only and localization-only are not specified
    assert(!(mappingOnlyMode && localizationOnlyMap.length() > 0));
    
    // Determine which mode to run based on the inputs
    if(mappingOnlyMode)
    {
        mode_ = mapping_only;
    }
    else if(localizationOnlyMap.length() > 0)
    {
        haveMap_ = map_.loadFromFile(localizationOnlyMap);
        assert(haveMap_);   // if there's no map, then the localization can't run!
        
        mode_ = localization_only;
    }
    
    currentOdometry_.utime = 0;
    currentScan_.utime = 0;
    
    // Laser and odometry data are always required
    lcm_.subscribe(LIDAR_CHANNEL, &OccupancyGridSLAM::handleLaser, this);
    lcm_.subscribe(ODOMETRY_CHANNEL, &OccupancyGridSLAM::handleOdometry, this);
    lcm_.subscribe(TRUE_POSE_CHANNEL, &OccupancyGridSLAM::handleOptitrack, this);
    
    // If we are only building the occupancy grid using ground-truth poses, then subscribe to the ground-truth poses.
    if(mode_ == mapping_only)
    {
        lcm_.subscribe(SLAM_POSE_CHANNEL, &OccupancyGridSLAM::handlePose, this);
    }
    
    // Zero-out all the poses to start. Either the robot will start at (0,0,0) or at the first pose received from the
    // Optitrack system.
    initialPose_.x = initialPose_.y = initialPose_.theta = 0.0f;
    previousPose_.x = previousPose_.y = previousPose_.theta = 0.0f;
    currentPose_.x  = currentPose_.y  = currentPose_.theta  = 0.0f;
}


void OccupancyGridSLAM::runSLAM(void)
{
    while(true)
    {
        // If new data has arrived
        if(isReadyToUpdate())
        {
            // Then run an iteration of our SLAM algorithm
            runSLAMIteration();
        }
        // Otherwise, do a quick spin while waiting for data rather than using more complicated condition variable.
        else
        {
            usleep(1000);
        }
    }
}


// Handlers for LCM messages
void OccupancyGridSLAM::handleLaser(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const lidar_t* scan)
{
    const int kNumIgnoredForMessage = 10;   // number of scans to ignore before printing a message about odometry
//std::cout << "laser!\n";    
    std::lock_guard<std::mutex> autoLock(dataMutex_);
    // Ignore scans until odometry data arrives -- need odometry before a scan to safely built the map
    bool haveOdom = (mode_ != mapping_only) // For full SLAM, odometry data is needed.
        && !odometryPoses_.empty() 
        && (odometryPoses_.front().utime <= scan->times.front());
    bool havePose = (mode_ == mapping_only) // For mapping-only, ground-truth poses are needed
        && !groundTruthPoses_.empty() 
        && (groundTruthPoses_.front().utime <= scan->times.front());
    
    // If there's appropriate odometry or pose data for this scan, then add it to the queue.
    if(haveOdom || havePose)
    {
        incomingScans_.push_back(*scan);
        
        // If we showed the laser error message, then provide another message indicating that laser scans are now
        // being saved
        if(numIgnoredScans_ >= kNumIgnoredForMessage)
        {
            std::cout << "INFO: OccupancyGridSLAM: Received odometry or pose data, so laser scans are now being \
                saved.\n";
            numIgnoredScans_ = 0;
        }
    }
    // Otherwise ignore it
    else
    {
        ++numIgnoredScans_;
    }
    
    if(numIgnoredScans_ == kNumIgnoredForMessage)
    {
        std::cout << "INFO: OccupancyGridSLAM: Ignoring scan because no odometry data is available. \
            Please start the odometry module or use a log with ground-truth poses.\n";
    }
}


void OccupancyGridSLAM::handleOdometry(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const odometry_t* odometry)
{
    std::lock_guard<std::mutex> autoLock(dataMutex_);

    pose_xyt_t odomPose;
    odomPose.utime = odometry->utime;
    odomPose.x = odometry->x;
    odomPose.y = odometry->y;
    odomPose.theta = odometry->theta;
    odometryPoses_.addPose(odomPose);
}


void OccupancyGridSLAM::handlePose(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const pose_xyt_t* pose)
{
    std::lock_guard<std::mutex> autoLock(dataMutex_);
    groundTruthPoses_.addPose(*pose);
}


void OccupancyGridSLAM::handleOptitrack(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const pose_xyt_t* pose)
{
    std::lock_guard<std::mutex> autoLock(dataMutex_);

    if(waitingForOptitrack_)
    {
        initialPose_ = *pose;
        waitingForOptitrack_ = false;
    }
}


bool OccupancyGridSLAM::isReadyToUpdate(void)
{
    std::lock_guard<std::mutex> autoLock(dataMutex_);
    
    bool haveData = false;
	    
    // If there's at least one scan to process, then check if odometry/pose information is available
    if(!incomingScans_.empty())
    {
        // Find if there's a scan that there is odometry data for
        const lidar_t& nextScan = incomingScans_.front();
        
        // Ensure that there's a pose that exists at or after the final laser measurement to be sure that valid
        // interpolation of robot motion during the scan can be performed.
        
        // Only care if there's odometry data if we aren't in mapping-only mode
        bool haveNewOdom = (mode_ != mapping_only) && (odometryPoses_.containsPoseAtTime(nextScan.times.back()));
        // Otherwise, only see if a new pose has arrived
        bool haveNewPose = (mode_ == mapping_only) && (groundTruthPoses_.containsPoseAtTime(nextScan.times.back()));        

        haveData = haveNewOdom || haveNewPose;
    }

    // If all SLAM data and optitrack data has arrived, then we're ready to go.
    return haveData && !waitingForOptitrack_;
}


void OccupancyGridSLAM::runSLAMIteration(void)
{
    copyDataForSLAMUpdate();
    initializePosesIfNeeded();
    
    // Sanity check the laser data to see if rplidar_driver has lost sync
    if(currentScan_.num_ranges > 100)//250)
    {
        updateLocalization();
        updateMap();
    }
    else 
    {
        std::cerr << "ERROR: OccupancyGridSLAM: Detected invalid laser scan with " << currentScan_.num_ranges 
            << " ranges.\n";
    }
    if(mapUpdateCount_ % 10 == 0) {
        checkInput();
    }
}


void OccupancyGridSLAM::copyDataForSLAMUpdate(void)
{
    std::lock_guard<std::mutex> autoLock(dataMutex_);
    
    // Copy the data needed for the new SLAM update
    currentScan_ = incomingScans_.front();
    incomingScans_.pop_front();
    
    if(mode_ == mapping_only)
    {
        // No localization is performed during mapping-only mode, so the previous pose needs to be correctly adjusted
        // here.
        previousPose_ = currentPose_;
        currentPose_  = groundTruthPoses_.poseAt(currentScan_.times.back());
    }
    else
    {
        currentOdometry_ = odometryPoses_.poseAt(currentScan_.times.back());
        // FILE* odometry_file = fopen("odometry_log.csv", "a");
        // fprintf(odometry_file, "%d, %f, %f, %f\n", (int) currentOdometry_.utime, currentOdometry_.x, currentOdometry_.y, currentOdometry_.theta);
        // fclose(odometry_file);
    }
}


void OccupancyGridSLAM::initializePosesIfNeeded(void)
{
    // The initial poses need to be set with the timestamps associated with the first last scan to ensure that proper
    // interpolation of the laser scan happen in MovingLaserScan. This initialization requires the timestamp of the
    // first laser scan, so it can't be performed in the constructor.
    if(!haveInitializedPoses_)
    {
        previousPose_ = initialPose_;
        previousPose_.utime = currentScan_.times.front();
        
        currentPose_ = previousPose_;
        currentPose_.utime  = currentScan_.times.back();
        haveInitializedPoses_ = true;
        
        filter_.initializeFilterAtPose(previousPose_);
    }
    
    assert(haveInitializedPoses_);
}


void OccupancyGridSLAM::updateLocalization(void)
{
    if(haveMap_ && (mode_ != mapping_only))
    {
        previousPose_ = currentPose_;
        currentPose_  = filter_.updateFilter(currentOdometry_, currentScan_, map_);//, v_, omega_, utime_); //remove last 3 args for odo
        
        auto particles = filter_.particles();

        lcm_.publish(SLAM_POSE_CHANNEL, &currentPose_);
        lcm_.publish(SLAM_PARTICLES_CHANNEL, &particles);
        // if(mapUpdateCount_ % 20 == 0) {
        //     std::ofstream particleFile;
        //     particleFile.open ("./log/particle_file"+ std::to_string(mapUpdateCount_) + ".csv");
        //     for(const particle_t& a_particle : particles.particles) {
        //         particleFile << std::fixed << std::setprecision(8) << a_particle.pose.utime << ", ";
        //         particleFile << std::fixed << std::setprecision(8) << a_particle.pose.x << ", ";
        //         particleFile << std::fixed << std::setprecision(8) << a_particle.pose.y << ", ";
        //         particleFile << std::fixed << std::setprecision(8) << a_particle.pose.theta << "\n";
        //     }
        //     particleFile.close();
        // }

   }
}


void OccupancyGridSLAM::updateMap(void)
{
    if(mode_ != localization_only)
    {
        // Process the map
        // mapper_.updateMap(currentScan_, currentPose_, map_);
        mapper_.updateMap(currentScan_, previousPose_, currentPose_, map_);
        haveMap_ = true;
    }

    // printf("currentPose_ = (%f, %f)\n", currentPose_.x, currentPose_.y);

    // Publish the map even in localization-only mode to ensure the visualization is meaningful
    // Send every 5th map -- about 1Hz update rate for map output -- can change if want more or less during operation
    if(mapUpdateCount_ % 5 == 0)
    {
        auto mapMessage = map_.toLCM();
        lcm_.publish(SLAM_MAP_CHANNEL, &mapMessage);
        //map_.saveToFile("current.map");

    }

    // if(mapUpdateCount_ % 420 == 0) {
    //     saveMap();
    // }

    ++mapUpdateCount_;
}
