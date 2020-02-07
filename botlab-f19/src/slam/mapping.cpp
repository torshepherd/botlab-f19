#include <slam/mapping.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <common/grid_utils.hpp>
#include <numeric>
#include <slam/moving_laser_scan.hpp>


Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
{
}


void Mapping::updateMap(const lidar_t& scan, const pose_xyt_t& pose, OccupancyGrid& map)
{
    //////////////// TODO: Implement your occupancy grid algorithm here ///////////////////////
}


void Mapping::updateMap(const lidar_t& scan, const pose_xyt_t& prevPose, const pose_xyt_t& currPose, OccupancyGrid& map)
{
    //////////////// TODO: Implement your occupancy grid algorithm here ///////////////////////
    int rayStride = 1;
    MovingLaserScan scanMoved(scan, prevPose, currPose, rayStride);
    for(size_t i = 0; i < scanMoved.size(); i++) {
        float xEnd = scanMoved[i].origin.x + scanMoved[i].range * cosf(scanMoved[i].theta);
        float yEnd = scanMoved[i].origin.y + scanMoved[i].range * sinf(scanMoved[i].theta);
        map.updateOneRay(scanMoved[i].origin.x, scanMoved[i].origin.y, xEnd, yEnd, kHitOdds_, kMissOdds_);
    }
}

int8_t Mapping::invSensorModel()
{
    // TODO
    return kHitOdds_;
    // return kMissOdds_;
}


