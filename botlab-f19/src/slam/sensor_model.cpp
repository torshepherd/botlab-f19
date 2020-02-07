#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/grid_utils.hpp>


SensorModel::SensorModel(void)
{
    ///////// TODO: Handle any initialization needed for your sensor model
    sensorHitOdds_ = 3.0f;
    sensorMissOdds_ = 1.5f;
    sensorBehindOdds_ = 0.5f;
    sensorMaxOdds_ = 0.0f;
    maxLaserDistanceLB_ = 4.95f;
    flagSensorFunction_ = SENSORFUNCTION_BINARY;
    rayStride_ = 2;
}

bool SensorModel::likelihoodBinary(const OccupancyGrid& map, const adjusted_ray_t& a_ray, float& singleLikelihood) const {
    float xEnd = a_ray.origin.x + a_ray.range * cosf(a_ray.theta);
    float yEnd = a_ray.origin.y + a_ray.range * sinf(a_ray.theta);

    if (!map.isCellInGrid(xEnd, yEnd)) {
        return false;
    }
    if (a_ray.range > maxLaserDistanceLB_) {
        return false;
    }
    if (map.notInitialized(xEnd, yEnd)) {
        singleLikelihood = sensorBehindOdds_;
    }
    else {
        CellOdds cellValue = map(xEnd, yEnd);
        if (cellValue > 0) {
            float sensorOdds = static_cast<float>(cellValue);
            singleLikelihood = interpAB(0.0f, sensorBehindOdds_, 127.0f, sensorHitOdds_, sensorOdds);
        }
        else if (cellValue < 0) {
            float sensorOdds = static_cast<float>(cellValue);
            singleLikelihood = interpAB(0.0f, sensorBehindOdds_, -127.0f, sensorMissOdds_, sensorOdds);
        }
        else {
            singleLikelihood = sensorBehindOdds_;
        }
    }
    return true;
}

bool SensorModel::likelihoodLinear(const OccupancyGrid& map, const adjusted_ray_t& a_ray, float& singleLikelihood) const {
    float xEnd = a_ray.origin.x + a_ray.range * cosf(a_ray.theta);
    float yEnd = a_ray.origin.y + a_ray.range * sinf(a_ray.theta);

    if (!map.isCellInGrid(xEnd, yEnd)) {
        return false;
    }
    if (a_ray.range > maxLaserDistanceLB_) {
        return false;
    }
    if (map.notInitialized(xEnd, yEnd)) {
        singleLikelihood = sensorBehindOdds_;
    }
    else {
        CellOdds cellValue = map(xEnd, yEnd);
        if (cellValue > 0) {
            singleLikelihood = sensorHitOdds_;
        }
        else {
            singleLikelihood = sensorMissOdds_;
        }
    }
    return true;
}

float SensorModel::likelihood(const particle_t& sample, const lidar_t& scan, const OccupancyGrid& map) const
{
    ///////////// TODO: Implement your sensor model for calculating the likelihood of a particle given a laser scan //////////
    MovingLaserScan scanMoved(scan, sample.parent_pose, sample.pose, rayStride_);

    int validRayNum = 0;
    float sumLikelihood = 0.0f;
    for(size_t i = 0; i < scanMoved.size(); i++) {

        float singleLikelihood = 0.0f;
        bool flagValidRay = false;
        if (flagSensorFunction_ == SENSORFUNCTION_LINEAR) {
            flagValidRay = likelihoodLinear(map, scanMoved[i], singleLikelihood);
        }
        else {
            flagValidRay = likelihoodBinary(map, scanMoved[i], singleLikelihood);
        }
        if (flagValidRay) {
            validRayNum++;
            sumLikelihood += singleLikelihood;
        }
    }

    float meanLikelihood = 0.0f;
    if (validRayNum > 0) {
        meanLikelihood = sumLikelihood / static_cast<float>(validRayNum);
    }
    else {
        meanLikelihood = sensorBehindOdds_;
    }

    return meanLikelihood;
}
