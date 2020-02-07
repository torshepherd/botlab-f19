#ifndef SLAM_SENSOR_MODEL_HPP
#define SLAM_SENSOR_MODEL_HPP

class  lidar_t;
class  OccupancyGrid;
struct particle_t;
struct adjusted_ray_t;

enum SensorFunction
{
    SENSORFUNCTION_BINARY = 0,
    SENSORFUNCTION_LINEAR = 1
};

/**
* SensorModel implement a sensor model for computing the likelihood that a laser scan was measured from a
* provided pose, give a map of the environment.
* 
* A sensor model is compute the unnormalized likelihood of a particle in the proposal distribution.
*
* To use the SensorModel, a single method exists:
*
*   - double likelihood(const particle_t& particle, const lidar_t& scan, const OccupancyGrid& map)
*
* likelihood() computes the likelihood of the provided particle, given the most recent laser scan and map estimate.
*/
class SensorModel
{
public:

    /**
    * Constructor for SensorModel.
    */
    SensorModel(void);

    /**
    * likelihood computes the likelihood of the provided particle, given the most recent laser scan and map estimate.
    * 
    * \param    particle            Particle for which the log-likelihood will be calculated
    * \param    scan                Laser scan to use for estimating log-likelihood
    * \param    map                 Current map of the environment
    * \return   Likelihood of the particle given the current map and laser scan.
    */
    float likelihood(const particle_t& particle, const lidar_t& scan, const OccupancyGrid& map) const;
    bool likelihoodBinary(const OccupancyGrid& map, const adjusted_ray_t& a_ray, float& singleLikelihood) const;
    bool likelihoodLinear(const OccupancyGrid& map, const adjusted_ray_t& a_ray, float& singleLikelihood) const;

    inline float interpAB(float aX, float aY, float bX, float bY, float cX) const {
        return ( (bY - aY) / (bX - aX) * (cX - aX) + aY );
    }

private:
    
    ///////// TODO: Add any private members for your SensorModel ///////////////////
    float sensorHitOdds_;
    float sensorMissOdds_;
    float sensorBehindOdds_;
    float sensorMaxOdds_;
    float maxLaserDistanceLB_;
    SensorFunction flagSensorFunction_;
    int rayStride_;
};

#endif // SLAM_SENSOR_MODEL_HPP
