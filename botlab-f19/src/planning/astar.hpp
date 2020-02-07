#ifndef PLANNING_ASTAR_HPP
#define PLANNING_ASTAR_HPP

#include <lcmtypes/robot_path_t.hpp>
#include <lcmtypes/pose_xyt_t.hpp>

class ObstacleDistanceGrid;

/**
* SearchParams defines the parameters to use when searching for a path. See associated comments for details
*/
struct SearchParams
{
    float minDistanceToObstacle;   ///< The minimum distance a robot can be from an obstacle before
                                    ///< a collision occurs
                                    
    float maxDistanceWithCost;     ///< The maximum distance from an obstacle that has an associated cost. The planned
                                    ///< path will attempt to stay at least this distance from obstacles unless it must
                                    ///< travel closer to actually find a path
                                    
    float distanceCostExponent;    ///< The exponent to apply to the distance cost, whose function is:
                                    ///<   pow(maxDistanceWithCost - cellDistance, distanceCostExponent)
                                    ///< for cellDistance > minDistanceToObstacle && cellDistance < maxDistanceWithCost
};


/**
* search_for_path uses an A* search to find a path from the start to goal poses. The search assumes a circular robot
* 
* \param    start           Starting pose of the robot
* \param    goal            Desired goal pose of the robot
* \param    distances       Distance to the nearest obstacle for each cell in the grid
* \param    params          Parameters specifying the behavior of the A* search
* \return   The path found to the goal, if one exists. If the goal is unreachable, then a path with just the initial
*   pose is returned, per the robot_path_t specification.
*/
robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             ObstacleDistanceGrid& distances,
                             const SearchParams& params, bool flagPushGoal);

float penaltyFunction(const SearchParams& params, float distanceToObstacle1);

float hFunction(float xFloat, float yFloat, float goalXFloat, float goalYFloat, int flagHeuristics);

bool getPath(int startX, int startY, int goalX, int goalY, const ObstacleDistanceGrid& map, robot_path_t& path, bool flagRemoveRedundantPoints);

bool getPath3D(int startX, int startY, int startAngle, int goalX, int goalY, int goalAngle, const ObstacleDistanceGrid& map, robot_path_t& path, bool flagRemoveRedundantPoints);

bool aStar(int startX, int startY, int goalX, int goalY,
            ObstacleDistanceGrid& map, const SearchParams& params, bool flagIncludeUninitialized,
            int flagHeuristics, float weight, int& closedNum, int& openNum, float& pathCost);

bool aStar3D(int startX, int startY, int startAngle, int goalX, int goalY, int goalAngle,
            ObstacleDistanceGrid& map, const SearchParams& params, bool flagIncludeUninitialized,
            int flagHeuristics, float weight, int& closedNum, int& openNum, float& pathCost);

int getNodeDist(int x1, int y1, int x2, int y2);

int getFrontier(const ObstacleDistanceGrid& map, const SearchParams& params, const pose_xyt_t& start);

int getFreeSpace(const ObstacleDistanceGrid& map, const SearchParams& params, const pose_xyt_t& start);

robot_path_t pathToFreeSpace(const pose_xyt_t& start, 
                             const ObstacleDistanceGrid& map,
                             const SearchParams& params);

#endif // PLANNING_ASTAR_HPP
