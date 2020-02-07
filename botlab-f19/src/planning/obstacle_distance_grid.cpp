#include <planning/obstacle_distance_grid.hpp>
// #include <slam/occupancy_grid.hpp>
#include <algorithm>
#include <queue>
#include <chrono>

#include <fstream>
#include <iostream>
// using namespace std;

#ifndef SQRT2
#define SQRT2 1.41421356237
#endif

State::State(){
	key_ = -1;
	g_ = 20000000;
	h_ = 0;
    penalty_ = -1;
	parent_ = -1;
	flagClosed_ = false;
}

State::State(int key){
	key_ = key;
	g_ = 20000000;
	h_ = 0;
    penalty_ = -1;
	parent_ = -1;
	flagClosed_ = false;
}

State::~State(){}

double ObstacleDistanceGrid::getTime(){
	std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
	int passed_duration = (int) std::chrono::duration_cast <std::chrono::milliseconds> (now-start_).count();
	return (( (double)passed_duration ) / 1000.0);
}

double ObstacleDistanceGrid::getDuration(){
	std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
	int passed_duration = (int) std::chrono::duration_cast <std::chrono::milliseconds> (now-start_).count();
	return (( (double)passed_duration ) / 1000.0 - prev_time_);
}

void ObstacleDistanceGrid::markTime(){
	std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
	int passed_duration = (int) std::chrono::duration_cast <std::chrono::milliseconds> (now-start_).count();
	prev_time_ = (( (double)passed_duration ) / 1000.0);
}

ObstacleDistanceGrid::ObstacleDistanceGrid(void)
: width_(0)
, height_(0)
, widthTimesHeight_(0)
, metersPerCell_(0.05f)
, cellsPerMeter_(20.0f)
{
    theHeap_ = nullptr;
    theHeap3D_ = nullptr;
    flagSearchHistory_ = false;
    flagSearchHistory3D_ = false;

    obstacleThresh_ = 30;
    freespaceThresh_ = -10;
    largeDistance_ = 10000.0f;

	start_ = std::chrono::system_clock::now();
	markTime();

    dirNum_ = 8;
    dX_.resize(8);
    dY_.resize(8);
    dX3D_.resize(8);
    dY3D_.resize(8);
    dAngle_.resize(8);
    angleFloat_.resize(8);
    dCost_.resize(8);
    dCost0_.resize(8);
    dCost03D_.resize(8);
    dCost3D_.resize(8);
    dAngleCost_.resize(8);
    dAngleCost0_.resize(8);

    dX_[0] =  1;
    dX_[1] =  0;
    dX_[2] = -1;
    dX_[3] =  0;
    dX_[4] =  1;
    dX_[5] = -1;
    dX_[6] = -1;
    dX_[7] =  1;

    dY_[0] =  0;
    dY_[1] =  1;
    dY_[2] =  0;
    dY_[3] = -1;
    dY_[4] =  1;
    dY_[5] =  1;
    dY_[6] = -1;
    dY_[7] = -1;

    dX3D_[0] =  1;
    dX3D_[1] =  1;
    dX3D_[2] =  0;
    dX3D_[3] = -1;
    dX3D_[4] = -1;
    dX3D_[5] = -1;
    dX3D_[6] =  0;
    dX3D_[7] =  1;

    dY3D_[0] =  0;
    dY3D_[1] =  1;
    dY3D_[2] =  1;
    dY3D_[3] =  1;
    dY3D_[4] =  0;
    dY3D_[5] = -1;
    dY3D_[6] = -1;
    dY3D_[7] = -1;

    dAngle_[0] = 0;
    dAngle_[1] = -1 + dirNum_;
    dAngle_[2] = 1;
    dAngle_[3] = -2 + dirNum_;
    dAngle_[4] = 2;
    dAngle_[5] = -3 + dirNum_;
    dAngle_[6] = 3;
    dAngle_[7] = 4;

    angleFloat_[0] = 0.0;
    angleFloat_[1] = MY_PI * 0.25;
    angleFloat_[2] = MY_PI * 0.50;
    angleFloat_[3] = MY_PI * 0.75;
    angleFloat_[4] = MY_PI;
    angleFloat_[5] = MY_PI * (-0.75);
    angleFloat_[6] = MY_PI * (-0.50);
    angleFloat_[7] = MY_PI * (-0.25);

    dCost0_[0] =  1.0;
    dCost0_[1] =  1.0;
    dCost0_[2] =  1.0;
    dCost0_[3] =  1.0;
    dCost0_[4] =  SQRT2;
    dCost0_[5] =  SQRT2;
    dCost0_[6] =  SQRT2;
    dCost0_[7] =  SQRT2;

    dCost03D_[0] =  1.0;
    dCost03D_[1] =  SQRT2;
    dCost03D_[2] =  1.0;
    dCost03D_[3] =  SQRT2;
    dCost03D_[4] =  1.0;
    dCost03D_[5] =  SQRT2;
    dCost03D_[6] =  1.0;
    dCost03D_[7] =  SQRT2;

    dAngleCost0_[0] = 0.0;
    dAngleCost0_[1] = 1.0;
    dAngleCost0_[2] = 1.0;
    dAngleCost0_[3] = 2.0;
    dAngleCost0_[4] = 2.0;
    dAngleCost0_[5] = 3.0;
    dAngleCost0_[6] = 3.0;
    dAngleCost0_[7] = 3.0;

    for (int i = 0; i < dirNum_; i++) {
        dCost_[i] = dCost0_[i] * metersPerCell_;
        dCost3D_[i] = dCost03D_[i] * metersPerCell_;
    }

    setAngleCost(metersPerCell_);
}

void ObstacleDistanceGrid::setAngleCost(float magnitute) {
    for(size_t i = 0; i < dAngleCost0_.size(); i++) {
        dAngleCost_[i] = dAngleCost0_[i] * magnitute;
    }
}


void ObstacleDistanceGrid::setDistances(const OccupancyGrid& map)
{
    resetGrid(map);
    
    ///////////// TODO: Implement an algorithm to mark the distance to the nearest obstacle for every cell in the map.
    std::fill(cells_.begin(), cells_.end(), -1.0); // Initial value
    // std::fill(cellSeen_.begin(), cellSeen_.end(), false);

    setDistancesBFS(map);

}

void ObstacleDistanceGrid::setDistancesBFS(const OccupancyGrid& map) {
    std::queue<int> openSet;
    const int mapSize = static_cast<int>(map.size());
    for (int i = 0; i < mapSize; i++) {
        // Check for initialization
        // if (map.notInitialized(i)) {
        //     continue;
        // }
        if (map(i) >= obstacleThresh_) {
            cells_[i] = 0.0;
            openSet.push(i);
        }
    }

    // If no obstacles in the map, then set the freespace to large distance values
    if (openSet.size() <= 0) {
        for (int i = 0; i < mapSize; i++) {
            // Check for initialization
            // if (map.notInitialized(i)) {
            //     continue;
            // }
            if (map(i) <= freespaceThresh_) {
                cells_[i] = largeDistance_;
            }
        }
        return;
    }

    // If obstacles in the map
    // Do breadth first search to ditermine distance
    while ( !openSet.empty() ) {
        int currIndex = openSet.front();
        openSet.pop();
        int currX, currY;
        cellXY(currIndex, currX, currY);
        for (int i = 0; i < dirNum_; i++) {
            int nextX = currX + dX_[i];
            int nextY = currY + dY_[i];
            if (!isCellInGrid(nextX, nextY)) {
                continue;
            }
            int nextIndex = cellIndex(nextX, nextY);
            // Check for initialization
            // if (map.notInitialized(currX, currY)) {
            //     continue;
            // }
            if (map(nextX, nextY) < freespaceThresh_) {
                if (cells_[nextIndex] < -0.5) {
                    cells_[nextIndex] = cells_[currIndex] + metersPerCell_;
                    openSet.push(nextIndex);
                }
            }
        }

    }
}


void ObstacleDistanceGrid::resetGrid(const OccupancyGrid& map)
{
    // Ensure the same cell sizes for both grid
    metersPerCell_ = map.metersPerCell();
    cellsPerMeter_ = map.cellsPerMeter();
    globalOrigin_ = map.originInGlobalFrame();

    // Copy over grid map values
    map.copyToDistanceGrid(mapOrigin_,
                            widthInMeters_,
                            heightInMeters_,
                            xMaxMeters_,
                            xMinMeters_,
                            yMaxMeters_,
                            yMinMeters_,
                            epsMeters_,
                            xMaxMetersShrink_,
                            xMinMetersShrink_,
                            yMaxMetersShrink_,
                            yMinMetersShrink_);

    for (int i = 0; i < dirNum_; i++) {
        dCost_[i] = dCost0_[i] * metersPerCell_;
        dCost3D_[i] = dCost03D_[i] * metersPerCell_;
    }
    setAngleCost(metersPerCell_);

    // If the grid is already the correct size, nothing needs to be done
    if((width_ == map.widthInCells()) && (height_ == map.heightInCells()))
    {
        return;
    }
    
    // Otherwise, resize the vector that is storing the data
    width_ = map.widthInCells();
    height_ = map.heightInCells();
    widthTimesHeight_ = width_ * height_;
    
    cells_.clear();
    cells_.resize(width_ * height_);
    std::fill(cells_.begin(), cells_.end(), -1.0); // Initial value
    // cellSeen_.resize(width_ * height_);
    clearSearch();
}

bool ObstacleDistanceGrid::saveToFile(const std::string& filename) const
{
    std::ofstream out(filename);
    if(!out.is_open())
    {
        std::cerr << "ERROR: OccupancyGrid::saveToFile: Failed to save to " << filename << '\n';
        return false;
    }
    
    // Write header
    // out << globalOrigin_.x << ' ' << globalOrigin_.y << ' ' << width_ << ' ' << height_ << ' ' << metersPerCell_ << '\n';
    
    // Write out each cell value
    for(int y = 0; y < height_; ++y)
    {
        for(int x = 0; x < width_; ++x)
        {
            // Unary plus forces output to be a a number rather than a character
             out << cells_[cellIndex(x, y)] << ' ';
        }
        out << '\n';
    }
    
    return out.good();
}
