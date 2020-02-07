#ifndef PLANNING_OBSTACLE_DISTANCE_GRID_HPP
#define PLANNING_OBSTACLE_DISTANCE_GRID_HPP

#include <common/point.hpp>
#include <vector>
#include <slam/occupancy_grid.hpp>
#include <planning/my_heap.hpp>
#include <chrono>

// class OccupancyGrid;

#ifndef MY_PI
#define MY_PI 3.141592653589
#endif

#ifndef DO_DEBUG
#define DO_DEBUG false
#endif

#ifndef VALUE_TYPE
#define VALUE_TYPE float
#endif

#ifndef MyHeapType
#define MyHeapType MyHeapFloat
#endif

#ifndef HeapNodeType
#define HeapNodeType HeapNodeFloat
#endif

class State{
public:
	int key_;
	VALUE_TYPE g_;
	VALUE_TYPE h_;
    VALUE_TYPE penalty_;
	int parent_;
	bool flagClosed_;
	State();
	State(int key);
	~State();
};

/**
* ObstacleDistanceGrid stores the distance to the nearest obstacle for each cell in the occupancy grid.
* 
*  - An obstacle is any cell with logOdds > 0.
*  - The size of the grid is identical to the occupancy grid whose obstacle distances the distance grid stores.
* 
* To update the grid, simply pass an OccupancyGrid to the setDistances method.
*/
class ObstacleDistanceGrid
{
public:
    
    /**
    * Default constructor for ObstacleDistanceGrid.
    * 
    * Create an ObstacleDistanceGrid with a width and height of 0. metersPerCell is set to 0.05.
    * The global origin is (0,0).
    */
    ObstacleDistanceGrid(void);
    
    // Accessors for the properties of the grid
    int   widthInCells (void) const { return width_; }
    float widthInMeters(void) const { return width_ * metersPerCell_; }
    
    int   heightInCells (void) const { return height_; }
    float heightInMeters(void) const { return height_ * metersPerCell_; }
    
    float metersPerCell(void) const { return metersPerCell_; }
    float cellsPerMeter(void) const { return cellsPerMeter_; }
    
    Point<float> originInGlobalFrame(void) const { return globalOrigin_; }
    
    /**
    * setDistances sets the obstacle distances stored in the grid based on the provided occupancy grid map of the
    * environment.
    */
    void setDistances(const OccupancyGrid& map);
    
    /**
    * isCellInGrid checks to see if the specified cell is within the boundary of the ObstacleDistanceGrid.
    * 
    * This test is equivalent to:
    * 
    *   (0 <= x < widthInCells) && (0 <= y < heightInCells)
    * 
    * \param    x           x-coordinate of the cell
    * \param    y           y-coordinate of the cell
    * \return   True if the cell is in the grid boundary as specified above.
    */
    // bool isCellInGrid(int x, int y) const;
    inline bool isCellInGrid(int x, int y) const
    {
        return (x >= 0) && (x < width_) && (y >= 0) && (y < height_);
    }
    
    /**
    * operator() provides unchecked access to the cell located at (x,y). If the cell isn't contained in the grid,
    * expect fireworks or a slow, ponderous death.
    * 
    * \param    x           x-coordinate of the cell
    * \param    y           y-coordinate of the cell
    * \return   The distance to the nearest obstacle to cell (x, y).
    */
    inline float operator()(int x, int y) const { return cells_[cellIndex(x, y)]; }
    inline float& operator()(int x, int y) { return cells_[cellIndex(x, y)]; }
    inline float operator()(int index) const { return cells_[index]; }
    inline float& operator()(int index) { return cells_[index]; }

    void setDistancesBFS(const OccupancyGrid& map);

    // Convert between cells and the underlying vector index
    inline int cellIndex(int x, int y) const { return y*width_ + x; }

    inline void cellXY(int index, int& x, int& y) const {
        y = index / width_;
        x = index - y * width_;
    }

    inline int cellIndex(int x, int y, int angle) const { return angle*widthTimesHeight_ +  y*width_ + x; }

    inline void cellXYAngle(int index, int& x, int& y, int& angle) const {
        angle = index / widthTimesHeight_;
        index -= angle * widthTimesHeight_;
        y = index / width_;
        x = index - y * width_;
    }

    inline void toInt(float xFloat, float yFloat, int& xInt, int& yInt) const {
        xInt = static_cast<int>( (xFloat - xMinMeters_) * cellsPerMeter_);
        yInt = static_cast<int>( (yFloat - yMinMeters_) * cellsPerMeter_);
        // xInt = static_cast<int>( (xFloat - xMinMeters_) * cellsPerMeter_ + 0.5 );
        // yInt = static_cast<int>( (yFloat - yMinMeters_) * cellsPerMeter_ + 0.5 );
    }

    inline void toFloat(int xInt, int yInt, float& xFloat, float& yFloat) const {
        xFloat = static_cast<float>(xInt) * metersPerCell_ + xMinMeters_;
        yFloat = static_cast<float>(yInt) * metersPerCell_ + yMinMeters_;
    }

    inline void toInt(float xFloat, float yFloat, float angleFloat, int& xInt, int& yInt, int& angleInt) const {
        xInt = static_cast<int>( (xFloat - xMinMeters_) * cellsPerMeter_);
        yInt = static_cast<int>( (yFloat - yMinMeters_) * cellsPerMeter_);

        float angleRes = 2 * MY_PI / dirNum_;
        if(angleFloat < (-angleRes/2) ) {
            angleFloat += 2 * MY_PI;
        }
        angleInt = static_cast<int>(angleFloat / angleRes + 0.5);
    }

    inline void toFloat(int xInt, int yInt, int angleInt, float& xFloat, float& yFloat, float& angleFloat) const {
        xFloat = static_cast<float>(xInt) * metersPerCell_ + xMinMeters_;
        yFloat = static_cast<float>(yInt) * metersPerCell_ + yMinMeters_;
        angleFloat = angleFloat_[angleInt];
    }

    inline bool isObstacle(int index, float minDistanceToObstacle, bool flagIncludeUninitialized) const {
        minDistanceToObstacle += metersPerCell_ / 2;
        if (flagIncludeUninitialized) {
            return ( cells_[index] <= minDistanceToObstacle );
        }
        else {
            return ( cells_[index] <= minDistanceToObstacle && cells_[index] > -0.5 );
        }
    }

    inline bool isUninitialized(int index) const {
        return ( cells_[index] < -0.5 );
    }

    inline void setFlagSearchHistory(bool flagSearchHistory) {
        flagSearchHistory_ = flagSearchHistory;
    }

    inline bool flagSearchHistory() const {
        return flagSearchHistory_;
    }

    inline void setFlagSearchHistory3D(bool flagSearchHistory3D) {
        flagSearchHistory3D_ = flagSearchHistory3D;
    }

    inline bool flagSearchHistory3D() const {
        return flagSearchHistory3D_;
    }

    inline void clearSearch() {
        states_.clear();
        states_.resize(width_ * height_);
        if (theHeap_ != nullptr) {
            delete theHeap_;
        }
        theHeap_ = new MyHeapType(width_ * height_);
        flagSearchHistory_ = false;
    }

    inline void clearSearch3D() {
        states3D_.clear();
        states3D_.resize(width_ * height_ * dirNum_);
        if (theHeap3D_ != nullptr) {
            delete theHeap3D_;
        }
        theHeap3D_ = new MyHeapType(width_ * height_ * dirNum_);
        flagSearchHistory3D_ = false;
    }

    inline int size() const {return width_ * height_;}

    double getTime();

    double getDuration();

    void markTime();

    void setAngleCost(float magnitute);

    inline void get3DCost(int dir, int currX, int currY, int currAngle, int& nextX, int& nextY, int& nextAngle, float& cost) {
        if(dir == 0) {
            nextX = currX + dX3D_[currAngle];
            nextY = currY + dY3D_[currAngle];
            nextAngle = currAngle;
            cost = dCost3D_[currAngle];
        }
        else {
            nextX = currX;
            nextY = currY;
            nextAngle = (currAngle + dAngle_[dir]) % dirNum_;
            cost = dAngleCost_[dir];
        }
    }

    inline float angleDist(int angle1, int angle2) {
        int dAngle1 = angle1 - angle2;
        if (dAngle1 < 0) {
            dAngle1 += dirNum_;
        }
        int dAngle2 = (angle2 - angle1);
        if (dAngle2 < 0) {
            dAngle2 += dirNum_;
        }
        if (dAngle1 < dAngle2) {
            return (static_cast<float>(dAngle1) * dAngleCost_[1]);
        }
        else {
            return (static_cast<float>(dAngle2) * dAngleCost_[1]);
        }
    }

    bool saveToFile(const std::string& filename) const;

    std::vector<State> states_;
    MyHeapType* theHeap_;

    std::vector<State> states3D_;
    MyHeapType* theHeap3D_;

    int dirNum_;
    std::vector<int> dX_;
    std::vector<int> dY_;
    std::vector<int> dX3D_;
    std::vector<int> dY3D_;
    std::vector<int> dAngle_;
    std::vector<float> angleFloat_;
    std::vector<float> dCost_;
    std::vector<float> dAngleCost_;
    std::vector<float> dCost3D_;

private:
    
    std::vector<float> cells_;          ///< The actual grid -- stored in row-major order
    // std::vector<bool> cellSeen_;          ///< The seen grid -- stored in row-major order
    
    int width_;                 ///< Width of the grid in cells
    int height_;                ///< Height of the grid in cells
    int widthTimesHeight_;        ///< Height * width of the grid in cells
    float metersPerCell_;       ///< Side length of a cell
    float cellsPerMeter_;       ///< Number of cells in a meter
    bool flagSearchHistory_;
    bool flagSearchHistory3D_;
    
    Point<float> globalOrigin_;         ///< Origin of the grid in global coordinates
    Point<float> mapOrigin_;         ///< Origin of the grid in global coordinates

    float widthInMeters_;
    float heightInMeters_;
    float xMaxMeters_;
    float xMinMeters_;
    float yMaxMeters_;
    float yMinMeters_;
    float epsMeters_;
    float xMaxMetersShrink_;
    float xMinMetersShrink_;
    float yMaxMetersShrink_;
    float yMinMetersShrink_;

    CellOdds obstacleThresh_;
    CellOdds freespaceThresh_;

    float largeDistance_;

    std::vector<float> dCost0_;
    std::vector<float> dAngleCost0_;
    std::vector<float> dCost03D_;
    std::chrono::system_clock::time_point start_;
	double prev_time_;

    void resetGrid(const OccupancyGrid& map);
    
    // Allow private write-access to cells
    inline float& distance(int x, int y) { return cells_[cellIndex(x, y)]; }
};

#endif // PLANNING_OBSTACLE_DISTANCE_GRID_HPP
