#ifndef MAPPING_OCCUPANCY_GRID_HPP
#define MAPPING_OCCUPANCY_GRID_HPP

#include <common/point.hpp>
#include <lcmtypes/occupancy_grid_t.hpp>
#include <algorithm>
#include <cstdint>
#include <vector>

typedef struct {
    int X1, Y1;
    int X2, Y2;
    int Increment;
    int UsingYIndex;
    int DeltaX, DeltaY;
    int DTerm;
    int IncrE, IncrNE;
    int XIndex, YIndex;
    int Flipped;
} bresenham_param_t;

typedef int8_t CellOdds;   ///< Type used to represent the data in a cell

/**
* OccupancyGrid represents a simple two-dimensional occupancy grid, which is a discretized representation of the
* environment. The basic abstraction of the occupancy grid is the cell. Each cell corresponds to a small square
* in the world. A cell represents the log-odds that some part of the environment within its boundaries contains
* an object.
* 
* The log-odds for a cell are represented using the typedef CellOdds. A value greater than 0 means more likely occupied.
* A value less than 0 means more likely free. 0 means even odds.
* 
* Most robot functionality doesn't operate using discrete representations. They use continuous values, like the length
* of a laser ray, the pose of a robot, etc. Functions to convert coordinates in the global, continuous coordinate system
* into the discretized coordinate system of the occupancy grid are found in occupancy_grid_utils.hpp.
* 
* This simple OccupancyGrid implementation does not support resizing. You can complete A1 without needing to resize
* the grid. Just make it big enough to start with.
* 
* You can change the typedef to use a different underlying value. The default int8_t provided plenty of resolution
* for successful mapping, though. Furthermore, increasing to a larger value type will, at minimum, quadruple the
* amount of memory used by OccupancyGrid. Also, if you change the value, you'll need to update occupancy_grid_t
* accordingly.
* 
* The grid is represented in row-major order, that is, a 3x3 grid is in memory as:
* 
*     0     1     2     3     4     5     6     7     8         (memory index)
*   (0,0) (1,0) (2,0) (0,1) (1,1) (2,1) (0,2) (1,2) (2,2)       (cell coordinate)
* 
* Therefore, if iterating through all the cells in a grid using nested for-loops -- the standard approach -- you should
* put the y-loop on the outside:
* 
*       for(std::size_t y = 0; y < grid.heightInCells(); ++y)
*       {
*           for(std::size_t x = 0; x < grid.widthInCells(); ++x)
*           {
*               // Operations on grid(x, y)
*           }
*       }
*
* This loop will be anywhere between nominally and dramatically faster than the alternative.
*/
class OccupancyGrid
{
public:
    
    /**
    * Default constructor for OccupancyGrid.
    * 
    * Create an OccupancyGrid with a width and height of 0. metersPerCell is set to 0.05. The global origin is (0,0).
    * 
    * This constructor is intended for use when creating a grid from an LCM message.
    */
    OccupancyGrid(void);
    
    /**
    * Constructor for OccupancyGrid.
    * 
    * The origin is (-widthInMeters/2, -heightInMeters/2).
    * 
    * \param    widthInMeters 
    * \param    heightInMeters
    * \param    metersPerCell
    * 
    * \pre  widthInMeters  > 0
    * \pre  heightInMeters > 0
    * \pre  metersPerCell <= widthInMeters
    * \pre  metersPerCell <= heightInMeters
    */
    OccupancyGrid(float widthInMeters,
                  float heightInMeters,
                  float metersPerCell);
    
    
    // Accessors for the properties of the grid
    int   widthInCells (void) const { return width_; }
    float widthInMeters(void) const { return width_ * metersPerCell_; }
    
    int   heightInCells (void) const { return height_; }
    float heightInMeters(void) const { return height_ * metersPerCell_; }
    
    float metersPerCell(void) const { return metersPerCell_; }
    float cellsPerMeter(void) const { return cellsPerMeter_; }
    
    Point<float> originInGlobalFrame(void) const { return globalOrigin_; }
    
    void setOrigin(float x, float y);

    /**
    * reset resets all cells into the grid to equal odds. All cells will have logOdds == 0.
    */
    void reset(void);
    
    /**
    * isCellInGrid checks to see if the specified cell is within the boundary of the OccupancyGrid.
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
    inline bool isCellInGrid(int x, int y) const { 
        return ( (x >= 0) && (x < width_) && (y >= 0) && (y < height_));
    }

    inline bool isCellInGrid(float x, float y) const { 
        return ( (x >= xMinMetersShrink_) && (x < xMaxMetersShrink_) && (y >= yMinMetersShrink_) && (y < yMaxMetersShrink_));
    }

    /**
    * odds retreives the log-odds that a specific cell is occupied.
    * 
    * \param    x           x-coordinate of the cell
    * \param    y           y-coordinate of the cell
    * \return   The log-odds at cell (x,y). If (x,y) is not contained in the grid, i.e. isCellInGrid(x,y) == false,
    *           then 0 is returned. This is equivalent to the log-odds of an unobserved cell in the map.
    */
    CellOdds logOdds(int x, int y) const;
    
    /**
    * setLogOdds sets the log-odds of occupancy for the specified cell if the cell falls within the grid boundary. If the
    * cell doesn't fall within the boundary, then nothing happens.
    * 
    * \param    x           x-coordinate of the cell
    * \param    y           y-coordinate of the cell
    * \param    logOdds     Log-odds to assign to the cell
    */
    void setLogOdds(int x, int y, CellOdds logOdds);
    
    /**
    * operator() provides unchecked access to the cell located at (x,y). If the cell isn't contained in the grid,
    * expect fireworks or a slow, ponderous death.
    * 
    * This version allows direct modification of the logOdds. If you are iterating through the grid
    * 
    * \param    x           x-coordinate of the cell
    * \param    y           y-coordinate of the cell
    * \return   A mutable reference to the cell (x,y)'s logOdds.
    */
    inline CellOdds& cell(int x, int y)       { return cells_[cellIndex(x, y)]; }
    inline CellOdds& operator()(int x, int y)       { return cells_[cellIndex(x, y)]; }
    inline CellOdds& operator()(float xF, float yF) {
        int x, y;
        toInt(xF, yF, x, y);
        return cells_[cellIndex(x, y)];
    }

    /**
    * operator() provides unchecked access to the cell located at (x,y). If the cell isn't contained in the grid,
    * expect fireworks or a slow, ponderous death.
    * 
    * \param    x           x-coordinate of the cell
    * \param    y           y-coordinate of the cell
    * \return   The logOdds of cell (x, y).
    */
    inline CellOdds  cell(int x, int y) const { return cells_[cellIndex(x, y)]; }
    inline CellOdds  operator()(int x, int y) const { return cells_[cellIndex(x, y)]; }
    inline CellOdds  operator()(float xF, float yF) const {
        int x, y;
        toInt(xF, yF, x, y);
        return cells_[cellIndex(x, y)];
    }
    inline CellOdds  operator()(int index) const { return cells_[index]; }
    inline size_t size() const { return cells_.size(); }

    /**
    * toLCM creates an LCM message from the grid.
    */
    occupancy_grid_t toLCM(void) const;
    
    /**
    * fromLCM populates the grid using an LCM message. The current contents of the grid are erased.
    */
    void fromLCM(const occupancy_grid_t& gridMessage);
    
    /**
    * saveToFile saves the OccupancyGrid to the specified file.
    * 
    * The format for saving is:
    * 
    *   origin_x origin_y width_in_cells height_in_cells meters_per_cell
    *   (0, 0) (0, 1) . . .
    *           .
    *           .
    *           .
    *   (0, height-1) . . . (width-1, height-1)
    * 
    * where each cell value is stored in ASCII text.
    * 
    * \param    filename            Name of the map to be saved
    * \return   True if the map is successfully saved. False if the file can't be opened or some other I/O error occurs.
    */
    bool saveToFile(const std::string& filename) const;
    
    /**
    * loadFromFile loads the OccupancyGrid from a file in the format specified in saveToFile.
    * 
    * Minimal error checking is performed, so an improperly formatted map file can produce very strange results.
    * 
    * \param    filename            Name of the map to be loaded
    * \return   True if the map is successfully loaded. False if the file can't be opened or some other I/O error occurs.
    */
    bool loadFromFile(const std::string& filename);

    void getBresenhamParameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params) const;
    void getCurrentPoint(bresenham_param_t *params, int *x, int *y) const;
    int getNextPoint(bresenham_param_t *params) const;
    bool findCollisionPoint(float x0, float y0, float x1, float y1, int& collision_x, int& collision_y) const;
    bool updateOneRay(float x0, float y0, float x1, float y1, const int8_t kHitOdds, const int8_t kMissOdds);
    void setMapOrigin(float x, float y);
    void calculateMeterParams();

    inline bool notInitialized(int x, int y) const {
        return (cells_[cellIndex(x, y)] == notInitializedValue_);
    }

    inline bool notInitialized(float xF, float yF) const {
        int x, y;
        toInt(xF, yF, x, y);
        return (cells_[cellIndex(x, y)] == notInitializedValue_);
    }

    inline bool notInitialized(int index) const {
        return (cells_[index] == notInitializedValue_);
    }

    inline void copyToDistanceGrid(Point<float>& mapOrigin,
                                    float& widthInMeters,
                                    float& heightInMeters,
                                    float& xMaxMeters,
                                    float& xMinMeters,
                                    float& yMaxMeters,
                                    float& yMinMeters,
                                    float& epsMeters,
                                    float& xMaxMetersShrink,
                                    float& xMinMetersShrink,
                                    float& yMaxMetersShrink,
                                    float& yMinMetersShrink) const {
        mapOrigin = mapOrigin_;
        widthInMeters = widthInMeters_;
        heightInMeters = heightInMeters_;
        xMaxMeters = xMaxMeters_;
        xMinMeters = xMinMeters_;
        yMaxMeters = yMaxMeters_;
        yMinMeters = yMinMeters_;
        epsMeters = epsMeters_;
        xMaxMetersShrink = xMaxMetersShrink_;
        xMinMetersShrink = xMinMetersShrink_;
        yMaxMetersShrink = yMaxMetersShrink_;
        yMinMetersShrink = yMinMetersShrink_;
    }

    inline void copyToSlam(float& xMaxMeters,
                            float& xMinMeters,
                            float& yMaxMeters,
                            float& yMinMeters) const {
        xMaxMeters = xMaxMeters_;
        xMinMeters = xMinMeters_;
        yMaxMeters = yMaxMeters_;
        yMinMeters = yMinMeters_;
    }

    // inline void toIntDraw(float xFloat, float yFloat, float& xInt, float& yInt) const {
    //     xInt = (xFloat - xMinMeters_) * cellsPerMeter_;
    //     yInt = (yFloat - yMinMeters_) * cellsPerMeter_;
    // }

    inline void toIntDraw(float xFloat, float yFloat, int& xInt, int& yInt) const {
        xInt = static_cast<int>( (xFloat - xMinMeters_) * cellsPerMeter_);
        yInt = static_cast<int>( (yFloat - yMinMeters_) * cellsPerMeter_);
        // xInt = static_cast<int>( (xFloat - xMinMeters_) * cellsPerMeter_ + 0.5 );
        // yInt = static_cast<int>( (yFloat - yMinMeters_) * cellsPerMeter_ + 0.5 );
    }

private:
    
    std::vector<CellOdds> cells_;       ///< The actual grid -- stored in row-column order
    
    int width_;                 ///< Width of the grid in cells
    int height_;                ///< Height of the grid in cells
    float metersPerCell_;
    float cellsPerMeter_;
    
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
    int8_t notInitializedValue_;

    // Convert between cells and the underlying vector index
    inline int cellIndex(int x, int y) const { return y*width_ + x; }

    // inline int toInt(float x) const {
    //     return ( (int) (x / metersPerCell_ + 0.5) );
    // }

    inline void toInt(float xFloat, float yFloat, int& xInt, int& yInt) const {
        xInt = static_cast<int>( (xFloat - xMinMeters_) * cellsPerMeter_);
        yInt = static_cast<int>( (yFloat - yMinMeters_) * cellsPerMeter_);
        // xInt = static_cast<int>( (xFloat - xMinMeters_) * cellsPerMeter_ + 0.5 );
        // yInt = static_cast<int>( (yFloat - yMinMeters_) * cellsPerMeter_ + 0.5 );
    }

    inline void hitCell(int nX, int nY, int8_t kHitOdds) {
        if (cells_[cellIndex(nX, nY)] == notInitializedValue_) {
            cells_[cellIndex(nX, nY)] = 0;
        }
        if (cells_[cellIndex(nX, nY)] + kHitOdds < 127) {
            cells_[cellIndex(nX, nY)] += kHitOdds;
        }
        else {
            cells_[cellIndex(nX, nY)] = 127;
        }
    }

    inline void missCell(int nX, int nY, int8_t kMissOdds) {
        if (cells_[cellIndex(nX, nY)] == notInitializedValue_) {
            cells_[cellIndex(nX, nY)] = 0;
        }
        if (cells_[cellIndex(nX, nY)] - kMissOdds > -127) {
            cells_[cellIndex(nX, nY)] -= kMissOdds;
        }
        else {
            cells_[cellIndex(nX, nY)] = -127;
        }
    }

};

#endif // MAPPING_OCCUPANCY_GRID_HPP
