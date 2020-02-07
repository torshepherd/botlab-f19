#include <slam/occupancy_grid.hpp>
#include <fstream>
#include <cassert>

#include <iostream>
using namespace std;


OccupancyGrid::OccupancyGrid(void)
: width_(0)
, height_(0)
, metersPerCell_(0.05f)
, cellsPerMeter_(1.0 / metersPerCell_)
, globalOrigin_(0, 0)
, mapOrigin_(0, 0)
, widthInMeters_(0.0f)
, heightInMeters_(0.0f)
, xMaxMeters_(0.0f)
, xMinMeters_(0.0f)
, yMaxMeters_(0.0f)
, yMinMeters_(0.0f)
, epsMeters_(1e-6f)
, xMaxMetersShrink_(0.0f)
, xMinMetersShrink_(0.0f)
, yMaxMetersShrink_(0.0f)
, yMinMetersShrink_(0.0f)
, notInitializedValue_(0)
{
    std::fill(cells_.begin(), cells_.end(), notInitializedValue_);
}


OccupancyGrid::OccupancyGrid(float widthInMeters,
                             float heightInMeters,
                             float metersPerCell)
: metersPerCell_(metersPerCell)
, globalOrigin_(-widthInMeters/2.0f, -heightInMeters/2.0f)
, mapOrigin_(0.0f, 0.0f)
{
    assert(widthInMeters  > 0.0f);
    assert(heightInMeters > 0.0f);
    assert(metersPerCell_ <= widthInMeters);
    assert(metersPerCell_ <= heightInMeters);
    
    cellsPerMeter_ = 1.0f / metersPerCell_;
    width_         = widthInMeters * cellsPerMeter_;
    height_        = heightInMeters * cellsPerMeter_;
    calculateMeterParams();
    
    cells_.resize(width_ * height_);
    reset();
}

void OccupancyGrid::setOrigin(float x, float y){

    reset();

    globalOrigin_.x -= x;
    globalOrigin_.y -= y;

  //  cout << "set global origin to: " << x << ", " << y << "\n";
}

void OccupancyGrid::reset(void)
{
//    cout << "reset!\n";
    std::fill(cells_.begin(), cells_.end(), notInitializedValue_);
}


// bool OccupancyGrid::isCellInGrid(int x, int y) const
// { 
//     bool xCoordIsValid = (x >= 0) && (x < width_);
//     bool yCoordIsValid = (y >= 0) && (y < height_);
//     return xCoordIsValid && yCoordIsValid;
// }


CellOdds OccupancyGrid::logOdds(int x, int y) const
{
    if(isCellInGrid(x, y))
    {
        return operator()(x, y);
    }
    
    return 0;
}


void OccupancyGrid::setLogOdds(int x, int y, CellOdds value)
{
    if(isCellInGrid(x, y))
    {
        operator()(x, y) = value;
    }
}


occupancy_grid_t OccupancyGrid::toLCM(void) const
{
    occupancy_grid_t grid;

    grid.origin_x        = globalOrigin_.x;
    grid.origin_y        = globalOrigin_.y;
    grid.meters_per_cell = metersPerCell_;
    grid.width           = width_;
    grid.height          = height_;
    grid.num_cells       = cells_.size();
    grid.cells           = cells_;
    
    return grid;
}


void OccupancyGrid::fromLCM(const occupancy_grid_t& gridMessage)
{
    globalOrigin_.x = gridMessage.origin_x;
    globalOrigin_.y = gridMessage.origin_y;
    metersPerCell_  = gridMessage.meters_per_cell;
    cellsPerMeter_  = 1.0f / gridMessage.meters_per_cell;
    height_         = gridMessage.height;
    width_          = gridMessage.width;
    cells_          = gridMessage.cells;
    calculateMeterParams();
}


bool OccupancyGrid::saveToFile(const std::string& filename) const
{
    std::ofstream out(filename);
    if(!out.is_open())
    {
        std::cerr << "ERROR: OccupancyGrid::saveToFile: Failed to save to " << filename << '\n';
        return false;
    }
    
    // Write header
    out << globalOrigin_.x << ' ' << globalOrigin_.y << ' ' << width_ << ' ' << height_ << ' ' << metersPerCell_ << '\n';
    
    // Write out each cell value
    for(int y = 0; y < height_; ++y)
    {
        for(int x = 0; x < width_; ++x)
        {
            // Unary plus forces output to be a a number rather than a character
             out << +logOdds(x, y) << ' ';
        }
        out << '\n';
    }
    
    return out.good();
}


bool OccupancyGrid::loadFromFile(const std::string& filename)
{
    std::ifstream in(filename);
    if(!in.is_open())
    {
        std::cerr << "ERROR: OccupancyGrid::loadFromFile: Failed to load from " << filename << '\n';
        return false;
    }
    
    width_ = -1;
    height_ = -1;
    
    // Read header
    in >> globalOrigin_.x >> globalOrigin_.y >> width_ >> height_ >> metersPerCell_;
    calculateMeterParams();
    
    // Check sanity of values
    assert(width_ > 0);
    assert(height_ > 0);
    assert(metersPerCell_ > 0.0f);
    
    // Allocate new memory for the grid
    cells_.resize(width_ * height_);
    
    // // Allocate new memory for the grid
    // cells_.resize(width_ * height_);
    // Read in each cell value
    int odds = 0; // read in as an int so it doesn't convert the number to the corresponding ASCII code
    for(int y = 0; y < height_; ++y)
    {
        for(int x = 0; x < width_; ++x)
        {
            in >> odds;
            setLogOdds(x, y, odds);
        }
    }
    
    return true;
}

void OccupancyGrid::setMapOrigin(float x, float y) {
    // reset();
    mapOrigin_.x = x;
    mapOrigin_.y = y;
}

void OccupancyGrid::calculateMeterParams() {
    widthInMeters_ = ((float) width_) * metersPerCell_;
    heightInMeters_ = ((float) height_) * metersPerCell_;
    xMaxMeters_ = mapOrigin_.x + widthInMeters_/2.0f;
    xMinMeters_ = mapOrigin_.x - widthInMeters_/2.0f;
    yMaxMeters_ = mapOrigin_.y + heightInMeters_/2.0f;
    yMinMeters_ = mapOrigin_.y - heightInMeters_/2.0f;
    epsMeters_ = 1e-6;
    xMaxMetersShrink_ = mapOrigin_.x + widthInMeters_/2.0f - epsMeters_;
    xMinMetersShrink_ = mapOrigin_.x - widthInMeters_/2.0f + epsMeters_;
    yMaxMetersShrink_ = mapOrigin_.y + heightInMeters_/2.0f - epsMeters_;
    yMinMetersShrink_ = mapOrigin_.y - heightInMeters_/2.0f + epsMeters_;
    notInitializedValue_ = 0;
}

void OccupancyGrid::getBresenhamParameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params) const
{
    params->UsingYIndex = 0;

    if (p2x == p1x || fabs((float)(p2y-p1y)/(float)(p2x-p1x)) > 1)
        (params->UsingYIndex)++;

    if (params->UsingYIndex)
    {
        params->Y1=p1x;
        params->X1=p1y;
        params->Y2=p2x;
        params->X2=p2y;
    }
    else
    {
        params->X1=p1x;
        params->Y1=p1y;
        params->X2=p2x;
        params->Y2=p2y;
    }

    if ((p2x - p1x) * (p2y - p1y) < 0)
    {
        params->Flipped = 1;
        params->Y1 = -params->Y1;
        params->Y2 = -params->Y2;
    }
    else
        params->Flipped = 0;

    if (params->X2 > params->X1)
        params->Increment = 1;
    else
        params->Increment = -1;

    params->DeltaX=params->X2-params->X1;
    params->DeltaY=params->Y2-params->Y1;

    params->IncrE=2*params->DeltaY*params->Increment;
    params->IncrNE=2*(params->DeltaY-params->DeltaX)*params->Increment;
    params->DTerm=(2*params->DeltaY-params->DeltaX)*params->Increment;

    params->XIndex = params->X1;
    params->YIndex = params->Y1;
}


void OccupancyGrid::getCurrentPoint(bresenham_param_t *params, int *x, int *y) const
{
  if (params->UsingYIndex)
    {
      *y = params->XIndex;
      *x = params->YIndex;
      if (params->Flipped)
        *x = -*x;
    }
  else
    {
      *x = params->XIndex;
      *y = params->YIndex;
      if (params->Flipped)
        *y = -*y;
    }
}

int OccupancyGrid::getNextPoint(bresenham_param_t *params) const
{
  if (params->XIndex == params->X2)
    {
      return 0;
    }
  params->XIndex += params->Increment;
  if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
    params->DTerm += params->IncrE;
  else
    {
      params->DTerm += params->IncrNE;
      params->YIndex += params->Increment;
    }
  return 1;
}

bool OccupancyGrid::findCollisionPoint(float x0, float y0, float x1, float y1, int& collision_x, int& collision_y) const
{
	bresenham_param_t params;
    int nX0, nY0, nX1, nY1;
    collision_x = -1;
    collision_y = -1;

    //printf("checking link <%f %f> to <%f %f>\n", x0,y0,x1,y1);
    
    //make sure the line start point is inside the environment
    if( !isCellInGrid(x0, y0) ) {
        return true;
    }

    // nX0 = toInt(x0);
    // nY0 = toInt(y0);
    // nX1 = toInt(x1);
    // nY1 = toInt(y1);
    toInt(x0, y0, nX0, nY0);
    toInt(x1, y1, nX1, nY1);

    //printf("checking link <%d %d> to <%d %d>\n", nX0,nY0,nX1,nY1);

	//iterate through the points on the segment
    if(nX0 == nX1 && nY0 == nY1) {
		if( cell(nX0, nY0) ) {
            collision_x = nX0;
            collision_y = nY0;
            return true;
        }
    }
    else {
        getBresenhamParameters(nX0, nY0, nX1, nY1, &params);
        do {
        	  int nX, nY; 
            getCurrentPoint(&params, &nX, &nY);
            if( !isCellInGrid(nX, nY) ) {
                return false;
            }
            if( cell(nX, nY) ) {
                collision_x = nX;
                collision_y = nY;
                return true;
            }
        } while (getNextPoint(&params));
    }
    return false;
}

bool OccupancyGrid::updateOneRay(float x0, float y0, float x1, float y1, const int8_t kHitOdds, const int8_t kMissOdds)
{
	bresenham_param_t params;
    int nX0, nY0, nX1, nY1;

    //printf("checking link <%f %f> to <%f %f>\n", x0,y0,x1,y1);
    
    //make sure the line start point is inside the environment
    if( !isCellInGrid(x0, y0) ) {
        return true;
    }

    // nX0 = toInt(x0);
    // nY0 = toInt(y0);
    // nX1 = toInt(x1);
    // nY1 = toInt(y1);
    toInt(x0, y0, nX0, nY0);
    toInt(x1, y1, nX1, nY1);

    //printf("checking link <%d %d> to <%d %d>\n", nX0,nY0,nX1,nY1);

	//iterate through the points on the segment
    if(nX0 == nX1 && nY0 == nY1) {
        hitCell(nX0, nY0, kHitOdds);
    }
    else {
        getBresenhamParameters(nX0, nY0, nX1, nY1, &params);
        do {
        	int nX, nY; 
            getCurrentPoint(&params, &nX, &nY);
            if( !isCellInGrid(nX, nY) ) {
                break;
            }
            if( !(nX == nX1 && nY == nY1) ) {
                // If not the last cell in the ray
                missCell(nX, nY, kMissOdds);
            }
            else {
                // If the last cell in the ray
                hitCell(nX, nY, kHitOdds);
            }
        } while (getNextPoint(&params));
    }
    return false;
}
