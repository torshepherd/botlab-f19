#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>


robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             ObstacleDistanceGrid& map,
                             const SearchParams& params)
{
    ////////////////// TODO: Implement your A* search here //////////////////////////

    bool flagIncludeUninitialized = false;
    int flagHeuristics = 2;
    float weight = 1.0f;
    int closedNum = 0;
    int openNum = 0;
    float pathCost = -1;

    bool flagPath = false;
    bool flagAStar = false;

    robot_path_t path;
    path.utime = start.utime;
    path.path.push_back(start);    

    int startX, startY;
    int goalX, goalY;
    map.toInt(start.x, start.y, startX, startY);
    map.toInt(goal.x, goal.y, goalX, goalY);

    map.clearSearch();
    flagAStar = aStar(startX, startY, goalX, goalY,
                            map, params, flagIncludeUninitialized,
                            flagHeuristics, weight, closedNum, openNum, pathCost);
    if(flagAStar) {
        flagPath = getPath(startX, startY, goalX, goalY, map, path);
    }
    if (!flagPath) {
        path.path.clear();
    }
	else{
	    path.path.push_back(goal);
	}

    path.path_length = path.path.size();
    return path;
}

// float costFunction(const ObstacleDistanceGrid& map, int dir, float distanceToObstacle1, float distanceToObstacle2) {
//     float costDist = map.dCost_[dir];
//     if (distanceToObstacle1 < params.maxDistanceWithCost) {
//         costDist +=  0.5 * powf(params.maxDistanceWithCost - distanceToObstacle1, params.distanceCostExponent);
//     }
//     if (distanceToObstacle2 < params.maxDistanceWithCost) {
//         costDist +=  0.5 * powf(params.maxDistanceWithCost - distanceToObstacle2, params.distanceCostExponent);
//     }
//     return costDist;
// }

float penaltyFunction(const SearchParams& params, float distanceToObstacle1) {
    if (distanceToObstacle1 < params.maxDistanceWithCost) {
        return 0.5 * powf(params.maxDistanceWithCost - distanceToObstacle1, params.distanceCostExponent);
    }
    else {
        return 0.0;
    }
}

float hFunction(float xFloat, float yFloat, float goalXFloat, float goalYFloat, int flagHeuristics) {
    if (flagHeuristics == 2) {
		float SQRT2_2 = -0.58578643762f;
        float dX = fabsf(goalXFloat - xFloat);
        float dY = fabsf(goalYFloat - yFloat);
		float dS;
		if(dX < dY) {
			dS = dX;
		}
		else {
			dS = dY;
		}
		return (dX + dY + SQRT2_2 * dS);
    }
    else if (flagHeuristics == 1) {
        float dX = goalXFloat - xFloat;
        float dY = goalYFloat - yFloat;
        return ( sqrtf(dX*dX + dY*dY) );
    }
	else {
        return 0.0;
	}
}

bool getPath(int startX, int startY, int goalX, int goalY, const ObstacleDistanceGrid& map, robot_path_t& path){

	std::vector<std::vector<int>> pathInt;
	std::vector<int> pathId;

	int currId = map.cellIndex(goalX, goalY);
    int currX = -1;
    int currY = -1;

	while(currId != -1){
		pathId.push_back(currId);
		currId = map.states3D_[currId].parent_;
	}

    int startX1, startY1;
    map.cellXY(pathId.back(), startX1, startY1);
    if ( (startX1 != startX) || (startY1 != startY) ) {
        return false;
    }

    float prevX = path.path[0].x;
    float prevY = path.path[0].y;
    int64_t prevUtime = path.path[0].utime;
    int64_t dUTime = 300000000;
	for(int i = (int) pathId.size() - 1; i >= 0; i--){
        float currXFloat, currYFloat, currAngle;
		currId = pathId[i];
		map.cellXY(currId, currX, currY);
        map.toFloat(currX, currY, currXFloat, currYFloat);
        currAngle = atan2f(currY - prevY, currX - prevX);
        pose_xyt_t currPose;
        currPose.utime =  prevUtime + dUTime;
        currPose.x = currXFloat;
        currPose.y = currYFloat;
        currPose.theta = currAngle;

        path.path.push_back(currPose);
	}
	return true;
}


bool aStar3D(int startX, int startY, int startAngle, int goalX, int goalY, int goalAngle,
            ObstacleDistanceGrid& map, const SearchParams& params, bool flagIncludeUninitialized,
            int flagHeuristics, float weight, int& closedNum, int& openNum, float& pathCost) {

	double initializationTime = 0.0;
	double searchTime = 0.0;
	double heapTime = 0.0;
	if(DO_DEBUG) {
		initializationTime = map.getTime();
	}

	bool flagBreak = false;
	closedNum = 0;
	openNum = 1;
    pathCost = -1;

	VALUE_TYPE goalXFloat = (VALUE_TYPE) goalX;
	VALUE_TYPE goalYFloat = (VALUE_TYPE) goalY;

	// Initialize
    int goalId3D = map.cellIndex(goalX, goalY, goalAngle);
    int goalId = map.cellIndex(goalX, goalY);
    int startId3D = map.cellIndex(startX, startY, startAngle);
    int startId = map.cellIndex(startX, startY);
    int currX = startX;
    int currY = startY;
	int currAngle = startAngle;
    int currId3D = startId3D;
    int currId = startId;
    int nextX, nextY, nextAngle, nextId3D, nextId;

	if( !map.isCellInGrid(startX, startY) || map.isObstacle(startId, params.minDistanceToObstacle, flagIncludeUninitialized) ) {
        if (DO_DEBUG) {
    		printf("Invalid start location!\n");
        }
		return flagBreak;
	}

	if( !map.isCellInGrid(goalX, goalY) || map.isObstacle(goalId, params.minDistanceToObstacle, flagIncludeUninitialized) ) {
        if (DO_DEBUG) {
    		printf("Invalid goal location!\n");
        }
		return flagBreak;
	}

	// if(!hasid(currId3D)) {
		map.states3D_[currId3D] = State(currId3D);
	// }
	map.states3D_[currId3D].g_ = 0.0;
	map.states3D_[currId3D].h_ = 0.0;
    map.states3D_[currId3D].penalty_ = penaltyFunction(params, map(currId));
	HeapNodeType* currNode = new HeapNodeType(currId3D, map.states3D_[currId3D].g_ + weight * map.states3D_[currId3D].h_);
	HeapNodeType* nextNode = nullptr;

	if(DO_DEBUG) {
		initializationTime = map.getTime() - initializationTime;
		searchTime = map.getTime();
	}

	// Search
	while(currNode != nullptr) {
		currId3D = currNode->id_;
		map.cellXYAngle(currId3D, currX, currY, currAngle);

		map.states3D_[currId3D].flagClosed_ = true;
		if(DO_DEBUG) {
			closedNum++;
			openNum--;
		}

        for(int i = 0; i < map.dirNum_; i++) {

			float moveCost = 0.0;
			map.get3DCost(i, currX, currY, currAngle, nextX, nextY, nextAngle, moveCost);

            nextId3D = map.cellIndex(nextX, nextY, nextAngle);
            nextId = map.cellIndex(nextX, nextY);
            
            if( !map.isCellInGrid(nextX, nextY) || map.isObstacle(nextId, params.minDistanceToObstacle, flagIncludeUninitialized) ){
				continue;
			}
			// if(!hasid(nextId)){ // state is unseen
			// 	map.states3D_[nextId] = State(nextId);
			// }
			// else 
			if(map.states3D_[nextId3D].flagClosed_){ // state is in closed
				continue;
			}
            if(map.states3D_[nextId3D].penalty_ < -0.5) {
                map.states3D_[nextId3D].penalty_ = penaltyFunction( params, map(nextId) );
            }
			VALUE_TYPE new_g = map.states3D_[currId3D].g_ + moveCost + map.states3D_[currId3D].penalty_ + map.states3D_[nextId3D].penalty_;
			if( new_g < map.states3D_[nextId3D].g_ ){
				map.states3D_[nextId3D].g_ = new_g;
				map.states3D_[nextId3D].parent_ = currId3D;
				if(map.theHeap3D_->hasId(nextId3D)){ // already in open
					map.theHeap3D_->update(nextId3D, new_g + map.states3D_[nextId].h_ );
				}
				else{
                    // Todo: compute h_ here
					map.states3D_[nextId3D].h_ = weight * hFunction((VALUE_TYPE) nextX, (VALUE_TYPE) nextY, goalXFloat, goalYFloat, flagHeuristics) + map.angleDist(nextAngle, goalAngle);
					nextNode = new HeapNodeType(nextId3D, new_g + map.states3D_[nextId3D].h_ );
					map.theHeap3D_->insert(nextNode);
				    if(DO_DEBUG) {
	                    openNum++;
					}
				}
			}
			if(DO_DEBUG) { heapTime += map.getDuration(); }
		}
		delete currNode;
		if(currId3D == goalId3D){
			flagBreak = true;
            pathCost = map.states3D_[currId3D].g_;
			break;
		}
		currNode = map.theHeap3D_->deleteMin();
	}
    if(DO_DEBUG) {
        printf("Closed: %d, Open: %d, %d\n", closedNum, openNum, map.theHeap3D_->size() );
    }
	if(DO_DEBUG) {
		searchTime = map.getTime() - searchTime;
	}
	// if(DO_DEBUG) {
	// 	markTime();
	// }
	// delete map.theHeap3D_;
	// delete[] map.states3D_;
	// if(DO_DEBUG) {
	// 	printf("delete_time = %f\n", getDuration());
	// }

	// if(DO_DEBUG) {
		// printf("initializationTime = %f, searchTime = %f, otherTime1 = %f, otherTime2 = %f, heapTime = %f\n", initializationTime, searchTime, otherTime1, otherTime2, heapTime);
	// }

	return flagBreak;
}
