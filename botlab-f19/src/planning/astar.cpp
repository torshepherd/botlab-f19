#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <algorithm>
#include <queue>
#include <vector>


robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             ObstacleDistanceGrid& map,
                             const SearchParams& params, bool flagPushGoal)
{
    ////////////////// TODO: Implement your A* search here //////////////////////////

    bool flagIncludeUninitialized = false;
    int flagHeuristics = 2;
    float weight = 1.0f;
    int closedNum = 0;
    int openNum = 0;
    float pathCost = -1;

	bool flag3D = true;
	bool flagRemoveRedundantPoints = true;

	bool flagIgnoreGoalAngle = true;

    bool flagPath = false;
    bool flagAStar = false;

    robot_path_t path;
    path.utime = start.utime;
    path.path.push_back(start);    

    int startX, startY, startAngle;
    int goalX, goalY, goalAngle;


	if (flag3D) {
		map.toInt(start.x, start.y, start.theta, startX, startY, startAngle);
		map.toInt(goal.x, goal.y, goal.theta, goalX, goalY, goalAngle);

		map.clearSearch3D();
		flagAStar = aStar3D(startX, startY, startAngle, goalX, goalY, goalAngle,
								map, params, flagIncludeUninitialized,
								flagHeuristics, weight, closedNum, openNum, pathCost);
		if(flagAStar) {
			flagPath = getPath3D(startX, startY, startAngle, goalX, goalY, goalAngle, map, path, flagRemoveRedundantPoints);
		}

	}
	else {
		map.toInt(start.x, start.y, startX, startY);
		map.toInt(goal.x, goal.y, goalX, goalY);

		map.clearSearch();
		flagAStar = aStar(startX, startY, goalX, goalY,
								map, params, flagIncludeUninitialized,
								flagHeuristics, weight, closedNum, openNum, pathCost);
		if(flagAStar) {
			flagPath = getPath(startX, startY, goalX, goalY, map, path, flagRemoveRedundantPoints);
		}
	}

    if(DO_DEBUG) {
        printf("Closed: %d, Open: %d\n", closedNum, openNum );
    }

    if (!flagPath) {
        path.path.clear();
    }
	else{
		if (flagPushGoal) {
		    path.path.push_back(goal);
			if (flagIgnoreGoalAngle && (path.path.size() >= 2) ) {
				size_t lastMinus1 = path.path.size() - 2;
				path.path.back().theta = atan2f(path.path.back().y - path.path[lastMinus1].y, path.path.back().x - path.path[lastMinus1].x);
			}
		}
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

bool getPath(int startX, int startY, int goalX, int goalY, const ObstacleDistanceGrid& map, robot_path_t& path, bool flagRemoveRedundantPoints){

	std::vector<std::vector<int>> pathInt;
	std::vector<int> pathId;

	int currId = map.cellIndex(goalX, goalY);
    int currX = -1;
    int currY = -1;

	while(currId != -1){
		pathId.push_back(currId);
		currId = map.states_[currId].parent_;
	}

    int startX1, startY1;
    map.cellXY(pathId.back(), startX1, startY1);
    if ( (startX1 != startX) || (startY1 != startY) ) {
        return false;
    }

	int prevDX = -100;
	int prevDY = -100;

    float prevXFloat = path.path[0].x;
    float prevYFloat = path.path[0].y;
    int64_t currUtime = path.path[0].utime;
    int64_t dUTime = 300000000;
	for(int i = (int) pathId.size() - 1; i >= 0; i--){
		currUtime += dUTime;

        float currXFloat, currYFloat, currAngle;
		currId = pathId[i];
		map.cellXY(currId, currX, currY);
		if (flagRemoveRedundantPoints) {
			if(i > 0) {
				int nextId, nextX, nextY;
				nextId = pathId[i-1];
				map.cellXY(nextId, nextX, nextY);
				int currDX = nextX - currX;
				int currDY = nextY - currY;
				if( (currDX == prevDX) && (currDY == prevDY) ) {
					continue;
				}
				prevDX = currDX;
				prevDY = currDY;
			}
		}

        map.toFloat(currX, currY, currXFloat, currYFloat);
        currAngle = atan2f(currYFloat - prevYFloat, currXFloat - prevXFloat);
		prevXFloat = currXFloat;
		prevYFloat = currYFloat;

        pose_xyt_t currPose;
        currPose.utime =  currUtime + dUTime;
        currPose.x = currXFloat;
        currPose.y = currYFloat;
        currPose.theta = currAngle;

        path.path.push_back(currPose);
	}
	return true;
}

int getNodeDist(int x1, int y1, int x2, int y2) {
	int dx = x1 - x2;
	if(dx < 0) {
		dx = -dx;
	}
	int dy = y1 - y2;
	if(dy < 0) {
		dy = -dy;
	}
	if(dx > dy) {
		return dx;
	}
	else {
		return dy;
	}
}

bool getPath3D(int startX, int startY, int startAngle, int goalX, int goalY, int goalAngle, const ObstacleDistanceGrid& map, robot_path_t& path, bool flagRemoveRedundantPoints){

	std::vector<std::vector<int>> pathInt;
	std::vector<int> pathId3D;

	int currId3D = map.cellIndex(goalX, goalY, goalAngle);
    int currX = -1;
    int currY = -1;
    int currAngle = -1;

	while(currId3D != -1){
		pathId3D.push_back(currId3D);
		currId3D = map.states3D_[currId3D].parent_;
	}

    int startX1, startY1, startAngle1;
    map.cellXYAngle(pathId3D.back(), startX1, startY1, startAngle1);
    if ( (startX1 != startX) || (startY1 != startY) || (startAngle1 != startAngle) ) {
        return false;
    }

    // float prevX = path.path[0].x;
    // float prevY = path.path[0].y;

	int prevAngle = -100;

    int64_t currUtime = path.path[0].utime;
    int64_t dUTime = 300000000;
	for(int i = (int) pathId3D.size() - 1; i > 0; i--){
		currUtime += dUTime;

        float currXFloat, currYFloat, currAngleFloat;
		currId3D = pathId3D[i];
		map.cellXYAngle(currId3D, currX, currY, currAngle);

		if (flagRemoveRedundantPoints) {
			if(i > 0 && (i <= (int) pathId3D.size() - 1) ) {
				if( prevAngle == currAngle ) {
					continue;
				}
				if( getNodeDist(currX, currY, startX, startY) <= 3 ) {
					continue;
				}
				if ( (i < (int) pathId3D.size() - 1) ) {
					int prevId3D, prevX, prevY;
					float prevXFloat, prevYFloat, prevAngleFloat;
					prevId3D = pathId3D[i+1];
					map.cellXYAngle(prevId3D, prevX, prevY, prevAngle);
					map.toFloat(prevX, prevY, prevAngle, prevXFloat, prevYFloat, prevAngleFloat);

					pose_xyt_t prevPose;
					prevPose.utime =  currUtime - dUTime;
					prevPose.x = prevXFloat;
					prevPose.y = prevYFloat;
					prevPose.theta = prevAngleFloat;

					path.path.push_back(prevPose);
				}
			}
			else {
				map.toFloat(currX, currY, currAngle, currXFloat, currYFloat, currAngleFloat);
				pose_xyt_t currPose;
				currPose.utime =  currUtime;
				currPose.x = currXFloat;
				currPose.y = currYFloat;
				currPose.theta = currAngleFloat;

				path.path.push_back(currPose);
			}
			prevAngle = currAngle;
		}
		else {
			map.toFloat(currX, currY, currAngle, currXFloat, currYFloat, currAngleFloat);

			pose_xyt_t currPose;
			currPose.utime =  currUtime;
			currPose.x = currXFloat;
			currPose.y = currYFloat;
			currPose.theta = currAngleFloat;

			path.path.push_back(currPose);
		}
	}
	return true;
}


bool aStar(int startX, int startY, int goalX, int goalY,
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

	float goalXFloat = 0.0;
	float goalYFloat = 0.0;
	map.toFloat(goalX, goalY, goalXFloat, goalYFloat);

	// Initialize
    int goalId = map.cellIndex(goalX, goalY);
    int startId = map.cellIndex(startX, startY);
    int currX = startX;
    int currY = startY;
    int currId = startId;
    int nextX, nextY, nextId;

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

	// if(!hasid(currId)) {
		map.states_[currId] = State(currId);
	// }
	map.states_[currId].g_ = 0.0;
	map.states_[currId].h_ = 0.0;
    map.states_[currId].penalty_ = penaltyFunction(params, map(currId));
	HeapNodeType* currNode = new HeapNodeType(currId, map.states_[currId].g_ + weight * map.states_[currId].h_);
	HeapNodeType* nextNode = nullptr;

	if(DO_DEBUG) {
		initializationTime = map.getTime() - initializationTime;
		searchTime = map.getTime();
	}

	// Search
	while(currNode != nullptr) {
		currId = currNode->id_;
        map.cellXY(currId, currX, currY);

		map.states_[currId].flagClosed_ = true;
		if(DO_DEBUG) {
			closedNum++;
			openNum--;
		}

        for(int i = 0; i < map.dirNum_; i++) {

            nextX = currX + map.dX_[i];
            nextY = currY + map.dY_[i];
            nextId = map.cellIndex(nextX, nextY);
            
            if( !map.isCellInGrid(nextX, nextY) || map.isObstacle(nextId, params.minDistanceToObstacle, flagIncludeUninitialized) ){
				continue;
			}
			// if(!hasid(nextId)){ // state is unseen
			// 	map.states_[nextId] = State(nextId);
			// }
			// else 
			// if(map.states_[nextId].flagClosed_){ // state is in closed
			// 	continue;
			// }
            if(map.states_[nextId].penalty_ < -0.5) {
                map.states_[nextId].penalty_ = penaltyFunction( params, map(nextId) );
            }
			VALUE_TYPE new_g = map.states_[currId].g_ + map.dCost_[i] + map.states_[currId].penalty_ + map.states_[nextId].penalty_;
			if( new_g < map.states_[nextId].g_ ){
				map.states_[nextId].g_ = new_g;
				map.states_[nextId].parent_ = currId;
				if(map.states_[nextId].flagClosed_){ // state is in closed
					continue;
				}
				if(map.theHeap_->hasId(nextId)){ // already in open
					map.theHeap_->update(nextId, new_g + map.states_[nextId].h_ );
				}
				else{
                    // Todo: compute h_ here
					float nextXFloat = 0.0;
					float nextYFloat = 0.0;
					map.toFloat(nextX, nextY, nextXFloat, nextYFloat);

					map.states_[nextId].h_ = weight * hFunction(nextXFloat, nextYFloat, goalXFloat, goalYFloat, flagHeuristics);
					nextNode = new HeapNodeType(nextId, new_g + map.states_[nextId].h_ );
					map.theHeap_->insert(nextNode);
				    if(DO_DEBUG) {
	                    openNum++;
					}
				}
			}
			if(DO_DEBUG) { heapTime += map.getDuration(); }
		}
		delete currNode;
		if(currId == goalId){
			flagBreak = true;
            pathCost = map.states_[currId].g_;
			break;
		}
		currNode = map.theHeap_->deleteMin();
	}
    if(DO_DEBUG) {
        // printf("Closed: %d, Open: %d, %d\n", closedNum, openNum, map.theHeap_->size() );
    }
	if(DO_DEBUG) {
		searchTime = map.getTime() - searchTime;
	}
	// if(DO_DEBUG) {
	// 	markTime();
	// }
	// delete map.theHeap_;
	// delete[] map.states_;
	// if(DO_DEBUG) {
	// 	printf("delete_time = %f\n", getDuration());
	// }

	// if(DO_DEBUG) {
		// printf("initializationTime = %f, searchTime = %f, otherTime1 = %f, otherTime2 = %f, heapTime = %f\n", initializationTime, searchTime, otherTime1, otherTime2, heapTime);
	// }

	return flagBreak;
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

	float goalXFloat = 0.0;
	float goalYFloat = 0.0;
	map.toFloat(goalX, goalY, goalXFloat, goalYFloat);

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
		printf("params.minDistanceToObstacle = %f\n", params.minDistanceToObstacle);
		printf("(x, y, distance) = (%d, %d", startX, startY);
		if(map.isCellInGrid(startX, startY)) {
			printf("%f\n", map(startId) );
		}
		else{
			printf("%f\n", -101.0 );
		}

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
			// if(map.states3D_[nextId3D].flagClosed_){ // state is in closed
			// 	continue;
			// }
            if(map.states3D_[nextId3D].penalty_ < -0.5) {
                map.states3D_[nextId3D].penalty_ = penaltyFunction( params, map(nextId) );
            }
			VALUE_TYPE new_g = map.states3D_[currId3D].g_ + moveCost + map.states3D_[currId3D].penalty_ + map.states3D_[nextId3D].penalty_;
			if( new_g < map.states3D_[nextId3D].g_ ){
				map.states3D_[nextId3D].g_ = new_g;
				map.states3D_[nextId3D].parent_ = currId3D;
				if(map.states3D_[nextId3D].flagClosed_){ // state is in closed
					continue;
				}
				if(map.theHeap3D_->hasId(nextId3D)){ // already in open
					map.theHeap3D_->update(nextId3D, new_g + map.states3D_[nextId].h_ );
				}
				else{
                    // Todo: compute h_ here
					float nextXFloat = 0.0;
					float nextYFloat = 0.0;
					map.toFloat(nextX, nextY, nextXFloat, nextYFloat);

					map.states3D_[nextId3D].h_ = weight * (hFunction(nextXFloat, nextYFloat, goalXFloat, goalYFloat, flagHeuristics)
															 + map.angleDist(nextAngle, goalAngle) );
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
        // printf("Closed: %d, Open: %d, %d\n", closedNum, openNum, map.theHeap3D_->size() );
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

int getFrontier(const ObstacleDistanceGrid& map, const SearchParams& params, const pose_xyt_t& start) {
	int nearestFrontierId = -1;
	bool flagIncludeUninitialized = false;

    std::queue<int> openSet;
	std::vector<int> searchMap;
	searchMap.resize(map.size());
    std::fill(searchMap.begin(), searchMap.end(), -1); // Initial value

    int startX, startY;
	map.toInt(start.x, start.y, startX, startY);
	int startId = map.cellIndex(startX, startY);
	searchMap[startId] = 0;

	openSet.push(startId);

    // If obstacles in the map
    // Do breadth first search to ditermine distance
    while ( !openSet.empty() ) {
        int currId = openSet.front();
        openSet.pop();
        int currX, currY;
        map.cellXY(currId, currX, currY);
        for (int i = 0; i < map.dirNum_; i++) {
            int nextX = currX + map.dX_[i];
            int nextY = currY + map.dY_[i];
            if (!map.isCellInGrid(nextX, nextY)) {
                continue;
            }
            int nextId = map.cellIndex(nextX, nextY);
			if (map.isUninitialized(nextId) ) {
				nearestFrontierId = currId;
				break;
			}
			else if (map.isObstacle(nextId, params.minDistanceToObstacle, flagIncludeUninitialized) ) {
				continue;
			}
            else {
				if(searchMap[nextId] == -1) {
					searchMap[nextId] = searchMap[currId] + 1;
					openSet.push(nextId);
				}
            }
        }
    }

	return nearestFrontierId;
}

robot_path_t pathToFreeSpace(const pose_xyt_t& start, 
                             const ObstacleDistanceGrid& map,
                             const SearchParams& params)
{
    ////////////////// TODO: Implement your A* search here //////////////////////////

    robot_path_t path;
    path.utime = start.utime;

	int goalId = getFreeSpace(map, params, start);
	if(goalId == -1) {
	    path.path_length = path.path.size();
		return path;
	}

    path.path.push_back(start);


    int goalX, goalY;
	float goalXFloat, goalYFloat;
	map.cellXY(goalId, goalX, goalY);
	map.toFloat(goalX, goalY, goalXFloat, goalYFloat);

	pose_xyt_t goal;
	goal.utime = start.utime;
	goal.x = goalXFloat;
	goal.y = goalYFloat;
	goal.theta = atan2f(goal.y - start.y, goal.x - start.x);
	
	path.path.push_back(goal);

    path.path_length = path.path.size();
    return path;
}

int getFreeSpace(const ObstacleDistanceGrid& map, const SearchParams& params, const pose_xyt_t& start) {

	bool flagIncludeUninitializedTrue = true;

	int startX, startY;
	map.toInt(start.x, start.y, startX, startY);

	int goalId = -1;
	float goalDist = -2.0;

	int searchBlockSize = 3;
	for(int i = -searchBlockSize ; i < searchBlockSize ; i++) {
		for(int j = -searchBlockSize ; j < searchBlockSize ; j++) {
			int currX = startX + i;
			int currY = startY + j;
			int currId = map.cellIndex(currX, currY);
			if(!map.isCellInGrid(currX, currY)) {
				continue;
			}
			if (map.isObstacle(currId, params.minDistanceToObstacle, flagIncludeUninitializedTrue) ) {
				continue;
			}
			if(map(currId) > goalDist) {
				goalDist = map(currId);
				goalId = currId;
			}
		}
	}

	printf("goalId haha = %d\n", goalId);

	return goalId;

}

// int getFreeSpace(const ObstacleDistanceGrid& map, const SearchParams& params, const pose_xyt_t& start) {
// 	int nearestFreeSpaceId = -1;
// 	bool flagIncludeUninitializedFalse = false;
// 	bool flagIncludeUninitializedTrue = true;

//     std::queue<int> openSet;
// 	std::vector<int> searchMap;
// 	searchMap.resize(map.size());
//     std::fill(searchMap.begin(), searchMap.end(), -1); // Initial value

//     int startX, startY;
// 	map.toInt(start.x, start.y, startX, startY);
// 	int startId = map.cellIndex(startX, startY);
// 	searchMap[startId] = 0;

// 	openSet.push(startId);

//     // If obstacles in the map
//     // Do breadth first search to ditermine distance
//     while ( !openSet.empty() ) {
//         int currId = openSet.front();
//         openSet.pop();
//         int currX, currY;
//         map.cellXY(currId, currX, currY);
//         for (int i = 0; i < map.dirNum_; i++) {
//             int nextX = currX + map.dX_[i];
//             int nextY = currY + map.dY_[i];
//             if (!map.isCellInGrid(nextX, nextY)) {
//                 continue;
//             }
//             int nextId = map.cellIndex(nextX, nextY);
// 			if (!map.isObstacle(nextId, params.minDistanceToObstacle, flagIncludeUninitializedTrue) ) {
// 				nearestFreeSpaceId = nextId;
// 				break;
// 			}
//             else {
// 				if(searchMap[nextId] == -1) {
// 					searchMap[nextId] = searchMap[currId] + 1;
// 					openSet.push(nextId);
// 				}
//             }
//         }
//     }

// 	return nearestFreeSpaceId;
// }
