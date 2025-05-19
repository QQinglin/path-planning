#include "Astar_searcher.h"

using namespace std;
using namespace Eigen;

// setting up the grid map, including defining the grid's dimensions, resolution, and initializing the grid nodes.
void AstarPathFinder::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id)
{   
    // set the lower and upper bounds of the gird
    gl_xl = global_xyz_l(0);
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);

    gl_xu = global_xyz_u(0);
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);
    
    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;
    GLYZ_SIZE  = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE; // total number of cells in the 3D grid.

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution; // converting coordinates to grid indices.    

    data = new uint8_t[GLXYZ_SIZE]; // 1D array representing the occupancy state of each cell
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t)); // all cells are initially free (unoccupied).
    
    GridNodeMap = new GridNodePtr ** [GLX_SIZE];
    for(int i = 0; i < GLX_SIZE; i++){
        GridNodeMap[i] = new GridNodePtr * [GLY_SIZE];
        for(int j = 0; j < GLY_SIZE; j++){
            GridNodeMap[i][j] = new GridNodePtr [GLZ_SIZE];
            for( int k = 0; k < GLZ_SIZE;k++){
                Vector3i tmpIdx(i,j,k); // creates a 3D index for the current grid cell
                Vector3d pos = gridIndex2coord(tmpIdx); // converts the grid index to world coordinates
                GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos); // GridNodeMap stores structure pointer
            }
        }
    }
}

void AstarPathFinder::resetGrid(GridNodePtr ptr)
{
    // Initialize all Map Grids
    ptr->id = 0;
    ptr->cameFrom = NULL;
    ptr->gScore = inf;
    ptr->fScore = inf;
}

void AstarPathFinder::resetUsedGrids()
{   
    for(int i=0; i < GLX_SIZE ; i++)
        for(int j=0; j < GLY_SIZE ; j++)
            for(int k=0; k < GLZ_SIZE ; k++)
                resetGrid(GridNodeMap[i][j][k]); // reset all nodes in the 3D grid map to their initial state
}

void AstarPathFinder::setObs(const double coord_x, const double coord_y, const double coord_z)
{
    if( coord_x < gl_xl  || coord_y < gl_yl  || coord_z <  gl_zl || 
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu )
        return;

    int idx_x = static_cast<int>( (coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>( (coord_y - gl_yl) * inv_resolution);
    int idx_z = static_cast<int>( (coord_z - gl_zl) * inv_resolution);      

    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
}

vector<Vector3d> AstarPathFinder::getVisitedNodes()
{   
    vector<Vector3d> visited_nodes;
    for(int i = 0; i < GLX_SIZE; i++)
        for(int j = 0; j < GLY_SIZE; j++)
            for(int k = 0; k < GLZ_SIZE; k++){   
                // id == 0: The node is unvisited; id == 1: The node is in the open list
                // id == -1: The node is in the closed list. it has already been expanded.
                if(GridNodeMap[i][j][k]->id == -1)  // visualize nodes in closed list only
                    visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
            }
    ROS_WARN("visited_nodes size : %d", visited_nodes.size());
    return visited_nodes;
}

Vector3d AstarPathFinder::gridIndex2coord(const Vector3i & index) 
{
    Vector3d pt;

    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

    return pt;
}

Vector3i AstarPathFinder::coord2gridIndex(const Vector3d & pt) 
{
    Vector3i idx;
    idx <<  min( max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
            min( max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
            min( max( int( (pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);                  
  
    return idx;
}

Eigen::Vector3d AstarPathFinder::coordRounding(const Eigen::Vector3d & coord)
{
    return gridIndex2coord(coord2gridIndex(coord));
}

inline bool AstarPathFinder::isOccupied(const Eigen::Vector3i & index) const
{
    return isOccupied(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isFree(const Eigen::Vector3i & index) const
{
    return isFree(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return  (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

inline bool AstarPathFinder::isFree(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
           (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr, vector<GridNodePtr> & neighborPtrSets, vector<double> & edgeCostSets)
{   
    neighborPtrSets.clear();
    edgeCostSets.clear();
    /*
    *
    STEP 4: finish AstarPathFinder::AstarGetSucc yourself 
    please write your code below
    *
    *
    */
   GridNodePtr nodePtr;
   vector3i center = currentPtr->index; // get the index of current node
   
   // iterate over all possible neighbors in a 3x3x3 cube centered around the current node, excluding the current node itself.
   for(int dev_X = -1;dev_X <= 1;dev_X++){
        if(center[0]+dev_X>=0 && center[0]+dev_X<= GLX_SIZE){ // prevents accessing indices that are beyond the upper bound of the grid.
            for(int dev_Y = -1;dev_Y <= 1;dev_Y++){
                if(center[1]+dev_Y>=0 && center[1]+dev_Y<= GLY_SIZE){ // prevents accessing indices that are beyond the upper bound of the grid.
                    for(int dev_Z = -1;dev_Z <= 1;dev_Z++){
                        if(center[2]+dev_Z>=0 && center[2]+dev_Z<= GLZ_SIZE){ // prevents accessing indices that are beyond the upper bound of the grid.
                            nodePtr = GridNodeMap[center[0]+dev_X][center[1]+dev_Y][center[2]+dev_Z]; // get the pointer of neightbor node
                            if(isOccupied(nodePtr->index) || nodePtr->id == -1)
                                continue;
                            else{
                                neighborPtrSets.push_back(nodePtr); // add the valid neighboring node in the vector
                                
                                // calculate the cost from current node to neighbor nodes, Euclidean distance
                                edgeCostSets.push_back(
                                    sqrt(
                                    (nodePtr->index(0) - currentPtr->index(0)) * (nodePtr->index(0) - currentPtr->index(0)) + // Difference in the X coordinates. 
                                    (nodePtr->index(1) - currentPtr->index(1)) * (nodePtr->index(1) - currentPtr->index(1)) + // Difference in the Y coordinates.
                                    (nodePtr->index(2) - currentPtr->index(2)) * (nodePtr->index(2) - currentPtr->index(2)))  // Difference in the Z coordinates. 
                                    );
                            }
                        }
                    }
                }
            }
        }
    }
}

double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2)
{
    /* 
    choose possible heuristic function you want
    Manhattan, Euclidean, Diagonal, or 0 (Dijkstra)
    Remember tie_breaker learned in lecture, add it here ?
    *
    *
    *
    STEP 1: finish the AstarPathFinder::getHeu , which is the heuristic function
    please write your code below
    *
    *
    */

   double Euclidean_dis = 0;

   // node 2 is the goal node, node 1 is current node
   Euclidean_dis = sqrt(
    (node1->index(0) - node2->index(0)) * (node1->index(0) - node2->index(0)) +
    (node1->index(1) - node2->index(1)) * (node1->index(1) - node2->index(1)) +
    (node1->index(2) - node2->index(2)) * (node1->index(2) - node2->index(2)) +
    );

    // Apply tie-breaker to avoid exploring nodes with the same fScore
    // The tie breaker helps to prefer nodes that are closer to the goal
    double tie_breaker = 1.0 + 1.0/10000;
    Euclidean_dis *= tie_breaker;

    return Euclidean_dis;
}

void AstarPathFinder::AstarGraphSearch(Vector3d start_pt, Vector3d end_pt)
{   
    ros::Time time_1 = ros::Time::now();    

    //index of start_point and end_point
    Vector3i start_idx = coord2gridIndex(start_pt);
    Vector3i end_idx   = coord2gridIndex(end_pt);
    goalIdx = end_idx;

    //position of start_point and end_point
    start_pt = gridIndex2coord(start_idx);
    end_pt   = gridIndex2coord(end_idx);

    //Initialize the pointers of struct GridNode which represent start node and goal node
    GridNodePtr startPtr = new GridNode(start_idx, start_pt);
    GridNodePtr endPtr   = new GridNode(end_idx,   end_pt);

    //openSet is the open_list implemented through multimap in STL library
    openSet.clear();
    // currentPtr represents the node with lowest f(n) in the open_list
    GridNodePtr currentPtr  = NULL;
    GridNodePtr neighborPtr = NULL;

    startPtr -> gScore = 0;
    startPtr -> fScore = getHeu(startPtr,endPtr);  
    startPtr -> id = 1; 
    startPtr -> coord = start_pt;
    // startPtr->fScore as the first element (key) and startPtr as the second element (value)
    openSet.insert(make_pair(startPtr -> fScore, startPtr));

    /*
    *
    STEP 2 :  some else preparatory works which should be done before while loop
    please write your code below
    *
    *
    */
    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;
    // Initialize the closed set
    set<GridNodePtr> closeSet;

    // this is the main loop
    while ( !openSet.empty() ){
        /*
        *
        *
        step 3: Remove the node with lowest cost function from open set to closed set
        please write your code below
        
        IMPORTANT NOTE!!!
        This part you should use the C++ STL: multimap, more details can be find in Homework description
        *
        *
        */
        auto it = openSet.begin(); // Iterator to the node with the lowest fScore
        currentPtr = it->second; // // Get the GridNodePtr from the iterator
        // it->first): Represents the fScore of the node ; it->second): Represents a pointer to the GridNode
        // properties of GridNode include id ; gScore; fScore; Vector31 index(x,y,z) ;Vector3d coord(coordinate in space)
        // And GridNode* cameFrom (Pointer to the parent node)
        currentPtr -> id = -1; //Mark the node as processed (moved to closed set)
        // multimap openSet keeps nodes sorted by their fScore automaticall
        closeSet.insert(make_pair(currentPtr -> first, currentPtr));
        openSet.erase(it);

        // if the current node is the goal 
        if( currentPtr->index == goalIdx ){
            ros::Time time_2 = ros::Time::now();
            terminatePtr = currentPtr;
            ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );            
            return;
        }
        //get the succetion
        //STEP 4: finish AstarPathFinder::AstarGetSucc yourself 
        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);      

        /*
        *
        *
        STEP 5:  For all unexpanded neigbors "m" of node "n", please finish this for loop
        please write your code below
        *        
        */         
        for(int i = 0; i < (int)neighborPtrSets.size(); i++){
            /*
            *
            *
            Judge if the neigbors have been expanded
            please write your code below
            
            IMPORTANT NOTE!!!
            neighborPtrSets[i]->id = -1 : expanded, equal to this node is in close set
            neighborPtrSets[i]->id = 1 : unexpanded, equal to this node is in open set
            *        
            */
            if(neighborPtr -> id == 0){ //discover a new node, which is not in the closed set and open set
                /*
                *
                *
                STEP 6:  As for a new node, do what you need do ,and then put neighbor in open set and record it
                please write your code below
                *        
                */
               neighborPtr->cameFrom = currentPtr;

               neighborPtr->gScore = getHeu(neighborPtr,currentPtr) + currentPtr->gScore;
               neighborPtr->fScore = getHeu(neighborPtr,endPtr) + neighborPtr->gScore;

               neighborPtr->id = 1;
               neighborPtr-> coord gridIndex2coord(neighborPtr->index);
               openSet.insert(make_pair(neighborPtr->fScore,neighborPtr));
                continue;
            }
            else if(neighborPtr->id == 1){ //this node is in open set and need to judge if it needs to update, the "0" should be deleted when you are coding
                /*
                *
                *
                STEP 7:  As for a node in open set, update it , maintain the openset ,and then put neighbor in open set and record it
                please write your code below
                *        
                */
               int temp_geScore = getHeu(neighborPtr,currentPtr);
               if((temp_geScore + currentPtr->fScore) < neighborPtr->fScore){
                neighborPtr->cameFrom = currentPtr;
                neighborPtr->gScore = temp_geScore + currentPtr->gScore;
                neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr,endPtr);
                if (neighborPtr != endPtr){
                // returns a pair of iterators (pr.first and pr.second) representing the range of elements with a specific key in the multimap
                //This range includes all elements with keys equal to the specified key,
                auto pr = openSet.equal_range(neighborPtr->fScore); 
                 for (auto it = pr.first; it != pr.second;++it ){
                    if(it ->second.index == neighborPtr->index){
                        openSet.erase(it); // openSet is a priority queue. Key is fScore, Value is a pointer
                        openSet.insert(make_pair(neighborPtr->fScore,neighborPtr));
                    }
                }
               }
               }
                continue;
            }
            else{//this node is in closed set
                /*
                *
                please write your code below
                *        
                */

                continue;
            }
        }      
    }
    //if search fails
    ros::Time time_2 = ros::Time::now();
    if((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in Astar path finding is %f", (time_2 - time_1).toSec() );
}


vector<Vector3d> AstarPathFinder::getPath() 
{   
    vector<Vector3d> path;
    vector<GridNodePtr> gridPath;
    /*
    *
    *
    STEP 8:  trace back from the curretnt nodePtr to get all nodes along the path
    please write your code below
    *      
    */
   GridNodePtr tempPtr = terminatePtr;
   while (tempPtr != 0)
   {
    gridPath.push_back(tempPtr->cameFrom);
    tempPtr = tempPtr->cameFrom;
   }
   
    for (auto ptr: gridPath)
        path.push_back(ptr->coord);
        
    reverse(path.begin(),path.end());

    return path;
}