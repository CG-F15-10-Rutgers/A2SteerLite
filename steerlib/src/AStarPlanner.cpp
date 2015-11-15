//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#include <vector>
#include <stack>
#include <set>
#include <map>
#include <iostream>
#include <algorithm>
#include <functional>
#include <queue>
#include <math.h>
#include "planning/AStarPlanner.h"


#define COLLISION_COST  1000
#define GRID_STEP  1
#define OBSTACLE_CLEARANCE 1
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))

namespace SteerLib
{
	AStarPlanner::AStarPlanner(){}

	AStarPlanner::~AStarPlanner(){}

	bool AStarPlanner::canBeTraversed ( int id )
	{
		double traversal_cost = 0;
		int current_id = id;
		unsigned int x,z;
		gSpatialDatabase->getGridCoordinatesFromIndex(current_id, x, z);
		int x_range_min, x_range_max, z_range_min, z_range_max;

		x_range_min = MAX(x-OBSTACLE_CLEARANCE, 0);
		x_range_max = MIN(x+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsX());

		z_range_min = MAX(z-OBSTACLE_CLEARANCE, 0);
		z_range_max = MIN(z+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsZ());


		for (int i = x_range_min; i<=x_range_max; i+=GRID_STEP)
		{
			for (int j = z_range_min; j<=z_range_max; j+=GRID_STEP)
			{
				int index = gSpatialDatabase->getCellIndexFromGridCoords( i, j );
				traversal_cost += gSpatialDatabase->getTraversalCost ( index );

			}
		}

		if ( traversal_cost > COLLISION_COST )
			return false;
		return true;
	}



	Util::Point AStarPlanner::getPointFromGridIndex(int id)
	{
		Util::Point p;
		gSpatialDatabase->getLocationFromIndex(id, p);
		return p;
	}


	double AStarPlanner::manhattan_dist(Util::Point n, Util::Point g)
	{
		return fabs(n.x - g.x) + fabs(n.z - g.z);
	}

	double AStarPlanner::euclidean_dist(Util::Point n, Util::Point g)
	{
		return sqrt(pow(n.x - g.x, 2) + pow(n.z - g.z, 2));
	}

	/* get index of node in list with the lowest f value */
	int AStarPlanner::get_lowest_f(std::vector<AStarPlannerNode> vec)
	{
		AStarPlannerNode *l = &vec[0];
		int ind = 0;

		for (int i = 1; i < vec.size(); i++) {
			if (vec[i].f < l->f) {
				l = &vec[i];
				ind = i;
			} else if (vec[i].f == l->f) {
				/* CHANGE ME: change this for assignment submission */
				if (vec[i].g > l->g)
					l = &vec[i];
			}
		}
		return ind;
	}

	/* remove cur from open, add it to closed. returns the new index of cur
	 * in closed */
	int AStarPlanner::close_node(std::vector<AStarPlannerNode>& open, std::vector<AStarPlannerNode>& closed, int ind)
	{
		closed.push_back(open[ind]);
		open.erase(open.begin()+ind);
		return closed.size()-1;
	}

	/* get all nodes adjecent to the current node */
	std::vector<AStarPlannerNode> AStarPlanner::get_neighbors(SteerLib::GridDatabase2D *grid, std::vector<AStarPlannerNode> closed, int ind)
	{
		std::vector<AStarPlannerNode> nbrs;
		double diag_cost;

		/* CHANGE ME: change this for assignment submission
		 * change to 1.414 */
		diag_cost = 1.0f;
		for (int x = -1; x <= 1; x++) {
			float xpos = closed[ind].point.x + x*grid->getCellSizeX();
			if (xpos < grid->getOriginX() || xpos > (grid->getOriginX()+grid->getGridSizeX()))
				continue;
			for (int z = -1; z <= 1; z++) {
				if (x == 0 && z == 0)
					continue;
				float zpos = closed[ind].point.z + z*grid->getCellSizeZ();
				if (zpos < grid->getOriginZ() || zpos > (grid->getOriginZ()+grid->getGridSizeZ()))
					continue;
				int grid_ind = grid->getCellIndexFromLocation(xpos, zpos);
				if (canBeTraversed(grid_ind)) {
					AStarPlannerNode nbr;
					nbr.point = {xpos, 0.0f, zpos};
					if (x != 0 || z != 0)
						nbr.g = closed[ind].g + diag_cost;
					else
						nbr.g = closed[ind].g + 1.0f;
					nbrs.push_back(nbr);
				}
			}
		}
		return nbrs;
	}

	/* if vec contains n, returns the node's index. otherwise returns -1 */
	int AStarPlanner::find_node(std::vector<AStarPlannerNode> vec, AStarPlannerNode n)
	{
		for (int i = 0; i < vec.size(); i++) {
			if (vec[i] == n) {
				return i;
			}
		}
		return -1;
	}

	int AStarPlanner::recreate_path(std::vector<Util::Point>& path, std::vector<AStarPlannerNode> closed, int ind)
	{
		int len = 0, start, end;
		AStarPlannerNode *n = NULL;
		for (; ind != -1; ind = n->parent_ind) {
			n = &closed[ind];
			path.push_back(n->point);
			len++;
		}
		start = 0;
		end = path.size()-1;
		while(start < end) {
			Util::Point tmp = path[end];
			path[end] = path[start];
			path[start] = tmp;
			start++;
			end--;
		}

		return len;
	}

	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path,  Util::Point start, Util::Point goal, SteerLib::GridDatabase2D * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;
		//std::vector<AStarPlannerNode> nodes;
		std::vector<AStarPlannerNode> open;
		std::vector<AStarPlannerNode> closed;
		AStarPlannerNode startn;
		AStarPlannerNode goaln;
		double heuristic_wt;
		int num_expanded, sol_len;

		/* CHANGE ME: change this for assignment submission */
		heuristic_wt = 2.0f;
		num_expanded = 1;
		startn.point = start;
		startn.parent_ind = -1;
		/* CHANGE ME: change this for assignment submission */
		startn.f = manhattan_dist(start, goal) * heuristic_wt;
		goaln.point = goal;
		open.push_back(startn);

		while (open.size() > 0) {
			int open_ind, closed_ind;
			std::vector<AStarPlannerNode> nbrs;

			open_ind = get_lowest_f(open);
			if (open[open_ind] == goaln) {
				closed_ind = close_node(open, closed, open_ind);
				int path_len;
				path_len = recreate_path(agent_path, closed, closed_ind);
				printf("Path length: %d\n", path_len);
				printf("Number of expanded nodes: %d\n", num_expanded);
				return true;
			}
			closed_ind = close_node(open, closed, open_ind);
			/* NOTE: get_neighbors calculates each neighbor's g value from the current node */
			nbrs = get_neighbors(_gSpatialDatabase, closed, closed_ind);
			for (int i = 0; i < nbrs.size(); i++) {
				int nbr_ind;

				if (find_node(closed, nbrs[i]) >= 0)
					continue;
				if ((nbr_ind = find_node(open, nbrs[i])) >= 0) {
					if (nbrs[i].g >= open[nbr_ind].g)
						continue;
				} else {
					num_expanded++;
					open.push_back(nbrs[i]);
					nbr_ind = open.size()-1;
				}

				open[nbr_ind].parent_ind = closed_ind;
				//if (open[nbr_ind] == closed[closed_ind])
				open[nbr_ind].g = nbrs[i].g;
				/* CHANGE ME: change this for assignment submission */
				open[nbr_ind].f = nbrs[i].g + manhattan_dist(nbrs[i].point, goal) * heuristic_wt;
			}
		}

		return false;
	}
}
