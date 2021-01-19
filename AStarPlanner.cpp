#include "AStarPlanner.h"
#include <queue>
#include <unordered_map>
#include <algorithm> // reverse
#include <iostream>

ostream& operator<<(ostream& os, const Path& path)
{
    for (auto loc : path) {
        os << loc << " ";
    }
    return os;
}

Path AStarPlanner::make_path(const AStarNode* goal_node) const {
    Path path;
    const AStarNode* curr = goal_node;
    while (curr != nullptr) {
        path.push_back(curr->location);
        curr = curr->parent;
    }
    std::reverse(path.begin(),path.end());
    return path;
}

// Stores the latest time period of a constraint on the goal location
// inside a member variable called last_constraint
void AStarPlanner::setLastConstraint(int agent_id, const list<Constraint>& constraints)
{
	last_constraint = 0;
	for (auto it = constraints.begin(); it != constraints.end(); it++)
	{
		Constraint constraint = *it;
		if (std::get<2>(constraint) == -1)
		{
			if ((std::get<0>(constraint) == agent_id) && (std::get<1>(constraint) == ins.goal_locations[agent_id]) 
				&& (std::get<3>(constraint) >= last_constraint))
			{
				last_constraint = std::get<3>(constraint);
			}
		}
	}
}

// Check if a given move for the agent is valid considering all the move constraints on the agent
bool AStarPlanner::validMove(int agent_id, AStarNode * node, int next_time, const list<Constraint>& constraints)
{
	for (auto it = constraints.begin(); it != constraints.end(); it++)
	{
		Constraint constraint = *it;
		// Checks for regular vertex constraint
		if (std::get<2>(constraint) == -1)
		{
			if ((std::get<0>(constraint) == agent_id) && (std::get<1>(constraint) == node->location) 
				&& (std::get<3>(constraint) == next_time))
				return false;
		}
		// Checks for permanent vertex constraint
		else if (std::get<2>(constraint) == -2)
		{
			if ((std::get<0>(constraint) == agent_id) && (std::get<1>(constraint) == node->location) 
				&& (std::get<3>(constraint) <= next_time))
				return false;
		}
		// Checks for edge constraint
		else
		{
			if((std::get<0>(constraint) == agent_id) && (std::get<1>(constraint) == node->parent->location) 
				&& (std::get<2>(constraint) == node->location) && (std::get<3>(constraint) == next_time))
				return false;
		}
	}
	return true;
}

Path AStarPlanner::find_path(int agent_id, const list<Constraint>& constraints) 
{
    int start_location = ins.start_locations[agent_id];
    int goal_location = ins.goal_locations[agent_id];
    // Determines the time period of the last constraint on the agent
    setLastConstraint(agent_id, constraints);
    int maxTime = last_constraint + ins.map_size();
    // Open list
    priority_queue<AStarNode*, vector<AStarNode*>, CompareAStarNode> open;

    // Unordered map is an associative container that contains key-value pairs with unique keys.
    // The following unordered map is used for duplicate detection, where the key is the location of the node.
    unordered_map<pair<int, int>, AStarNode*, hash_pair> all_nodes;
    int h = ins.get_Manhattan_distance(start_location, goal_location); // h value for the root node
    int next_time = 0;
    auto root = new AStarNode(next_time, start_location, 0, h, nullptr);
    open.push(root);
    Path path;
    while (!open.empty()) 
    {
        auto curr = open.top();
        open.pop();
        // Goal test condition 
        if((curr->location == goal_location) && (next_time > last_constraint))
        {
            path = make_path(curr);
            break;
        }
        // No solution is possible
        if(next_time > maxTime)
        {
        	break;
        }
        // generate child nodes
        for (auto next_location : ins.get_adjacent_locations(curr->location)) 
        {
        	// Change timestamp
        	next_time = curr->time + 1;
            auto it = all_nodes.find(std::make_pair(next_location, next_time));
            // The location has not been visited before 
            if(it == all_nodes.end())
            {
            	int next_g = curr->g + 1;
                int next_h = ins.get_Manhattan_distance(next_location, goal_location);
                auto next = new AStarNode(next_time, next_location, next_g, next_h, curr);
                // Adds the node to the queue if it is a valid option from time t-1 to time t
                if(validMove(agent_id, next, next_time, constraints))
                {
                	open.push(next);
					all_nodes[std::make_pair(next_location, next_time)] = next;
                }
                // Deletes the memory of the invalid node
                else
                {
                	delete next;
                }
            }
            // Note that if the location has been visited before,
            // next_g + next_h must be greater than or equal to the f value of the existing node,
            // because we are searching on a 4-neighbor grid with uniform-cost edges.
            // So we don't need to update the existing node.
        }
    }

    // release memory
    for (auto n : all_nodes)
        delete n.second;

    return path;
}