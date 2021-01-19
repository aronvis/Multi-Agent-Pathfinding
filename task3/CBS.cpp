#include "CBS.h"
#include <iostream>
#include <queue>
#include <list>

// Determines if there are any collision in the path stored inside currNode 
// Creates constraints based on the collions and the function returns true 
// if the path has at least one collision
bool CBS::findCollisions(CBSNode* currNode, list<Constraint>& constraints)
{
    vector<Path> paths = currNode->paths;
    int agentVal;
    int prevAgentVal;
    int OtherAgentVal;
    int prevOtherAgentVal;
    for(int agent = 0; agent<paths.size(); agent++)
    {
        for(int otherAgent = agent+1; otherAgent<paths.size(); otherAgent++)
        {
            int maxIndex = paths[agent].size();
            if(paths[otherAgent].size() >= paths[agent].size())
            {
                maxIndex = paths[otherAgent].size();
            }
            for(int time = 0; time < maxIndex; time++)
            {
                // Updates index of agent1
                if(time < paths[agent].size())
                {
                    agentVal = paths[agent][time];
                }
                // Updates index of agent2
                if(time < paths[otherAgent].size())
                {
                    OtherAgentVal = paths[otherAgent][time];
                }
                // Add vertex constraints
                if(agentVal == OtherAgentVal)
                {
                    auto vertex = std::make_tuple(agent, agentVal, -1, time);
                    constraints.push_back(vertex);
                    vertex = std::make_tuple(otherAgent, OtherAgentVal, -1, time);
                    constraints.push_back(vertex);
                }
                // Add edge constraints
                if(time >= 1)
                {
                    if((prevAgentVal == OtherAgentVal) && (prevOtherAgentVal == agentVal))
                    {
                        // Adds the edge constraint for agent
                        auto edge = std::make_tuple(agent, prevAgentVal, agentVal, time);
                        constraints.push_back(edge);
                        // edge = std::make_tuple(agent, agentVal, prevAgentVal, time);
                        // constraints.push_back(edge);
                        // Adds the edge constraint for otherAgent
                        edge = std::make_tuple(otherAgent, prevOtherAgentVal, OtherAgentVal, time);
                        constraints.push_back(edge);
                        // edge = std::make_tuple(otherAgent, OtherAgentVal, prevOtherAgentVal, time);
                        // constraints.push_back(edge);
                    }
                }
                prevAgentVal = agentVal;
                prevOtherAgentVal = OtherAgentVal; 
            }
        }
    }
    if(constraints.size() == 0)
    {
        return false;
    }
    // Adds the goal location constraint
    // for(int agent = 0; agent<paths.size(); agent++)
    // {
    //     int lastIndex = paths[agent].size() - 1;
    //     int goalLoc = paths[agent][lastIndex];
    //     auto vertex = std::make_tuple(agent, goalLoc, -2, lastIndex);
    //     constraints.push_back(vertex);
    // }
    return true;
}


vector<Path> CBS::find_solution() 
{
    priority_queue<CBSNode*, vector<CBSNode*>, CompareCBSNode> open; // open list

    /* generate the root CBS node */
    auto root = new CBSNode();
    all_nodes.push_back(root);  

    // find paths for the root node
    root->paths.resize(a_star.ins.num_of_agents);
    for (int i = 0; i < a_star.ins.num_of_agents; i++) 
    {
        root->paths[i] = a_star.find_path(i,root->constraints);
        if (root->paths[i].empty()) 
        {
            cout << "Fail to find a path for agent " << i << endl;
            return vector<Path>(); // return "No solution"
        }
    }
    // compute the cost of the root node
    for (const auto& path : root->paths)
        root->cost += (int)path.size() - 1;

    // put the root node into open list
    open.push(root);
    while (!open.empty()) 
    {
        list<Constraint> newConstraints;
        auto curr = open.top();
        open.pop();
        bool hasCollision = findCollisions(curr, newConstraints);
        if(hasCollision == false)
        {
            return curr->paths;
        }
        else
        {
            for(Constraint c: newConstraints)
            {
                auto next = new CBSNode(*curr);
                all_nodes.push_back(next);
                next->constraints.push_back(c);
                int agent_id = std::get<0>(c);
                Path nextPath = a_star.find_path(agent_id, next->constraints);
                if(!nextPath.empty())
                {
                    next->paths[agent_id] = nextPath;
                     // compute the cost of the root node
                    for (const auto& path : next->paths)
                        next->cost += (int)path.size() - 1;
                    open.push(next);
                }
            }
        }

    }
    return vector<Path>(); // return "No solution"
}


CBS::~CBS() {
    // release the memory
    for (auto n : all_nodes)
        delete n;
}