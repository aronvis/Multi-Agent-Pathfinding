#include <iostream>
#include <fstream>
#include "MAPFInstance.h"
#include "AStarPlanner.h"
#include <tuple>
#include <vector>

int main(int argc, char *argv[]) {
    MAPFInstance ins;
    string input_file = argv[1];
    string output_file = argv[2];
    if (ins.load_instance(input_file)) 
    {
        ins.print_instance();
    } 
    else 
    {
        cout << "Fail to load the instance " << input_file << endl;
        exit(-1);
    }

    AStarPlanner a_star(ins);
    vector<Path> paths(ins.num_of_agents);
    list<Constraint> constraints;
    // assign priority ordering to agents
    // By default, we use the index ordering of the agents where
    // the first always has the highest priority.
    vector<int> priorities;
    for (int i = 0; i < ins.num_of_agents; i++) 
    {
        priorities.push_back(i);
    }

    // Question 3: Switched Priorities
    // int temp = priorities[0];
    // priorities[0] = priorities[1];
    // priorities[1] = temp;

    // plan paths
    for (int i = 0; i<priorities.size(); i++) 
    {
        if(i >= 1)
        {
            int prevAgent = priorities[i-1];
            // Iterates over the path of the last agent to create a constraint
            for(int time = 0; time < paths[prevAgent].size(); time++)
            {
                // Creates a constraint for each agent after the previous agent
                for(int agent = i; agent < priorities.size(); agent++)
                {
                    int nextAgent = priorities[agent];
                    // Regular vertex constraint
                    if (time < paths[prevAgent].size() - 1)
                    {
                        auto vertex = std::make_tuple(nextAgent, paths[prevAgent][time], -1, time);
                        constraints.push_back(vertex);
                    }
                    // Permanent vertex constraint
                    else
                    {
                        auto vertex = std::make_tuple(nextAgent, paths[prevAgent][time], -2, time);
                        constraints.push_back(vertex);
                    }
                    // Edge constraint
                    if(time >= 1)
                    {
                        // Adds the edge constraint from left to right 
                        auto edge = std::make_tuple(nextAgent, paths[prevAgent][time-1], paths[prevAgent][time], time);
                        constraints.push_back(edge);
                        // Adds the edge constraint from right to left 
                        edge = std::make_tuple(nextAgent, paths[prevAgent][time], paths[prevAgent][time-1], time);
                        constraints.push_back(edge);
                    }
                }
            }
        }
        int currAgent = priorities[i];
        paths[currAgent] = a_star.find_path(currAgent, constraints);
        if (paths[currAgent].empty()) 
        {
            cout << "Fail to find any solutions for agent " << currAgent << endl;
            return 0;
        }
    }

    // print paths
    cout << "Paths:" << endl;
    int sum = 0;
    for (int i = 0; i < ins.num_of_agents; i++) {
        cout << "a" << i << ": " << paths[i] << endl;
        sum += (int)paths[i].size() - 1;
    }
    cout << "Sum of cost: " << sum << endl;

    // save paths
    ofstream myfile (output_file.c_str(), ios_base::out);
    if (myfile.is_open()) {
        for (int i = 0; i < ins.num_of_agents; i++) {
            myfile << paths[i] << endl;
        }
        myfile.close();
    } else {
        cout << "Fail to save the paths to " << output_file << endl;
        exit(-1);
    }
    return 0;
}