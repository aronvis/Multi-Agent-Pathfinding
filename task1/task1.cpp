#include <iostream>
#include <fstream>
#include "MAPFInstance.h"
#include "AStarPlanner.h"
#include <tuple>

int main(int argc, char *argv[]) {
    MAPFInstance ins;
    string input_file = argv[1];
    string output_file = argv[2];
    if (ins.load_instance(input_file)) {
        ins.print_instance();
    } else {
        cout << "Fail to load the instance " << input_file << endl;
        exit(-1);
    }

    AStarPlanner a_star(ins);
    vector<Path> paths(ins.num_of_agents);
    for (int i = 0; i < ins.num_of_agents; i++) 
    {
        list<Constraint> constraints;
        //  constraints for Q1
        // if(i == 0)
        // {
        //     auto temp = std::make_tuple(i, ins.goal_locations[i], -1, 4);
        //     constraints.push_back(temp);
        // }
        // else if(i == 1)
        // {
        //     for (auto next_location : ins.get_adjacent_locations(ins.start_locations[i])) 
        //     {
        //         if(next_location != ins.start_locations[i])
        //         {
        //             auto temp = std::make_tuple(i, ins.start_locations[i], next_location, 1);
        //             constraints.push_back(temp);
        //         }
        //     }
        // }
        //  constraints for Q2
        // if(i == 0)
        // {
        //     auto temp = std::make_tuple(i, ins.goal_locations[i], -1, 10);
        //     constraints.push_back(temp);
        // }
        // constraints for Q3
        // if(i == 1)
        // {
        //     for (auto next_location : ins.get_adjacent_locations(10))
        //     {
        //         if(next_location != 17)
        //         {
        //             auto temp = std::make_tuple(i, next_location, -1, 2);
        //             constraints.push_back(temp);
        //         }
        //     }
        // }
        paths[i] = a_star.find_path(i,constraints);
        if (paths[i].empty()) {
            cout << "Fail to find any solutions for agent " << i << endl;
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