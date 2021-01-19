# Multi Agent Pathfinding
Multi-agent pathfinding AI in C++ using time-space A*, prioritized planning, and conflict-based search. To run the program:
1. Install python3-numpy python3-matplotlib
2. Time-Space A-Star:  
	cd task1  
	cmake .  
	make  
	task1 exp1.txt exp1_paths.txt  
	python3 ../visualize.py exp1.txt exp1_paths.txt  

3. Prioritized Planning:  
	cd task2  
	cmake .  
	make  
	task2 exp2_1.txt exp2_1_paths.txt  
	python3 ../visualize.py exp2_1.txt exp2_1_paths.txt  

3. Conflict-Based Search:  
	cd task3  
	cmake .  
	make  
	task3 exp3_1.txt exp3_1_paths.txt  
	python3 ../visualize.py exp3_1.txt exp3_1_paths.txt 