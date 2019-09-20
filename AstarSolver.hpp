/* This hpp file declares A* search on a given graph
with specified start and goal */

#ifndef ASTARSOLVER_H
#define ASTARSOLVER_H

#include <vector>
#include <queue>
#include <cstring>
#include <fstream>

#include "Graph.hpp"


struct AstarNode_t
{
	int m_id;
	float m_h;
	float m_f;
	AstarNode_t *m_parent;
	AstarNode_t(int id, float h, float f, AstarNode_t *p)
	{
		m_id = id;
		m_h = h;
		m_f = f;
		m_parent = p;
	}
};

struct AstarNode_comparison
{
	bool operator()(const AstarNode_t* a, const AstarNode_t* b)
	{
		if (a->m_f == b->m_f)
		{
			return (a->m_h) > (b->m_h);
		}
		return (a->m_f) > (b->m_f);
	}
};


class AstarSolver_t
{
	std::vector<int> m_path;
	std::vector<std::vector<float>> m_trajectory;

	std::priority_queue<AstarNode_t*, std::vector<AstarNode_t*>, AstarNode_comparison> m_open;
	std::vector<AstarNode_t*> m_closed;
	std::vector<bool> m_expanded;
	
	std::vector<float> m_G;
	std::vector<float> m_H;
	int m_start;
	std::vector<int> m_goalSet;
	std::vector<int> m_targetPoses;
	std::map<int, int> m_goalmap;

	std::ofstream m_outFile_;
	bool m_isFailure;

	std::vector<int> m_goalLabels;
	int m_goalIdxReached;
	int m_obstaclesCollided;
	bool m_isPathSuccess;
	float m_pathCost;


public:
	AstarSolver_t(Graph_t &g, int start, std::vector<int> goalSet);
	~AstarSolver_t();

	void computeH(Graph_t &g);
	void Astar_search(Graph_t &g);
	void back_track_path();
	void pathToTrajectory(Graph_t &g);
	void writeTrajectory(std::string trajectory_file);
	void computeLabels(Graph_t &g);
	void printLabels();
	void print_path();
	void print_cost();
	void print_goalIdxReached();
	void printAll();

	std::vector<int> label_union(std::vector<int> s1, std::vector<int> s2);


	// harvest the results
	void checkPathSuccess(int nhypo);

	// getters
	std::vector<std::vector<float>> getTrajectory() {return m_trajectory;}
	bool getFailureIndicator() {return m_isFailure;}
	int getObstaclesCollided() {return m_obstaclesCollided;}
	bool getIsPathSuccess() {return m_isPathSuccess;}
	float getPathCost() {return m_pathCost;}

};



#endif