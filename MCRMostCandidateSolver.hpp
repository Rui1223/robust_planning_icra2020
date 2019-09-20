/* This hpp file declares MCR most candidate greedy search on a given labeled graph
with specified start and goal set */

#ifndef MCRMostCandidateSolver_H
#define MCRMostCandidateSolver_H

#include <vector>
#include <queue>
#include <cstring>
#include <fstream>

#include "Graph.hpp"

struct MCRMCNode_t
{
	int m_id;
	float m_h;
	float m_f;
	// label and cardinality
	std::vector<int> m_labels;
	int m_labelCardinality;

	MCRMCNode_t *m_parent;

	MCRMCNode_t(int id, float h, float f, std::vector<int> ls, int c, MCRMCNode_t *p)
	{
		m_id = id;
		m_h = h;
		m_f = f;
		m_labels = ls;
		m_labelCardinality = c;
		m_parent = p;
	}
};

struct MCRMCNode_comparison
{
	bool operator()(const MCRMCNode_t* a, const MCRMCNode_t* b)
	{
		if (a->m_labelCardinality == b->m_labelCardinality)
		{
			if (a->m_f == b->m_f)
			{
				return (a->m_h) > (b->m_h);
			}
			else
				return (a->m_f) > (b->m_f);
		}
		else
			return (a->m_labelCardinality) > (b->m_labelCardinality);
	}
};


class MCRMostCandidateSolver_t
{
	std::vector<int> m_path;
	std::vector<std::vector<float>> m_trajectory;

	std::priority_queue<MCRMCNode_t*, std::vector<MCRMCNode_t*>, MCRMCNode_comparison> m_open;	
	std::vector<MCRMCNode_t*> m_closed;
	std::vector<bool> m_expanded;

	std::vector<float> m_G;
	std::vector<float> m_H;
	std::vector<int> m_smallestCardinality;
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

	std::vector<int> m_mostPromisingLabels;

public:
	MCRMostCandidateSolver_t(Graph_t &g, int start, std::vector<int> goalSet);
	~MCRMostCandidateSolver_t();

	void computeH(Graph_t &g);
	void MCRMCGreedy_search(Graph_t &g);
	void back_track_path();
	void pathToTrajectory(Graph_t &g);
	void writeTrajectory(std::string trajectory_file);
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