/* This hpp file declares MaxSuccess Greedy search on a given labeled graph
with specified start and goal set */

#ifndef MAXSUCCESSGREEDYSOLVER_H
#define MAXSUCCESSGREEDYSOLVER_H

#include <vector>
#include <queue>
#include <cstring>
#include <fstream>
#include <map>

#include "Graph.hpp"

struct MaxSuccGreedyNode_t
{
	int m_id;
	float m_g;
	float m_h;
	float m_f;
	// label and survivability
	std::vector<int> m_labels;
	float m_survival;
	int m_labelCardinality;

	//goal uncertainty
	std::vector<int> m_goalIndexes;
	float m_reachability;
	float m_successValue;
	bool m_isGoal;

	MaxSuccGreedyNode_t *m_parent;

	MaxSuccGreedyNode_t(int id, float g, float h, std::vector<int> labels, float survival, 
		std::vector<int> goalIdxes, float reachability, bool isGoal, MaxSuccGreedyNode_t *parent)
	{
		m_id = id;
		m_g = g;
		m_h = h;
		m_f = m_g + m_h;
		m_labels = labels;
		m_survival = survival;
		m_labelCardinality = m_labels.size();
		m_goalIndexes = goalIdxes;
		m_reachability = reachability;
		m_successValue = m_survival * m_reachability;
		m_isGoal = isGoal;
		m_parent = parent;
	}
};

struct MaxSuccGreedyNode_comparison
{
	bool operator()(const MaxSuccGreedyNode_t* a, const MaxSuccGreedyNode_t* b)
	{
		if (a->m_successValue == b->m_successValue)
		{
			if (a->m_f == b->m_f)
			{
				return (a->m_h) > (b->m_h);
			}
			else
				return (a->m_f) > (b->m_f);
		}
		else
			return (a->m_successValue) < (b->m_successValue);
	}
};

class MaxSuccessGreedySolver_t
{
	std::vector<int> m_path;
	std::vector<std::vector<float>> m_trajectory;

	std::priority_queue<MaxSuccGreedyNode_t*, std::vector<MaxSuccGreedyNode_t*>, 
													MaxSuccGreedyNode_comparison> m_open;
	std::vector<MaxSuccGreedyNode_t*> m_closed;
	std::vector<bool> m_expanded;
	std::map<int, bool> m_expandedForGoals;
	std::vector<float> m_F;
	std::map<int, float> m_FForGoals;
	std::vector<float> m_highestSuccess;
	std::map<int, float> m_highestSuccessForGoals;

	// No m_H since heuristics are dynamic

	int m_start;

	std::vector<int> m_goalSet;
	std::vector<int> m_targetPoses;
	std::map<int, int> m_goalmap;

	std::vector<int> m_goalhypos;
	std::map<int, int> m_goalCounts;
	std::map<int, std::vector<float>> m_goalMean;

	std::map<int, std::pair<int, float>> m_labelWeights;
	int m_nobstacles;

	std::ofstream m_outFile_;
	bool m_isFailure;

	std::vector<int> m_goalLabels;
	int m_goalIdxReached;
	int m_obstaclesCollided;
	bool m_isPathSuccess;
	float m_pathCost;

public:
	MaxSuccessGreedySolver_t(Graph_t &g);
	~MaxSuccessGreedySolver_t();

	std::vector<int> update_goalIdxes(std::vector<int> currGoalIndexes, std::vector<int> edgelabels);

	int countObs();
	void computeGoalMean(Graph_t &g);
	float computeH(std::vector<float> temp_state, std::vector<int> goalIdxes);
	float computeSurvival(std::vector<int> labels);
	float computeReach(std::vector<int> goalIdxes); // the highest reachability among available goals
	void MSGreedy_search(Graph_t &g);
	void back_track_path();
	void pathToTrajectory(Graph_t &g);
	void writeTrajectory(std::string trajectory_file);
	void printLabels();
	void print_path();
	void print_cost();
	void print_goalIdxReached();
	void printAll();

	std::vector<int> label_union(std::vector<int> s1, std::vector<int> s2);
	int getIndex(int goalIdx);

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