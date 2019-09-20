/* This cpp file defines MCR greedy search on a given labeled graph
with specified start and goal */

#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>

#include "Graph.hpp"
#include "MCRGreedySolver.hpp"
#include "Timer.hpp"


MCRGreedySolver_t::MCRGreedySolver_t(Graph_t &g, int start, std::vector<int> goalSet)
{
	// initialize the start & goalSet
	m_start = start;
	m_goalSet = goalSet;
	m_targetPoses = g.getTargetPoses();
	for (int i=0; i < m_goalSet.size(); i++)
	{
		m_goalmap[m_goalSet[i]] = m_targetPoses[i];
	}
	// essential elements for MCR Greedy search
	computeH(g); // heuristics
	m_G = std::vector<float>(g.getnNodes(), std::numeric_limits<float>::max());
	m_G[m_start] = 0.0;
	m_smallestCardinality = std::vector<int>(g.getnNodes(), std::numeric_limits<int>::max());
	m_smallestCardinality[m_start] = 0;
	m_open.push(new MCRGNode_t(m_start, m_H[m_start], m_G[m_start]+m_H[m_start], {}, 0, nullptr));
	m_expanded = std::vector<bool>(g.getnNodes(), false);

	m_isFailure = false;

}

MCRGreedySolver_t::~MCRGreedySolver_t()
{
	while (!m_open.empty())
	{
		MCRGNode_t* a1 = m_open.top();
		delete a1;
		m_open.pop();
	}
	for (auto &e : m_closed) { delete e; }
}

void MCRGreedySolver_t::computeH(Graph_t &g)
{
	std::vector<float> goal_mean = std::vector<float>(g.getState(0).size(), 0.0);
	for (auto const &goal : m_goalSet)
	{
		std::vector<float> v_goal = g.getState(goal);
		for (int j=0; j < v_goal.size(); j++)
		{
			goal_mean[j] = goal_mean[j] + v_goal[j];
		}
	}
	for (int j=0; j < goal_mean.size(); j++)
	{
		goal_mean[j] = goal_mean[j] / m_goalSet.size();
	}

	std::vector<float> temp_v;
	for (int i=0; i < g.getnNodes(); i++)
	{
		if (std::find(m_goalSet.begin(), m_goalSet.end(), i) != m_goalSet.end())
		{
			m_H.push_back(0.0);
			continue;
		}
		// compute euclidean distance
		float temp_h = 0.0;
		temp_v = g.getState(i);
		for (int j=0; j < goal_mean.size(); j++)
		{
			temp_h += pow(goal_mean[j]-temp_v[j], 2);
		}
		temp_h = sqrt(temp_h);
		m_H.push_back(temp_h);
	}
}

void MCRGreedySolver_t::MCRGreedy_search(Graph_t &g)
{

	while (!m_open.empty())
	{
		MCRGNode_t *current = m_open.top();
		m_open.pop();
		// Now check if the current node has been expanded
		if (m_expanded[current->m_id] == true)
		{
			// This node has been expanded with the smallest label cardinality for its id
			// No need to put it into the closed list
			delete current;
			continue;
		}
		m_closed.push_back(current);
		m_expanded[current->m_id] = true;

		// a goal in the goalSet has been found
		if ( std::find(m_goalSet.begin(), m_goalSet.end(), current->m_id) != m_goalSet.end() )
		{
			std::cout << "Goal is connected all the way to the start\n";
			back_track_path(); // construct your path
			pathToTrajectory(g);
			m_goalLabels = current->m_labels;
			// printLabels();
			// print the pose the goal indicates
			m_goalIdxReached = m_goalmap[current->m_id];
			m_pathCost = current->m_f;
			// std::cout << "The reaching target pose is: " << m_goalIdxReached << "\n";
			// std::cout << "The cost: " << m_pathCost << "\n";
			return;
		}
		// get neighbors of the current node
		std::vector<int> neighbors = g.getNodeNeighbors(current->m_id);
		for (auto const &neighbor : neighbors)
		{
			// check if the neighbor node has been visited or expanded before
			if ( m_expanded[neighbor] ) {continue;}
			// check neighbor's labels
			std::vector<int> neighborLabels =
				label_union(current->m_labels, g.getEdgeLabels(current->m_id, neighbor));
			int labelsSize = neighborLabels.size();
			// If the neighbor has a smller labels cardinality, update the smallest cardinality
			// record and put into open
			if (labelsSize < m_smallestCardinality[neighbor])
			{
				m_smallestCardinality[neighbor] = labelsSize;
				m_G[neighbor] = m_G[current->m_id]+g.getEdgeCost(current->m_id, neighbor);
				m_open.push(new MCRGNode_t(neighbor, m_H[neighbor], m_G[neighbor]+m_H[neighbor], 
															neighborLabels, labelsSize, current));
				continue;
			}
			if (labelsSize == m_smallestCardinality[neighbor])
			{
				if (m_G[current->m_id]+g.getEdgeCost(current->m_id, neighbor) < m_G[neighbor])
				{
					m_G[neighbor] = m_G[current->m_id]+g.getEdgeCost(current->m_id, neighbor);
					m_open.push(new MCRGNode_t(neighbor, m_H[neighbor], m_G[neighbor]+m_H[neighbor], 
															neighborLabels, labelsSize, current));

				}
			}

		}
	}
	// You are reaching here since the open list is empty and the goal is not found
	std::cout << "The problem is not solvable. Search failed...\n";
	m_isFailure = true;
}

void MCRGreedySolver_t::checkPathSuccess(int nhypo)
{
	m_obstaclesCollided = 0;
	m_isPathSuccess = true;
	// compute the obstacles collided
	// loop through the m_goalLabels
	for (auto const &l : m_goalLabels)
	{
		if (l % nhypo == 0)
		{
			m_obstaclesCollided += 1;
		}
	}
	if (m_obstaclesCollided != 0 or m_goalIdxReached != 0)
	{
		m_isPathSuccess = false;
	}

}

void MCRGreedySolver_t::back_track_path()
{
	// start from the goal
	MCRGNode_t *current = m_closed[m_closed.size()-1];
	while (current->m_id != m_start)
	{
		// keep backtracking the path until you reach the start
		m_path.push_back(current->m_id);
		current = current->m_parent;
	}
	// finally put the start into the path
	m_path.push_back(current->m_id);

	// print the path for checking purpose
	// std::cout << "path: \n";
	// for (auto const &waypoint : m_path)
	// {
	// 	std::cout << waypoint << " ";
	// }
	// std::cout << "\n";	
}

void MCRGreedySolver_t::pathToTrajectory(Graph_t &g)
{
	// start from the start
	for (int i=m_path.size()-1; i >=0; i--)
	{
		m_trajectory.push_back(g.getState(m_path[i]));

	}
	// // print the trajectory for checking purpose
	// std::cout << "The trajectory: \n";
	// for (auto const &t : m_trajectory)
	// {
	// 	for (auto const &d : t)
	// 	{
	// 		std::cout << d << "   ";
	// 	}
	// 	std::cout << "\n";
	// }
}

void MCRGreedySolver_t::writeTrajectory(std::string trajectory_file)
{
	m_outFile_.open(trajectory_file);
	if (m_outFile_.is_open())
	{
		for (auto const &t : m_trajectory)
		{
			for (auto const &d : t)
			{
				m_outFile_ << d << " ";
			}
			m_outFile_ << "\n";
		}
	}
	m_outFile_.close();

}

void MCRGreedySolver_t::print_path()
{
	// print the path for checking purpose
	std::cout << "path: \n";
	for (auto const &waypoint : m_path)
	{
		std::cout << waypoint << " ";
	}
	std::cout << "\n";
}

void MCRGreedySolver_t::printLabels()
{
	std::cout << "labels: " << "< ";
	for (auto const &l : m_goalLabels)
	{
		std::cout << l << " ";
	}
	std::cout << ">\n";	
}

void MCRGreedySolver_t::print_cost()
{
	std::cout << "cost: " << m_pathCost << "\n";
}
		
void MCRGreedySolver_t::print_goalIdxReached()
{
	std::cout << "The reaching target pose is: " << m_goalIdxReached << "\n";
}

void MCRGreedySolver_t::printAll()
{
	print_path();
	print_cost();
	printLabels();
	print_goalIdxReached();
}

std::vector<int> MCRGreedySolver_t::label_union(std::vector<int> s1, std::vector<int> s2)
{
	// sort the sets first before applying union operation
	std::sort(s1.begin(), s1.end());
	std::sort(s2.begin(), s2.end());

	// Declaring resultant vector for union
	std::vector<int> v(s1.size()+s2.size());
	// using function set_union() to compute union of 2
	// containers v1 and v2 and store result in v
	auto it = std::set_union(s1.begin(), s1.end(), s2.begin(), s2.end(), v.begin());

	// resizing new container
	v.resize(it - v.begin());
	return v;
}