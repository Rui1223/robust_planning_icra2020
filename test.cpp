#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <sstream>
#include <algorithm>
#include <map>

#include <iterator>
// #include "Graph.hpp"
// #include "AstarSolver.hpp"
// #include "Timer.hpp"

std::vector<int> label_union(std::vector<int> s1, std::vector<int> s2)
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


int main(int argc, char** argv)
{
	bool a = true;
	bool b = false;

	if (a > b)
	{
		std::cout << "happy!\n";
	}

	return 0;
}









//////////////////////////////////////////////////////////////////////////////
// int main(int argc, char** argv)
// {
// 	std::vector<int> v = { 1, 2, 3, 3, 3, 10, 1, 2, 3, 7, 7, 8 };
// 	std::vector<int>::iterator ip;
// 	std::vector<int> copy_v = v;
// 	std::sort(copy_v.begin(),copy_v.end());
// 	copy_v.resize(std::distance(copy_v.begin(), std::unique(copy_v.begin(), copy_v.end())));
// 	int count = copy_v.size();
// 	std::cout << "count: " << count << "\n";
// 	for (auto const &e : copy_v)
// 	{
// 		std::cout << e << " ";
// 	}
// 	std::cout << std::endl;
// 	for (auto const &ee: v)
// 	{
// 		std::cout << ee << " ";
// 	}
// 	std::cout << std::endl;


// 	return 0;
// }






//////////////////////////////////////////////////////////////////////////////
// int main(int argc, char** argv)
// {
// 	std::string label_file = "labels.txt";
// 	std::ifstream m_inFile_;
// 	m_inFile_.open(label_file);

// 	std::string str;

// 	int a;
// 	float b;
// 	std::vector<int> k;
// 	int c;

// 	if (!m_inFile_)
// 	{
// 		std::cerr << "Unable to open the samples file\n";
// 		exit(1); // call system to stop
// 	}
// 	while (std::getline(m_inFile_, str))
// 	{
// 		std::stringstream ss(str);
// 		// std::cout << ss.str() << "\n";
// 		// continue;
// 		ss >> a >> b;
// 		// std::cout << a << " " << b << " ";
// 		while(ss >> c)
// 		{
// 			k.push_back(c);
// 			// std::cout << c << " ";
// 		}
// 		// std::cout << "\n";
// 	}

// 	// if (!m_inFile_)
// 	// {
// 	// 	std::cerr << "Unable to open the file\n";
// 	// 	exit(1); // call system to stop
// 	// }
// 	// while (!m_inFile_.eof())
// 	// {
// 	// 	m_inFile_ >> a >> b;
// 	// 	std::cout << "What is the problem?\n";
// 	// 	while (m_inFile_ >> c)
// 	// 	{
// 	// 		k.push_back(c);
// 	// 		std::cout << "successfully push back " << c << "/n";
// 	// 	}
// 	// }

// 	for (auto const &kk : k)
// 	{
// 		std::cout << kk << " ";
// 	}
// 	std::cout << "\n";

// 	return 0;
// }