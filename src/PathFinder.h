#pragma once
// file PathFinder.h
// Stores all data and function definitions used with PathFinder.cpp
// author Jack Pitman
//File Includes
#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>
#include <queue>
#include <string>
#include <algorithm>
#include <math.h>
#include <chrono>
#include <ctime>
#include <random>
#include <SFML/Graphics.hpp>

//Used for A* ONLY, all other uses of methods within std, feature std::
using namespace std;
//Header Contents
class PathFinder
{
//Public variables
public:
	//Constructor and Destructor for this class
	PathFinder();
	~PathFinder();
	//Start method
	void Start();
	//Generator used to create random numbers
	std::default_random_engine m_generator;

//Private variables
private:
	//Window used to show the UI
	sf::RenderWindow m_window;
	//Width of the map
	int m_width = 0;
	//Height of the map
	int m_height = 0;
	//X position of the start point
	int m_xStart;
	//Y position of the start point
	int m_yStart;
	//X position of the finish point
	int m_xFinish;
	//Y position of the finish point
	int m_yFinish;
	//Variables used to create the map, and also further arrays used for A*
	int** m_map1;
	int** m_open_nodes_map;
	int** m_closed_nodes_map;
	int** m_dir_map;


	//amount of chromosomes in the population
	unsigned int m_populationSize = 10;
	//Length of the chromosome
	unsigned int m_chromoLength = 16;
	//Current chromosome number
	unsigned int m_cNum = 0;
	//Current chromosome position
	unsigned int m_cPos = 0;
	//Total fitness
	float m_totalFitness = 0;
	//Chance of crossover
	float m_crossOverRate = 0.7;
	//Chance of mutation
	float m_mutationRate = 0.01;
	//Generation number
	unsigned int m_generation = 1;
	//Struct storing information regarding chromsomes
	struct Chromosome
	{
		//Amount of moves possible (Pairs), set at run time
		std::vector<unsigned int> m_chromo;
		//Moves the chromosome has done, used to display the path
		std::vector<char> m_chromoMoves;
		//ID of the chromosome
		unsigned int m_id = 0;
		//Fitness of the chromosome
		float m_fitness = 0;
		//Current X position of the chromosome
		unsigned int m_xPos = 0;
		//Current Y position of the chromosome
		unsigned int m_yPos = 0;
		//Current mating chance of the chromosome
		float m_matingChance = 0;
		//Current position to breed
		float m_matingPos = 0;
		//Method used to sort chromsomes in terms of mating chance
		bool operator<(const Chromosome& a) const
		{
			return m_matingChance < a.m_matingChance;
		}
	};
	//Vector of chromosomes
	std::vector<Chromosome> m_chromosomes;
	//Vector of breeding chromosomes
	std::vector<Chromosome> m_breedChromo;
	//Last picked chromosome so it does not get picked again
	Chromosome m_lastPicked;
	//Current chromosome
	Chromosome m_currentChromo;
	//Whether A* is being used, used for determining window title etc
	bool m_isAStar = false;
	//Time variables, used to determine time taken to find path
	std::chrono::system_clock::time_point m_tStart, m_tEnd;
	//Textures used to display sprites for SFML
	sf::Texture* m_Tex;
	sf::Sprite m_floorSpr;
	sf::Sprite m_goalSpr;
	sf::Sprite m_obstacleSpr;
	sf::Sprite m_startSpr;

	//Method used to load the map
	void LoadMap();
	//Method used to load textures
	sf::Texture* LoadTexture(std::string path);
	//Method used to set textures
	void SetTexture();
	//Method used to show the window
	void ShowWindow();
	//Methopd used to pathfind using A*
	void AStar();
	//Method used to pathfind using A*
	string PathFind(const int & xStart, const int & yStart, const int & xFinish, const int & yFinish);
	//Method used to pathfind using GA
	void GeneticAlgorithm();
	//Method used to generate an initial random population
	void GenerateInitialPopulation();
	//Method used to pathfind using GA
	bool GAPathfind();
	//Method used to breed chromosomes for future generations
	void BreedSelection();
	//Method used to crossover chromosomes at random
	void CrossoverSelection();
	//Method used to mutate chromosomes at random
	void MutateSelection();
};
