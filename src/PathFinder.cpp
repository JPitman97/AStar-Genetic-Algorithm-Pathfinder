#include "PathFinder.h"

//Global variables used for A*, for example directional movement and direction pairs
const int g_dir = 8; // number of possible directions to go at any position
// if dir==4
//static int dx[dir]={1, 0, -1, 0};
//static int dy[dir]={0, 1, 0, -1};
// if dir==8
static int g_dx[g_dir] = { 1, 1, 0, -1, -1, -1, 0, 1 };
static int g_dy[g_dir] = { 0, 1, 1, 1, 0, -1, -1, -1 };

class node
{
	// current position
	int xPos;
	int yPos;
	// total distance already travelled to reach the node
	int level;
	// priority=level+remaining distance estimate
	int priority;  // smaller: higher priority

public:
	node(int xp, int yp, int d, int p)
	{
		xPos = xp; yPos = yp; level = d; priority = p;
	}

	int getxPos() const { return xPos; }
	int getyPos() const { return yPos; }
	int getLevel() const { return level; }
	int getPriority() const { return priority; }

	void updatePriority(const int & xDest, const int & yDest)
	{
		priority = level + estimate(xDest, yDest) * 10; //A*
	}

	// give better priority to going strait instead of diagonally
	void nextLevel(const int & i) // i: direction
	{
		level += (g_dir == 8 ? (i % 2 == 0 ? 10 : 14) : 10);
	}

	// Estimation function for the remaining distance to the goal.
	const int & estimate(const int & xDest, const int & yDest) const
	{
		static int xd, yd, d;
		xd = xDest - xPos;
		yd = yDest - yPos;

		// Euclidian Distance
		d = static_cast<int>(sqrt(xd*xd + yd * yd));

		// Manhattan distance
		//d=abs(xd)+abs(yd);

		// Chebyshev distance
		//d=max(abs(xd), abs(yd));

		return(d);
	}
};
// Determine priority (in the priority queue)
bool operator<(const node& a, const node& b)
{
	return a.getPriority() > b.getPriority();
}
//PathFinder class constructor, unused
PathFinder::PathFinder()
{

}
//PathFinder class destructor, used to deallocate pointers
PathFinder::~PathFinder()
{
	//deallocate the arrays
	for (int i = 0; i < m_height + 1; i++)
		delete[] m_map1[i];
	delete[] m_map1;
	if (m_open_nodes_map != nullptr)
	{
		for (int i = 0; i < m_height; i++)
			delete[] m_open_nodes_map[i];
		delete[] m_open_nodes_map;

		for (int i = 0; i < m_height; i++)
			delete[] m_closed_nodes_map[i];
		delete[] m_closed_nodes_map;

		for (int i = 0; i < m_height; i++)
			delete[] m_dir_map[i];
		delete[] m_dir_map;
	}

	delete m_Tex;
}
//Start method, used to give the player a choice and set variables
void PathFinder::Start()
{
	//Load the map from a txt file
	LoadMap();
	//Set the textures of sprites
	SetTexture();
	//Prompting the user to choose which AI to use
	std::cout << "Please choose which AI you want to use to pathfind" << std::endl;
	std::cout << "1) Genetic Algorithm" << std::endl;
	std::cout << "2) A* algorithm" << std::endl;
	//Choice variable used to set the users choice
	char choice = 0;
	choice = std::cin.get();
	//Switch statement used to run the corresponding code
	switch (choice)
	{
		case '1':
		{
			//Run GA
			m_isAStar = false;
			//Create a window to show paths taken by chromosomes
			m_window.create(sf::VideoMode(m_width * 50, m_height * 50), "Genetic Algorithm");
			//Start the timer used to work out time taken once a path is found
			m_tStart = std::chrono::system_clock::now();
			//Generate an initial population
			GenerateInitialPopulation();
			//Start the Genetic Algorithm
			GeneticAlgorithm();
			break;
		}
		case '2':
		{
			//Run A*
			m_isAStar = true;
			//Create a window to show paths taken by A*
			m_window.create(sf::VideoMode(m_width * 50, m_height * 50), "A* Algorithm");
			//Start A*
			AStar();
			break;
		}
		default:
			break;
	}
}
//Loads the map
void PathFinder::LoadMap()
{
	//Temporary variable used to store the current map number being loaded
	int t;
	//mapFile object
	std::ifstream mapFile;
	//Loading in the map.txt file
	mapFile.open("Map.txt");
	//Check if the file opened sucessfully
	if (mapFile.is_open())
	{
		//Check if there are still items to load and create a temporary variable
		int i = 0;
		while (mapFile >> t)
		{
			//Check an arbitry value used to make sure this code runs AFTER the code beneath
			if (i == 100)
			{
				//Loop through the height variable
				for (int x = 0; x < m_height; x++)
				{
					//Loop through the width variable
					for (int j = 0; j < m_width; j++)
					{
						//Set the map height and width to the loaded in value
						m_map1[x][j] = t;
						//If the current value is 2 meaning start, save it
						if (m_map1[x][j] == 2)
						{
							m_xStart = j;
							m_yStart = x;
						} //If the current value is 3 meaning finish, save it
						else if (m_map1[x][j] == 3)
						{
							m_xFinish = j;
							m_yFinish = x;
						}
						//If the file is not over, continue loading in new values
						if (!mapFile.eof())
						{
							mapFile >> t;
						}
					}
				}
			}
			//Check if i is 0 which it starts as, so this runs first
			if (i == 0)
			{
				//Set width equal to the first value loaded in from the text fle
				m_width = t;
				//Print the width
				std::cout << "Width is: " << m_width << std::endl;
				//Increment i
				i = 1;
			}//Check if i has been incremented, meaning the above code ran
			else if (i == 1)
			{
				//Set height equal to the next value in the file
				m_height = t;
				//allocate the array using the width and height
				m_map1 = new int*[m_height + 1];
				for (int i = 0; i < m_height + 1; i++)
					m_map1[i] = new int[m_width + 1];
				//Print height
				std::cout << "Height is: " << m_height << std::endl;
				i = 100; //So these statements never trigger again
			}
		}

	}
}
//Loads the textures and returns a pointer to them, slightly overkill but I wanted the possibility of sprites
sf::Texture* PathFinder::LoadTexture(std::string path)
{
	//Create a sf::Texture pointer
	sf::Texture* Tex = new sf::Texture();
	//Load in a texture and assign it, then return it
	if (!Tex->loadFromFile(path))
	{
		std::cout << "Failed to load image: " << path << std::endl;
	}
	else
	{
		return Tex;
	}
}
//Use the method above to set textures of the sprites
void PathFinder::SetTexture()
{
	//Set's the sprites texture to the ones loaded using the previous method
	m_Tex = LoadTexture("Sprites\\Floor.jpg");
	m_floorSpr.setTexture(*m_Tex);
	m_Tex = LoadTexture("Sprites\\Goal.jpg");
	m_goalSpr.setTexture(*m_Tex);
	m_Tex = LoadTexture("Sprites\\Obstacle.jpg");
	m_obstacleSpr.setTexture(*m_Tex);
	m_Tex = LoadTexture("Sprites\\Start.jpg");
	m_startSpr.setTexture(*m_Tex);
}
//Run A*
void PathFinder::AStar()
{

	m_closed_nodes_map = new int*[m_height];
	for (int i = 0; i < m_height; i++)
		m_closed_nodes_map[i] = new int[m_width]; // map of closed (tried-out) nodes

	m_open_nodes_map = new int*[m_height];
	for (int i = 0; i < m_height; i++)
		m_open_nodes_map[i] = new int[m_width]; // map of open (not-yet-tried) nodes

	m_dir_map = new int*[m_height];
	for (int i = 0; i < m_height; i++)
		m_dir_map[i] = new int[m_width]; // map of directions

	srand(time(NULL));

	cout << "Map Size (X,Y): " << m_width << "," << m_height << endl;
	cout << "Start: " << m_xStart << "," << m_yStart << endl;
	cout << "Finish: " << m_xFinish << "," << m_yFinish << endl;

	// get the route
	clock_t start = clock();
	string route = PathFind(m_yStart, m_xStart, m_yFinish, m_xFinish);
	if (route == "")
	{
		cout << "An empty route generated!" << endl;
		return;
	}
	clock_t end = clock();
	double time_elapsed = double(end - start);
	cout << "Time to calculate the route (ms): " << time_elapsed << endl;
	cout << "Route:" << endl;
	cout << route << endl << endl;

	// follow the route on the map and display it 
	if (route.length() > 0)
	{
		int j; char c;
		int x = m_yStart;
		int y = m_xStart;
		m_map1[x][y] = 2;
		for (int i = 0; i < route.length(); i++)
		{
			c = route.at(i);
			j = atoi(&c);
			x = x + g_dx[j];
			y = y + g_dy[j];
			m_map1[x][y] = 3;
		}
		m_map1[x][y] = 4;


		for (int i = 0; i < m_height; i++)
		{
			for (int y = 0; y < m_width; y++)
			{
				if (m_map1[i][y] == 0)
					cout << ".";
				else if (m_map1[i][y] == 1)
					cout << "O"; //obstacle
				else if (m_map1[i][y] == 2)
					cout << "S"; //start
				else if (m_map1[i][y] == 3)
					cout << "R"; //route
				else if (m_map1[i][y] == 4)
					cout << "F"; //finish
			}
			cout << endl;
		}
		ShowWindow();
	}
}

string PathFinder::PathFind(const int & xStart, const int & yStart, const int & xFinish, const int & yFinish)
{
	static priority_queue<node> pq[2]; // list of open (not-yet-tried) nodes
	static int pqi; // pq index
	static node* n0;
	static node* m0;
	static int i, j, x, y, xdx, ydy;
	static char c;
	pqi = 0;

	// reset the node maps
	for (y = 0; y < m_width; y++)
	{
		for (x = 0; x < m_height; x++)
		{
			m_closed_nodes_map[x][y] = 0;
			m_open_nodes_map[x][y] = 0;
		}
	}

	// create the start node and push into list of open nodes
	n0 = new node(xStart, yStart, 0, 0);
	n0->updatePriority(xFinish, yFinish);
	pq[pqi].push(*n0);
	m_open_nodes_map[x - 1][y - 1] = n0->getPriority(); // mark it on the open nodes map

	// A* search
	while (!pq[pqi].empty())
	{
		// get the current node w/ the highest priority
		// from the list of open nodes
		n0 = new node(pq[pqi].top().getxPos(), pq[pqi].top().getyPos(),
			pq[pqi].top().getLevel(), pq[pqi].top().getPriority());

		x = n0->getxPos(); y = n0->getyPos();

		pq[pqi].pop(); // remove the node from the open list
		m_open_nodes_map[x][y] = 0;
		// mark it on the closed nodes map
		m_closed_nodes_map[x][y] = 1;

		// quit searching when the goal state is reached
		//if((*n0).estimate(xFinish, yFinish) == 0)
		if (x == xFinish && y == yFinish)
		{
			// generate the path from finish to start
			// by following the directions
			string path = "";
			while (!(x == xStart && y == yStart))
			{
				j = m_dir_map[x][y];
				c = '0' + (j + g_dir / 2) % g_dir;
				path = c + path;
				x += g_dx[j];
				y += g_dy[j];
			}

			// garbage collection
			delete n0;
			// empty the leftover nodes
			while (!pq[pqi].empty()) pq[pqi].pop();
			return path;
		}

		// generate moves (child nodes) in all possible directions
		for (i = 0; i < g_dir; i++)
		{
			xdx = x + g_dx[i]; ydy = y + g_dy[i];

			if (!(xdx<0 || xdx>m_height - 1 || ydy<0 || ydy>m_width - 1 || m_map1[xdx][ydy] == 1
				|| m_closed_nodes_map[xdx][ydy] == 1))
			{
				// generate a child node
				m0 = new node(xdx, ydy, n0->getLevel(),
					n0->getPriority());
				m0->nextLevel(i);
				m0->updatePriority(xFinish, yFinish);

				// if it is not in the open list then add into that
				if (m_open_nodes_map[xdx][ydy] == 0)
				{
					m_open_nodes_map[xdx][ydy] = m0->getPriority();
					pq[pqi].push(*m0);
					// mark its parent node direction
					m_dir_map[xdx][ydy] = (i + g_dir / 2) % g_dir;
				}
				else if (m_open_nodes_map[xdx][ydy] > m0->getPriority())
				{
					// update the priority info
					m_open_nodes_map[xdx][ydy] = m0->getPriority();
					// update the parent direction info
					m_dir_map[xdx][ydy] = (i + g_dir / 2) % g_dir;

					// replace the node
					// by emptying one pq to the other one
					// except the node to be replaced will be ignored
					// and the new node will be pushed in instead
					while (!(pq[pqi].top().getxPos() == xdx &&
						pq[pqi].top().getyPos() == ydy))
					{
						pq[1 - pqi].push(pq[pqi].top());
						pq[pqi].pop();
					}
					pq[pqi].pop(); // remove the wanted node

					// empty the larger size pq to the smaller one
					if (pq[pqi].size() > pq[1 - pqi].size()) pqi = 1 - pqi;
					while (!pq[pqi].empty())
					{
						pq[1 - pqi].push(pq[pqi].top());
						pq[pqi].pop();
					}
					pqi = 1 - pqi;
					pq[pqi].push(*m0); // add the better node instead
				}
				else delete m0; // garbage collection
			}
		}
		delete n0; // garbage collection
	}
	return ""; // no route found
}
//Run my Genetic Algorithm
void PathFinder::GeneticAlgorithm()
{
	//Loop over the GAPathFind method until it returns true, meaning path found
	while (!GAPathfind())
	{
		//Loop through all chromosomes and add up total fitness from the individuals fitness
		for (auto& c : m_chromosomes)
		{
			m_totalFitness += c.m_fitness;
		}
		//Sort the chromosomes in terms of fitness
		std::sort(m_chromosomes.begin(), m_chromosomes.end());
		//Loop through all chromosomes
		for (unsigned i = 0; i < m_chromosomes.size(); i++)
		{
			//Set the chromsomes mating chance to the fitness divided by total fitness, then times is by 100 to make it a whole value
			m_chromosomes.at(i).m_matingChance = 100 * (m_chromosomes.at(i).m_fitness / m_totalFitness);
			try
			{
				//Set the chromosomes mating position to it's mating chance plus the previous chromsomes position (Try catch if there is no previous chromosome
				m_chromosomes.at(i).m_matingPos = (m_chromosomes.at(i).m_matingChance + m_chromosomes.at(i - 1).m_matingPos);
			}
			catch (const std::out_of_range& e)
			{
				//If the above fails, just set the mating positions equal to it's mating chance
				m_chromosomes.at(i).m_matingPos = m_chromosomes.at(i).m_matingChance;
			}
		}
		//Loop until the size of breedChromo matches the size of the population
		while (m_breedChromo.size() != m_populationSize)
		{
			//Run the BreedSelection method
			BreedSelection();
		}
		//Clear the chromosomes vector, ready for new population
		m_chromosomes.clear();
		//Try crossing over chromosomes to add differences
		CrossoverSelection();
		//Try mutating chromosomes to add differences
		MutateSelection();
		//Set the chromosomes vector equal to the breedChromo vector
		m_chromosomes.assign(m_breedChromo.begin(), m_breedChromo.end());
		//Clear the breedChromo vector
		m_breedChromo.clear();
		//Reset total fitness
		m_totalFitness = 0;
		//Add 1 to the generation to keep count
		m_generation += 1;
		//Print a new line to add a line break
		std::cout << std::endl;
		//Print the generation number
		std::cout << "Generation: " << m_generation << std::endl;
	}
	//Make sure the window is shown once path is found
	ShowWindow();
	//End the timer
	m_tEnd = std::chrono::system_clock::now();
	//Get the elapsed time since starting the path finder to finding the goal
	auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(m_tEnd - m_tStart);
	//Print the time taken
	std::cout << "Time taken: " << elapsed.count() << " seconds" << std::endl;
	//Return
	return;
}
//Generate the initial population
void PathFinder::GenerateInitialPopulation()
{
	//Create a random number between 0.0 and 1.0
	std::uniform_real_distribution<double> distribution(0.0, 1.0);
	//Set the chromosomes length (Amount of moves) equal to 2 / 3 * width * height
	m_chromoLength = ((2.0f / 3) * (m_width * m_height));
	//Make sure the number is even (Rounds up)
	(m_chromoLength + 1) & ~1;

	//Generate the population
	for (int i = 0; i < m_populationSize; i++)
	{
		//Create a new chromosome
		Chromosome chromo;
		//Loop through chromolength and set the value to either 0 or 1 depending on the random value above
		for (int j = 0; j < m_chromoLength; j++)
		{
			double r = distribution(m_generator);
			if (r <= 0.5)
				chromo.m_chromo.push_back(0);
			else
				chromo.m_chromo.push_back(1);
		}
		//Set the chromosomes xPos and yPos equal to the start of the map
		chromo.m_xPos = m_xStart;
		chromo.m_yPos = m_yStart;
		//Set the chromosome ID so it can be tracked
		chromo.m_id = i;
		//Add the chromosome to the chromosomes vector
		m_chromosomes.push_back(chromo);
	}
}
//Begin pathfinding using GA
bool PathFinder::GAPathfind()
{
	//Set a value used later
	int i = 0;
	//Loop while all moves have not been complete and chromosomes have not been ran
	while (m_cNum < m_populationSize)
	{
		while (m_cPos < m_chromoLength)
		{
			//Move up
			if (m_chromosomes.at(m_cNum).m_chromo.at(m_cPos) == 0 && m_chromosomes.at(m_cNum).m_chromo.at(m_cPos + 1) == 0)
			{
				int oldX = m_chromosomes.at(m_cNum).m_xPos;
				int oldY = m_chromosomes.at(m_cNum).m_yPos;
				m_chromosomes.at(m_cNum).m_xPos = m_chromosomes.at(m_cNum).m_xPos;
				m_chromosomes.at(m_cNum).m_yPos = m_chromosomes.at(m_cNum).m_yPos - 1;
				m_chromosomes.at(m_cNum).m_chromoMoves.push_back('u');
				//If out of bounds
				if (m_chromosomes.at(m_cNum).m_xPos < 0 || m_chromosomes.at(m_cNum).m_yPos < 0 || m_chromosomes.at(m_cNum).m_xPos > m_width || m_chromosomes.at(m_cNum).m_yPos > m_height)
				{
					m_chromosomes.at(m_cNum).m_xPos = oldX;
					m_chromosomes.at(m_cNum).m_yPos = oldY;
					m_chromosomes.at(m_cNum).m_chromoMoves.pop_back();
				}
				//Hitting wall
				else if (m_map1[m_chromosomes.at(m_cNum).m_yPos][m_chromosomes.at(m_cNum).m_xPos] == 1)
				{
					std::cout << "Hitting wall" << std::endl;
					m_chromosomes.at(m_cNum).m_xPos = oldX;
					m_chromosomes.at(m_cNum).m_yPos = oldY;
					m_chromosomes.at(m_cNum).m_chromoMoves.pop_back();
				}
				else
				{

				}
			}

			//Move down
			if (m_chromosomes.at(m_cNum).m_chromo.at(m_cPos) == 1 && m_chromosomes.at(m_cNum).m_chromo.at(m_cPos + 1) == 0)
			{
				int oldX = m_chromosomes.at(m_cNum).m_xPos;
				int oldY = m_chromosomes.at(m_cNum).m_yPos;
				m_chromosomes.at(m_cNum).m_xPos = m_chromosomes.at(m_cNum).m_xPos;
				m_chromosomes.at(m_cNum).m_yPos = m_chromosomes.at(m_cNum).m_yPos + 1;
				m_chromosomes.at(m_cNum).m_chromoMoves.push_back('d');
				//If out of bounds
				if (m_chromosomes.at(m_cNum).m_xPos < 0 || m_chromosomes.at(m_cNum).m_yPos < 0 || m_chromosomes.at(m_cNum).m_xPos > m_width || m_chromosomes.at(m_cNum).m_yPos > m_height)
				{
					m_chromosomes.at(m_cNum).m_xPos = oldX;
					m_chromosomes.at(m_cNum).m_yPos = oldY;
					std::cout << "Out of bounds" << std::endl;
					m_chromosomes.at(m_cNum).m_chromoMoves.pop_back();
				}
				//Hitting wall
				else if (m_map1[m_chromosomes.at(m_cNum).m_yPos][m_chromosomes.at(m_cNum).m_xPos] == 1)
				{
					std::cout << "Hitting wall" << std::endl;
					m_chromosomes.at(m_cNum).m_xPos = oldX;
					m_chromosomes.at(m_cNum).m_yPos = oldY;
					m_chromosomes.at(m_cNum).m_chromoMoves.pop_back();
				}
				else
				{

				}
			}

			//Move left
			if (m_chromosomes.at(m_cNum).m_chromo.at(m_cPos) == 1 && m_chromosomes.at(m_cNum).m_chromo.at(m_cPos + 1) == 1)
			{
				int oldX = m_chromosomes.at(m_cNum).m_xPos;
				int oldY = m_chromosomes.at(m_cNum).m_yPos;
				m_chromosomes.at(m_cNum).m_xPos = m_chromosomes.at(m_cNum).m_xPos - 1;
				m_chromosomes.at(m_cNum).m_yPos = m_chromosomes.at(m_cNum).m_yPos;
				m_chromosomes.at(m_cNum).m_chromoMoves.push_back('l');
				//If out of bounds
				if (m_chromosomes.at(m_cNum).m_xPos < 0 || m_chromosomes.at(m_cNum).m_yPos < 0 || m_chromosomes.at(m_cNum).m_xPos > m_width || m_chromosomes.at(m_cNum).m_yPos > m_height)
				{
					m_chromosomes.at(m_cNum).m_xPos = oldX;
					m_chromosomes.at(m_cNum).m_yPos = oldY;
					m_chromosomes.at(m_cNum).m_chromoMoves.pop_back();
				}
				//Hitting wall
				else if (m_map1[m_chromosomes.at(m_cNum).m_yPos][m_chromosomes.at(m_cNum).m_xPos] == 1)
				{
					std::cout << "Hitting wall" << std::endl;
					m_chromosomes.at(m_cNum).m_xPos = oldX;
					m_chromosomes.at(m_cNum).m_yPos = oldY;
					m_chromosomes.at(m_cNum).m_chromoMoves.pop_back();
				}
				else
				{

				}
			}

			//Move right
			if (m_chromosomes.at(m_cNum).m_chromo.at(m_cPos) == 0 && m_chromosomes.at(m_cNum).m_chromo.at(m_cPos + 1) == 1)
			{
				int oldX = m_chromosomes.at(m_cNum).m_xPos;
				int oldY = m_chromosomes.at(m_cNum).m_yPos;
				m_chromosomes.at(m_cNum).m_xPos = m_chromosomes.at(m_cNum).m_xPos + 1;
				m_chromosomes.at(m_cNum).m_yPos = m_chromosomes.at(m_cNum).m_yPos;
				m_chromosomes.at(m_cNum).m_chromoMoves.push_back('r');
				//If out of bounds
				if (m_chromosomes.at(m_cNum).m_xPos < 0 || m_chromosomes.at(m_cNum).m_yPos < 0 || m_chromosomes.at(m_cNum).m_xPos > m_width || m_chromosomes.at(m_cNum).m_yPos > m_height)
				{
					m_chromosomes.at(m_cNum).m_xPos = oldX;
					m_chromosomes.at(m_cNum).m_yPos = oldY;
					m_chromosomes.at(m_cNum).m_chromoMoves.pop_back();
				}
				//Hitting wall
				else if (m_map1[m_chromosomes.at(m_cNum).m_yPos][m_chromosomes.at(m_cNum).m_xPos] == 1)
				{
					std::cout << "Hitting wall" << std::endl;
					m_chromosomes.at(m_cNum).m_xPos = oldX;
					m_chromosomes.at(m_cNum).m_yPos = oldY;
					m_chromosomes.at(m_cNum).m_chromoMoves.pop_back();
				}
				else
				{

				}
			}

			//Reached the end/goal
			if (m_chromosomes.at(m_cNum).m_xPos == m_xFinish && m_chromosomes.at(m_cNum).m_yPos == m_yFinish)
			{
				//Print the sucessful chromosome
				std::cout << "Found the chromosome: " << m_chromosomes.at(m_cNum).m_id << std::endl;
				//Loop through the chromosome and print the values of its moves
				for (int i = 0; i < m_chromoLength; i++)
				{
					std::cout << m_chromosomes.at(m_cNum).m_chromo.at(i);
				}
				std::cout << std::endl;
				//Make sure current chromo is equal to the current chromosome
				m_currentChromo = m_chromosomes.at(m_cNum);
				//Return true, ending the loop
				return true;
			}
			//Add 2 to the moves position, going to the next pair of chromosomes
			m_cPos = m_cPos + 2;
		}
		//Set the current chromo equal to the current running chromo
		m_currentChromo = m_chromosomes.at(m_cNum);
		//Use the current chromo to show its path
		ShowWindow();
		//Set the dx and dy values equal to the difference in the position from the finish by the chromosomes position
		int dx = m_xFinish - m_chromosomes.at(m_cNum).m_xPos;
		int dy = m_yFinish - m_chromosomes.at(m_cNum).m_yPos;
		//Set the values to absolute, meaning they cannot be negative
		dx = abs(dx);
		dy = abs(dy);
		//Set the chromosomes fitness 
		m_chromosomes.at(m_cNum).m_fitness = 1.0f / (dx + dy + 1.0f);
		//Check if the chromosome's fitness is infinite meaning error or if it is negative meaning error
		if (std::isinf(m_chromosomes.at(m_cNum).m_fitness))
		{
			std::cout << "Infinite, breaking..." << std::endl;
			m_chromosomes.at(m_cNum).m_fitness = 0.1f;
		}
		else if (m_chromosomes.at(m_cNum).m_fitness < 0)
		{
			std::cout << "Negative, breaking..." << std::endl;
			m_chromosomes.at(m_cNum).m_fitness = 0.01f;
		}

		//Print the chromosomes fitness
		std::cout << "Chromosome: " << i << "'s fitness: " << m_chromosomes.at(m_cNum).m_fitness << std::endl;
		//reset the chromosomes position to the start
		m_chromosomes.at(m_cNum).m_xPos = m_xStart;
		m_chromosomes.at(m_cNum).m_yPos = m_yStart;
		//Iterate i
		i += 1;
		//Clear the current moves of the unsucessful chromosomes
		m_chromosomes.at(m_cNum).m_chromoMoves.clear();
		//Increment cNum to run the next chromosome
		m_cNum = m_cNum + 1;
		//Reset the current position in the chromosome
		m_cPos = 0;
	}
	//Reset the chromosome number
	m_cNum = 0;
	//Clear the chromosomes moves
	m_chromosomes.at(m_cNum).m_chromoMoves.clear();
	//Return false if no chromosome found the goal in the generation
	return false;
}
//Handle breeding of the chromosomes
void PathFinder::BreedSelection()
{
	//Create a random number between 0.0 and 1.0
	std::uniform_real_distribution<float> distribution(0.0, 1.0);
	//Set x equal to the random number
	float x = distribution(m_generator);
	//Times x by 100 to make it a whole number (0.4 becomes 40
	x *= 100;
	//Run through the subsequent statements checking if the random number is in range of one and that it was not last picked
	if (x >= 0 && x <= m_chromosomes.at(0).m_matingPos && m_chromosomes.at(0).m_id != m_lastPicked.m_id)
	{
		//Select chromo 0
		m_breedChromo.push_back(m_chromosomes.at(0));
		m_lastPicked = m_chromosomes.at(0);
		std::cout << "Picked: " << m_chromosomes.at(0).m_id << std::endl;
	}
	else if (x >= m_chromosomes.at(0).m_matingPos && x <= m_chromosomes.at(1).m_matingPos && m_chromosomes.at(1).m_id != m_lastPicked.m_id)
	{
		//Select chromo 1
		m_breedChromo.push_back(m_chromosomes.at(1));
		m_lastPicked = m_chromosomes.at(1);
		std::cout << "Picked: " << m_chromosomes.at(1).m_id << std::endl;
	}
	else if (x >= m_chromosomes.at(1).m_matingPos && x <= m_chromosomes.at(2).m_matingPos && m_chromosomes.at(2).m_id != m_lastPicked.m_id)
	{
		//Select chromo 2
		m_breedChromo.push_back(m_chromosomes.at(2));
		m_lastPicked = m_chromosomes.at(2);
		std::cout << "Picked: " << m_chromosomes.at(2).m_id << std::endl;
	}
	else if (x >= m_chromosomes.at(2).m_matingPos && x <= m_chromosomes.at(3).m_matingPos && m_chromosomes.at(3).m_id != m_lastPicked.m_id)
	{
		//Select chromo 3
		m_breedChromo.push_back(m_chromosomes.at(3));
		m_lastPicked = m_chromosomes.at(3);
		std::cout << "Picked: " << m_chromosomes.at(3).m_id << std::endl;
	}
	else if (x >= m_chromosomes.at(3).m_matingPos && x <= m_chromosomes.at(4).m_matingPos && m_chromosomes.at(4).m_id != m_lastPicked.m_id)
	{
		//Select chromo 4
		m_breedChromo.push_back(m_chromosomes.at(4));
		m_lastPicked = m_chromosomes.at(4);
		std::cout << "Picked: " << m_chromosomes.at(4).m_id << std::endl;
	}
	else if (x >= m_chromosomes.at(4).m_matingPos && x <= m_chromosomes.at(5).m_matingPos && m_chromosomes.at(5).m_id != m_lastPicked.m_id)
	{
		//Select chromo 5
		m_breedChromo.push_back(m_chromosomes.at(5));
		m_lastPicked = m_chromosomes.at(5);
		std::cout << "Picked: " << m_chromosomes.at(5).m_id << std::endl;
	}
	else if (x >= m_chromosomes.at(5).m_matingPos && x <= m_chromosomes.at(6).m_matingPos && m_chromosomes.at(6).m_id != m_lastPicked.m_id)
	{
		//Select chromo 6
		m_breedChromo.push_back(m_chromosomes.at(6));
		m_lastPicked = m_chromosomes.at(6);
		std::cout << "Picked: " << m_chromosomes.at(6).m_id << std::endl;
	}
	else if (x >= m_chromosomes.at(6).m_matingPos && x <= m_chromosomes.at(7).m_matingPos && m_chromosomes.at(7).m_id != m_lastPicked.m_id)
	{
		//Select chromo 7
		m_breedChromo.push_back(m_chromosomes.at(7));
		m_lastPicked = m_chromosomes.at(7);
		std::cout << "Picked: " << m_chromosomes.at(7).m_id << std::endl;
	}
	else if (x >= m_chromosomes.at(7).m_matingPos && x <= m_chromosomes.at(8).m_matingPos && m_chromosomes.at(8).m_id != m_lastPicked.m_id)
	{
		//Select chromo 8
		m_breedChromo.push_back(m_chromosomes.at(8));
		m_lastPicked = m_chromosomes.at(8);
		std::cout << "Picked: " << m_chromosomes.at(8).m_id << std::endl;
	}
	else if (x >= m_chromosomes.at(8).m_matingPos && x <= m_chromosomes.at(9).m_matingPos && m_chromosomes.at(9).m_id != m_lastPicked.m_id)
	{
		//Select chromo 9
		m_breedChromo.push_back(m_chromosomes.at(9));
		m_lastPicked = m_chromosomes.at(9);
		std::cout << "Picked: " << m_chromosomes.at(9).m_id << std::endl;
	}
	else
	{
		//std::cout << "No chromosome chosen" << std::endl;
	}
}
//Handle crossover of the chromosomes
void PathFinder::CrossoverSelection()
{
	//Create a random number between 0.0 and 1.0
	std::uniform_real_distribution<float> distribution(0.0, 1.0);
	//Set the value equal to the random number
	float crossOver = distribution(m_generator);
	//Loop over the population and incrememt 2 each time
	for (int i = 0; i < m_populationSize; i += 2)
	{
		//Check if the random number is greater than the crossover rate
		if (crossOver >= m_crossOverRate)
		{
			//Perform Crossover
			//Create 2 vectors which will store the swapped values
			std::vector<unsigned int> swap1((m_chromoLength / 2));
			std::vector<unsigned int> swap2((m_chromoLength / 2));
			//Iterate over the chromosomes and store the values in the new vectors
			for (int j = (m_chromoLength / 2); j < m_chromoLength; j++)
			{
				swap1.at(j - (m_chromoLength / 2)) = m_breedChromo.at(i).m_chromo.at(j);
				swap2.at(j - (m_chromoLength / 2)) = m_breedChromo.at(i + 1).m_chromo.at(j);
			}
			//Set the stored values to the chromosomes to swap their second halves
			for (int t = (m_chromoLength / 2); t < m_chromoLength; t++)
			{
				m_breedChromo.at(i).m_chromo[t] = swap2.at(t - (m_chromoLength / 2));
				m_breedChromo.at(i + 1).m_chromo[t] = swap1.at(t - (m_chromoLength / 2));
			}

		}
		else
		{
			//Don't crossover
		}

	}


}
//Handle mutation of the chromosomes
void PathFinder::MutateSelection()
{
	//Create a random number between 0.0 and 1.0
	std::uniform_real_distribution<double> distribution(0.0, 1.0);
	//Loop through all chromosomes in breedChromo
	for (auto& c : m_breedChromo)
	{
		//Loop through the length of the chromosome
		for (int i = 0; i < m_chromoLength; i++)
		{
			//Set the value of mutation to the random number
			float mutation = distribution(m_generator);
			//Check if mutation is lower, if so then mutate
			if (mutation <= m_mutationRate)
			{
				//Store the current value as temp
				int temp = c.m_chromo.at(i);
				try
				{
					//Set the chromosomes value to the one ahead of it
					c.m_chromo.at(i) = c.m_chromo.at(i + 1);
					//Use the temp value to swap the values
					c.m_chromo.at(i + 1) = temp;
				}
				catch (const std::exception&)
				{

				}
			}
		}
	}
}
//Handle showing the window
void PathFinder::ShowWindow()
{
	//Check whether A* was used or not
	if (m_isAStar)
	{
		//Loop through while the window is open
		while (m_window.isOpen())
		{
			sf::Event event;
			while (m_window.pollEvent(event))
			{
				if (event.type == sf::Event::Closed)
					m_window.close();
			}
			//Clear the window
			m_window.clear();
			//Create a new rectangle
			sf::RectangleShape rectangle(sf::Vector2f(50.f, 50.f));
			//Run through the map and render the relevant element, depending on the map
			for (int i = 0; i < m_height; i++)
			{
				for (int j = 0; j < m_width; j++)
				{
					switch (m_map1[i][j])
					{
						case 0:
						{
							//Floor
							m_floorSpr.setPosition(sf::Vector2f(j * 50, i * 50));
							m_window.draw(m_floorSpr);
							break;
						}
						case 1:
						{
							//Obstacle
							m_obstacleSpr.setPosition(sf::Vector2f(j * 50, i * 50));
							m_window.draw(m_obstacleSpr);
							break;
						}
						case 2:
						{
							//Start
							m_startSpr.setPosition(sf::Vector2f(j * 50, i * 50));
							m_window.draw(m_startSpr);
							break;
						}
						case 3:
						{
							//Route taken

							rectangle.setPosition(sf::Vector2f(j * 50, i * 50));
							rectangle.setScale(50, 50);
							rectangle.setFillColor(sf::Color(100, 250, 50));
							m_window.draw(rectangle);
							break;
						}
						case 4:
						{
							//Goal
							m_goalSpr.setPosition(sf::Vector2f(j * 50, i * 50));
							m_window.draw(m_goalSpr);
							break;
						}
						default:
							break;
					}

				}
			}

			//Display the window
			m_window.display();
		}

	}
	else
	{
		//Set the successful chromosomes x and y to the start
		m_currentChromo.m_xPos = m_xStart;
		m_currentChromo.m_yPos = m_yStart;
		sf::Event event;
		while (m_window.pollEvent(event))
		{
			if (event.type == sf::Event::Closed)
				m_window.close();
		}
		//Clear the window
		m_window.clear();
		//Create a new rectangle shape
		sf::RectangleShape rectangle(sf::Vector2f(50.f, 50.f));
		//Loop through the map and draw the relevant elements, except the start and goal
		for (int i = 0; i < m_height; i++)
		{
			for (int j = 0; j < m_width; j++)
			{
				switch (m_map1[i][j])
				{
					case 0:
					{
						//Floor
						m_floorSpr.setPosition(sf::Vector2f(j * 50, i * 50));
						m_window.draw(m_floorSpr);
						break;
					}
					case 1:
					{
						//Obstacle
						m_obstacleSpr.setPosition(sf::Vector2f(j * 50, i * 50));
						m_window.draw(m_obstacleSpr);
						break;
					}
					default:
						break;
					}
			}
		}
		//Loop through all the moves stored in the chromosome
		for (int i = 0; i < m_currentChromo.m_chromoMoves.size(); i++)
		{
			//Check the value of the chromosome movement and draw it as a rectangle
			switch (m_currentChromo.m_chromoMoves.at(i))
			{
				case 'u': //Up
				{
					m_currentChromo.m_yPos = m_currentChromo.m_yPos - 1;
					rectangle.setPosition(sf::Vector2f(m_currentChromo.m_xPos * 50, m_currentChromo.m_yPos * 50));
					rectangle.setFillColor(sf::Color::Magenta);
					m_window.draw(rectangle);
					break;
				}
				case 'd': // Down
				{
					m_currentChromo.m_yPos = m_currentChromo.m_yPos + 1;
					rectangle.setPosition(sf::Vector2f(m_currentChromo.m_xPos * 50, m_currentChromo.m_yPos * 50));
					rectangle.setFillColor(sf::Color::Magenta);
					m_window.draw(rectangle);
					break;
				}
				case 'l': //Left
				{
					m_currentChromo.m_xPos = m_currentChromo.m_xPos - 1;
					rectangle.setPosition(sf::Vector2f(m_currentChromo.m_xPos * 50, m_currentChromo.m_yPos * 50));
					rectangle.setFillColor(sf::Color::Magenta);
					m_window.draw(rectangle);
					break;
				}
				case 'r': //Right
				{
					m_currentChromo.m_xPos = m_currentChromo.m_xPos + 1;
					rectangle.setPosition(sf::Vector2f(m_currentChromo.m_xPos * 50, m_currentChromo.m_yPos * 50));
					rectangle.setFillColor(sf::Color::Magenta);
					m_window.draw(rectangle);
					break;
				}
				default:
					break;
				}
		}
		//Loop through the map and draw the start and goal, making sure they render on top
		for (int i = 0; i < m_height; i++)
		{
			for (int j = 0; j < m_width; j++)
			{
				switch (m_map1[i][j])
				{
					case 2:
					{
						//Start
						m_startSpr.setPosition(sf::Vector2f(j * 50, i * 50));
						m_window.draw(m_startSpr);
						break;
					}
					case 3:
					{
						//Goal
						m_goalSpr.setPosition(sf::Vector2f(j * 50, i * 50));
						m_window.draw(m_goalSpr);
						break;
					}
				}
			}
		}
		//Display the window
		m_window.display();
	}
}

