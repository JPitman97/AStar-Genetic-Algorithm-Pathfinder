//  @main.cpp
//  @Creates an instance of PathFinder and calls it's "start" method, as well as seeding the generator
#include "PathFinder.h"

// Module Contents
int main()
{
	//Fix GA, choosing bad routes
	std::shared_ptr<PathFinder> pathFinder(new PathFinder());
	pathFinder->m_generator.seed(time(NULL));
	pathFinder->Start();

	std::cout << "Press Enter to exit" << std::endl;
	getchar(); // wait for a (Enter) keypress  
	getchar(); // wait for a (Enter) keypress  

	return 0;
}

