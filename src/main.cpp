//std
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>

//Floater
#include "Tensegrity-Floater/inc/Tensegrity.hpp"

int main(void)
{
	try
	{
		//data
		Tensegrity tensegrity;
		//setup
		tensegrity.order(3);
		tensegrity.create_cables();
		//solve
		tensegrity.map_radial("data/radial/3x1.00e2.dat", 100, 100, 1.00e2);
	}
	catch(const std::exception& exception)
	{
		printf("%s\n", exception.what());
	}
	//return
	return EXIT_SUCCESS;
}