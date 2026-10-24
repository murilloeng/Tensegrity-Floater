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
		tensegrity.order(6);
		tensegrity.create_cables();
		tensegrity.map_vertical("map-vertical.dat", 100, 100, -1e3);
	}
	catch(const std::exception& exception)
	{
		printf("%s\n", exception.what());
	}
	//return
	return EXIT_SUCCESS;
}