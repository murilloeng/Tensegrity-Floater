//Floater
#include "Tensegrity-Floater/inc/Tensegrity.hpp"
#include "Tensegrity-Floater/inc/SolverDynamic.hpp"

//constructor
SolverDynamic::SolverDynamic(Tensegrity* tensegrity) : m_tensegrity{tensegrity}
{
	m_size = 6;
}

//solve
void SolverDynamic::setup(void)
{
	m_tensegrity->setup();
	math::solvers::Newmark::setup();
}