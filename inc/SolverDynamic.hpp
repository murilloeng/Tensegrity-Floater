#pragma once

//Math
#include "Math/inc/Solvers/Newmark.hpp"

class Tensegrity;

class SolverDynamic : public math::solvers::Newmark
{
public:
	//constructor
	SolverDynamic(Tensegrity*);

	//destructor
	~SolverDynamic(void) = default;

private:
	//data
	Tensegrity* m_tensegrity;
};