#pragma once

//Math
#include "Math/inc/Solvers/NewtonRaphson.hpp"

class Tensegrity;

class SolverStatic : public math::solvers::NewtonRaphson
{
public:
	//constructor
	SolverStatic(Tensegrity*);

	//destructor
	~SolverStatic(void) = default;

private:
	//data
	Tensegrity* m_tensegrity;
};