#pragma once

//Math
#include "Math/inc/Solvers/NewtonRaphson.hpp"

//definitions
class Tensegrity;

class SolverStatic : public math::solvers::NewtonRaphson
{
public:
	//constructor
	SolverStatic(Tensegrity*);

	//destructor
	~SolverStatic(void) = default;

private:
	//solve
	void setup(void) override;

	//data
	Tensegrity* m_tensegrity;
};