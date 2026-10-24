#pragma once

//Math
#include "Math/inc/Solvers/Newmark.hpp"

//definitions
class Tensegrity;

class SolverDynamic : public math::solvers::Newmark
{
public:
	//constructor
	SolverDynamic(Tensegrity*);

	//destructor
	~SolverDynamic(void) = default;

private:
	//solve
	void setup(void) override;

	//data
	Tensegrity* m_tensegrity;
};