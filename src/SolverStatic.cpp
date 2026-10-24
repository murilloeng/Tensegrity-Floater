//Math
#include "Math/inc/Groups/ASE3.hpp"

//Floater
#include "Tensegrity-Floater/inc/Tensegrity.hpp"
#include "Tensegrity-Floater/inc/SolverStatic.hpp"

//constructor
SolverStatic::SolverStatic(Tensegrity* tensegrity) : m_tensegrity{tensegrity}
{
	m_size = 6;
	m_system = [this](double* r, double* fr, double* K, double p, const double* x)
	{
		//state
		m_tensegrity->m_state_new = m_tensegrity->m_state_old * math::ASE3(x).exp();
		//compute
		m_tensegrity->compute_static(p, r, fr, K);
	};
}

//solve
void SolverStatic::setup(void)
{
	m_tensegrity->setup();
	math::solvers::NewtonRaphson::setup();
}