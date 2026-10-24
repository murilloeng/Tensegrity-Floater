//std
#include <cstdio>

//Math
#include "Math/inc/Groups/ASE3.hpp"

//Floater
#include "Tensegrity-Floater/inc/Tensegrity.hpp"
#include "Tensegrity-Floater/inc/SolverStatic.hpp"

//constructor
SolverStatic::SolverStatic(Tensegrity* tensegrity) : m_tensegrity{tensegrity}
{
	allocate(6);
	m_system = [this](double* r, double* fr, double* K, double p, const double* x)
	{
		//state
		m_tensegrity->m_state_new = m_tensegrity->m_state_old * math::ASE3(x).exp();
		//compute
		m_tensegrity->compute_static(p, r, fr, K);
	};
	m_callback_update = [this](void){
		math::Vector(m_x_new,m_size).zeros();
		m_tensegrity->m_state_old = m_tensegrity->m_state_new;
	};
	m_callback_restore = [this](void){
		m_tensegrity->m_state_new = m_tensegrity->m_state_old;
	};
}

//solve
void SolverStatic::setup(void)
{
	m_p_old = 0;
	m_tensegrity->setup();
	math::Vector(m_x_old, m_size).zeros();
	math::solvers::NewtonRaphson::setup();
}