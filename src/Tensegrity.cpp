//std
#include <cmath>

//Math
#include "Math/inc/Linear/Vec3.hpp"
#include "Math/inc/Linear/Mat3.hpp"
#include "Math/inc/Groups/ASE3.hpp"

//Floater
#include "Tensegrity-Floater/inc/Tensegrity.hpp"

//constructors
Tensegrity::Tensegrity(void) : 
	m_order{3}, m_inelastic{false}, m_twist_angle{0}, m_height_inner{1.50e-01}, m_height_outer{3.00e-01}, 
	m_residual_stress{2.00e+08}, m_solver_static{this}, m_solver_dynamic{this}, m_strain_measure{Strain::strain_measure::quadratic}
{
	return;
}

//data
uint32_t Tensegrity::order(void) const
{
	return m_order;
}
uint32_t Tensegrity::order(uint32_t order)
{
	return m_order = order;
}

double Tensegrity::twist_angle(void) const
{
	return m_twist_angle;
}
double Tensegrity::twist_angle(double twist_angle)
{
	return m_twist_angle = twist_angle;
}

double Tensegrity::height_inner(void) const
{
	return m_height_inner;
}
double Tensegrity::height_inner(double height_center)
{
	return m_height_inner = height_center;
}

double Tensegrity::height_outer(void) const
{
	return m_height_outer;
}
double Tensegrity::height_outer(double height_total)
{
	return m_height_outer = height_total;
}

double Tensegrity::residual_stress(void) const
{
	return m_residual_stress;
}
double Tensegrity::residual_stress(double residual_stress)
{
	return m_residual_stress = residual_stress;
}

SolverStatic& Tensegrity::solver_static(void)
{
	return m_solver_static;
}
SolverDynamic& Tensegrity::solver_dynamic(void)
{
	return m_solver_dynamic;
}

std::vector<Load>& Tensegrity::loads(void)
{
	return m_loads;
}
const std::vector<Load>& Tensegrity::loads(void) const
{
	return m_loads;
}

void Tensegrity::create_cables(void)
{
	//data
	const uint32_t nc = cables_count();
	//create
	m_cables.clear();
	m_cables.reserve(nc);
	for(uint32_t i = 0; i < nc; i++)
	{
		m_cables.push_back(Cable(this, i));
	}
}
std::vector<Cable>& Tensegrity::cables(void)
{
	return m_cables;
}
const std::vector<Cable>& Tensegrity::cables(void) const
{
	return m_cables;
}

Strain::strain_measure Tensegrity::strain_measure(void) const
{
	return m_strain_measure;
}
Strain::strain_measure Tensegrity::strain_measure(Strain::strain_measure strain_measure)
{
	return m_strain_measure = strain_measure;
}

//analysis
void Tensegrity::setup(void)
{
	m_state_old = m_state_new = math::GSE3(0, 0, m_height_outer);
}
void Tensegrity::compute_static(double p, double* r, double* fr, double* K)
{
	//data
	double U = 0, V = 0;
	const math::ASE3 ar = (m_state_old.inverse() * m_state_new).log();
	//compute
	math::Vector(r, 6).zeros();
	math::Vector(fr, 6).zeros();
	math::Matrix(K, 6, 6).zeros();
	for(const Cable& cable : m_cables) cable.compute(U, r, K);
	for(const Load& load : m_loads) load.compute_static(p, V, fr, K);
	for(uint32_t i = 0; i < 6; i++) r[i] = p * fr[i] - r[i];
	//parametrization
	const math::Matrix Jr = ar.jacobian();
	const math::Matrix Gr = ar.jacobian_gradient(math::Vector(r, 6));
	math::Vector(r, 6) = Jr.transpose() * math::Vector(r, 6);
	math::Vector(fr, 6) = Jr.transpose() * math::Vector(fr, 6);
	math::Matrix(K, 6, 6) = Jr.transpose() * math::Matrix(K, 6, 6) * Jr - Gr;
}