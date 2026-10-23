//std
#include <cmath>

//Math
#include "Math/inc/Linear/Vec3.hpp"

//Tensegrity
#include "Tensegrity-Floater/inc/Tensegrity.hpp"

//constructors
Tensegrity::Tensegrity(void) : 
	m_order{3}, m_inelastic{false}, m_twist_angle{0}, m_yield_stress{4.00e+08}, m_height_total{3.00e-01}, 
	m_height_center{1.50e-01}, m_cables_diameter{1.50e-03}, m_residual_stress{2.00e+08}, m_elastic_modulus{2.00e+11}, 
	m_state_old{0, 0, m_height_total}, m_state_new{0, 0, m_height_total}, m_strain_measure{Strain::strain_measure::quadratic}
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

double Tensegrity::yield_stress(void) const
{
	return m_yield_stress;
}
double Tensegrity::yield_stress(double yield_stress)
{
	return m_yield_stress = yield_stress;
}

double Tensegrity::height_total(void) const
{
	return m_height_total;
}
double Tensegrity::height_total(double height_total)
{
	return m_height_total = height_total;
}

double Tensegrity::height_center(void) const
{
	return m_height_center;
}
double Tensegrity::height_center(double height_center)
{
	return m_height_center = height_center;
}

double Tensegrity::cables_diameter(void) const
{
	return m_cables_diameter;
}
double Tensegrity::cables_diameter(double cables_diameter)
{
	return m_cables_diameter = cables_diameter;
}

double Tensegrity::residual_stress(void) const
{
	return m_residual_stress;
}
double Tensegrity::residual_stress(double residual_stress)
{
	return m_residual_stress = residual_stress;
}

double Tensegrity::elastic_modulus(void) const
{
	return m_elastic_modulus;
}
double Tensegrity::elastic_modulus(double elastic_modulus)
{
	return m_elastic_modulus = elastic_modulus;
}

math::solvers::Newmark& Tensegrity::solver_dynamic(void)
{
	return m_solver_dynamic;
}
math::solvers::NewtonRaphson& Tensegrity::solver_static(void)
{
	return m_solver_static;
}

Strain::strain_measure Tensegrity::strain_measure(void) const
{
	return m_strain_measure;
}
Strain::strain_measure Tensegrity::strain_measure(Strain::strain_measure strain_measure)
{
	return m_strain_measure = strain_measure;
}

//solve
void Tensegrity::solve_static(void)
{
	setup();
	m_solver_static.solve();
}
void Tensegrity::solve_dynamic(void)
{
	setup();
	m_solver_dynamic.solve();
}

//analysis
void Tensegrity::setup(void)
{
	m_state_old = m_state_new = math::GSE3(0, 0, m_height_total);
}

//formulation
void Tensegrity::inertia(double* K) const
{
	return;
}
void Tensegrity::damping(double* K) const
{
	return;
}
void Tensegrity::stiffness(double* K) const
{
	return;
}

void Tensegrity::internal_force(double* fi) const
{
	return;
}
void Tensegrity::external_force(double* fe) const
{
	return;
}

double Tensegrity::kinetic_energy(void) const
{
	return 0;
}
double Tensegrity::internal_energy(void) const
{
	return 0;
}
double Tensegrity::potential_energy(void) const
{
	return 0;
}

double Tensegrity::cable_force(uint32_t index) const
{
	//data
	math::Vec3 z1, z2, x1, x2;
	const double E = m_elastic_modulus;
	const double sr = m_residual_stress;
	const double dc = m_cables_diameter;
	//position
	cable_position(z1.data(), index, 0, 0);
	cable_position(z2.data(), index, 1, 0);
	cable_position(x1.data(), index, 0, 1);
	cable_position(x2.data(), index, 1, 1);
	//lengths
	const double Lr = (z2 - z1).norm();
	const double Ln = (x2 - x1).norm();
	//strain
	const double an = Ln / Lr;
	const double en = Strain::strain(an, m_strain_measure);
	const double gn = Strain::strain_gradient(an, m_strain_measure);
	//stress
	const double Ac = M_PI_4 * dc * dc;
	const double sn = fmax(E * en + sr, 0);
	//force
	return sn * gn * Ac * (E * en + sr > 0);
}
double Tensegrity::cable_energy(uint32_t index) const
{
	//data
	math::Vec3 z1, z2, x1, x2;
	const double E = m_elastic_modulus;
	const double sr = m_residual_stress;
	const double dc = m_cables_diameter;
	//position
	cable_position(z1.data(), index, 0, 0);
	cable_position(z2.data(), index, 1, 0);
	cable_position(x1.data(), index, 0, 1);
	cable_position(x2.data(), index, 1, 1);
	//lengths
	const double Lr = (z2 - z1).norm();
	const double Ln = (x2 - x1).norm();
	//strain
	const double an = Ln / Lr;
	const double en = Strain::strain(an, m_strain_measure);
	//stress
	const double Ac = M_PI_4 * dc * dc;
	const double sn = fmax(E * en + sr, 0);
	//energy
	return sn * sn * Ac * Lr / E / 2;
}
double Tensegrity::cable_stiffness(uint32_t index) const
{
	//data
	math::Vec3 z1, z2, x1, x2;
	const double E = m_elastic_modulus;
	const double sr = m_residual_stress;
	const double dc = m_cables_diameter;
	//position
	cable_position(z1.data(), index, 0, 0);
	cable_position(z2.data(), index, 1, 0);
	cable_position(x1.data(), index, 0, 1);
	cable_position(x2.data(), index, 1, 1);
	//lengths
	const double Lr = (z2 - z1).norm();
	const double Ln = (x2 - x1).norm();
	//strain
	const double an = Ln / Lr;
	const double en = Strain::strain(an, m_strain_measure);
	const double hn = Strain::strain_hessian(an, m_strain_measure);
	const double gn = Strain::strain_gradient(an, m_strain_measure);
	//stress
	const double Ac = M_PI_4 * dc * dc;
	const double sn = fmax(E * en + sr, 0);
	//stiffness
	return (E * gn * gn + sn * hn) * Ac / Lr * (E * en + sr > 0);
}