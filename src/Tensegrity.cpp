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
	m_order{3}, m_inelastic{false}, m_twist_angle{0}, m_yield_stress{4.00e+08}, m_height_inner{1.50e-01}, 
	m_height_outer{3.00e-01}, m_cables_diameter{1.50e-03}, m_residual_stress{2.00e+08}, m_elastic_modulus{2.00e+11}, 
	m_state_old{0, 0, m_height_outer}, m_state_new{0, 0, m_height_outer}, m_solver_static{this}, m_solver_dynamic{this}, m_strain_measure{Strain::strain_measure::quadratic}
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

SolverStatic& Tensegrity::solver_static(void)
{
	return m_solver_static;
}
SolverDynamic& Tensegrity::solver_dynamic(void)
{
	return m_solver_dynamic;
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

//formulation
void Tensegrity::inertia(double* M) const
{
	return;
}
void Tensegrity::damping(double* C) const
{
	return;
}
void Tensegrity::stiffness(double* K) const
{
	//data
	math::Vec3 z2, x1, x2, tk;
	const math::Mat3 I(math::mode::eye);
	math::Matrix A(3, 6, math::mode::eye);
	math::Matrix Nk(3, 6, math::mode::eye);
	const math::Vec3 zr(0, 0, m_height_outer);
	const math::Mat3 Rr = m_state_new.gso3().matrix();
	const math::ASE3 pr = (m_state_old.inverse() * m_state_new).log();
	//setup
	A.zeros();
	A.span(0, 3).eye();
	//stiffness
	math::Matrix(K, 6, 6).zeros();
	math::Vector f(6, math::mode::zeros);
	for(uint32_t k = 0; k <= m_order; k++)
	{
		//positions
		cable_position(z2.data(), k, 1, 0);
		cable_position(x1.data(), k, 0, 1);
		cable_position(x2.data(), k, 1, 1);
		//cable
		const double fk = cable_force(k);
		const double Lk = (x2 - x1).norm();
		const double Kk = cable_stiffness(k);
		//stiffness
		tk = (x2 - x1).unit();
		Nk.span(0, 3) = -(z2 - zr).spin();
		f += fk * Nk.transpose() * Rr.transpose() * tk;
		math::Matrix(K, 6, 6) += fk * Nk.transpose() * Rr.transpose() * tk.spin() * Rr * A;
		math::Matrix(K, 6, 6) += Kk * Nk.transpose() * Rr.transpose() * tk.outer(tk) * Rr * Nk;
		math::Matrix(K, 6, 6) += fk * Nk.transpose() * Rr.transpose() * (I - tk.outer(tk)) * Rr * Nk / Lk;
	}
	//parametrization
	const math::Matrix Jr = pr.jacobian();
	const math::Matrix Gr = pr.jacobian_gradient(f);
	math::Matrix(K, 6, 6) = Jr.transpose() * math::Matrix(K, 6, 6) * Jr + Gr;
}

void Tensegrity::internal_force(double* fi) const
{
	//data
	math::Vec3 z2, x1, x2, tk;
	math::Matrix Nk(3, 6, math::mode::eye);
	const math::Vec3 zr(0, 0, m_height_outer);
	const math::ASE3 pr = (m_state_old.inverse() * m_state_new).log();
	//internal force
	math::Vector(fi, 6).zeros();
	for(uint32_t k = 0; k <= m_order; k++)
	{
		const double fk = cable_force(k);
		cable_position(z2.data(), k, 1, 0);
		cable_position(x1.data(), k, 0, 1);
		cable_position(x2.data(), k, 1, 1);
		tk = (x2 - x1).unit();
		Nk.span(0, 3) = -(z2 - zr).spin();
		math::Vector(fi, 6) += fk * Nk.transpose() * m_state_new.gso3().inverse().action(tk);
	}
	math::Vector(fi, 6) = pr.jacobian().transpose() * math::Vector(fi, 6);
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
	double U = 0;
	for(uint32_t i = 0; i <= m_order; i++)
	{
		U += cable_energy(i);
	}
	return U;
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