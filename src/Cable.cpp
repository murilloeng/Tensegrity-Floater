//std
#include <cmath>

//Math
#include "Math/inc/Linear/Vec3.hpp"
#include "Math/inc/Linear/Mat3.hpp"

//Tensegrity
#include "Tensegrity-Floater/inc/Cable.hpp"
#include "Tensegrity-Floater/inc/Tensegrity.hpp"

//constructor
Cable::Cable(Tensegrity* tensegrity, uint32_t index) : 
	m_index{index}, m_diameter{1.50e-03}, m_yield_stress{4.00e+08}, m_elastic_modulus{2.10e+11}, m_plastic_modulus{0}, 
	m_residual_stress{2.00e+08}, m_plastic_strain_old{0}, m_plastic_strain_new{0}, m_tensegrity{tensegrity}
{
	return;
}

//data
double Cable::diameter(void) const
{
	return m_diameter;
}
double Cable::diameter(double diameter)
{
	return m_diameter = diameter;
}

double Cable::yield_stress(void) const
{
	return m_yield_stress;
}
double Cable::yield_stress(double yield_stress)
{
	return m_yield_stress = yield_stress;
}

double Cable::elastic_modulus(void) const
{
	return m_elastic_modulus;
}
double Cable::elastic_modulus(double elastic_modulus)
{
	return m_elastic_modulus = elastic_modulus;
}

double Cable::plastic_modulus(void) const
{
	return m_plastic_modulus;
}
double Cable::plastic_modulus(double plastic_modulus)
{
	return m_plastic_modulus = plastic_modulus;
}

//compute
void Cable::update(void)
{
	m_plastic_strain_old = m_plastic_strain_new;
}
void Cable::restore(void)
{
	m_plastic_strain_new = m_plastic_strain_old;
}
void Cable::compute(double& U, double* fi, double* K) const
{
	//data
	const double dc = m_diameter;
	const double Em = m_elastic_modulus;
	const double sr = m_residual_stress;
	const math::Mat3 I(math::mode::eye);
	const math::Vec3 zr(0, 0, m_tensegrity->m_height_outer);
	const math::Mat3 Rr = m_tensegrity->m_state_new.gso3().matrix();
	const math::Vec3 z1 = m_tensegrity->cable_position(m_index, 0, 0);
	const math::Vec3 z2 = m_tensegrity->cable_position(m_index, 1, 0);
	const math::Vec3 x1 = m_tensegrity->cable_position(m_index, 0, 1);
	const math::Vec3 x2 = m_tensegrity->cable_position(m_index, 1, 1);
	math::Matrix Nc(3, 6, math::mode::eye), Bc(3, 6, math::mode::zeros);
	//lengths
	const double Lr = (z2 - z1).norm();
	const double Ln = (x2 - x1).norm();
	const math::Vec3 tc = (x2 - x1).unit();
	//strain
	const double an = Ln / Lr;
	const double en = Strain::strain(an, m_tensegrity->m_strain_measure);
	const double hn = Strain::strain_hessian(an, m_tensegrity->m_strain_measure);
	const double gn = Strain::strain_gradient(an, m_tensegrity->m_strain_measure);
	//stress
	const double Ac = M_PI_4 * dc * dc;
	const double sn = fmax(Em * en + sr, 0);
	//energy
	U += sn * sn * Ac * Lr / Em / 2;
	//force
	Nc.span(0, 3) = -(z2 - zr).spin();
	const double fc = sn * gn * Ac * (Em * en + sr > 0);
	math::Vector(fi, 6) += fc * Nc.transpose() * Rr.transpose() * tc;
	//stiffness
	Bc.span(0, 3).eye();
	const double Kc = (Em * gn * gn + sn * hn) * Ac / Lr * (Em * en + sr > 0);
	math::Matrix(K, 6, 6) += fc * Nc.transpose() * Rr.transpose() * tc.spin() * Rr * Bc;
	math::Matrix(K, 6, 6) += Kc * Nc.transpose() * Rr.transpose() * tc.outer(tc) * Rr * Nc;
	math::Matrix(K, 6, 6) += fc * Nc.transpose() * Rr.transpose() * (I - tc.outer(tc)) * Rr * Nc / Ln;
}