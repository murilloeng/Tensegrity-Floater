//std
#include <cmath>

//Tensegrity
#include "Tensegrity-Plate/inc/Tensegrity.hpp"

//constructors
Tensegrity::Tensegrity(void) : 
	m_order{3}, m_inelastic{false}, m_rod_length{5.00e-02}, m_twist_angle{0}, m_yield_stress{4.00e+08}, m_height_total{3.20e-01}, 
	m_height_center{1.40e-01}, m_rod_thickness{1.00e-02},m_plate_diameter{2.80e-01}, m_plate_thickness{5.00e-03}, m_cables_diameter{1.50e-03},
	m_residual_stress{2.00e+08}, m_elastic_modulus{2.00e+11}, m_strain_measure{Strain::strain_measure::quadratic}
{
	return;
}

//destructor
Tensegrity::~Tensegrity(void)
{
	return;
}

//data
double Tensegrity::tension(double fr)
{
	const double dc = m_cables_diameter;
	m_residual_stress = fr / (M_PI_4 * dc * dc);
	return fr;
}
double Tensegrity::tension(void) const
{
	const double dc = m_cables_diameter;
	const double sr = m_residual_stress;
	return sr * M_PI_4 * dc * dc;
}

uint32_t Tensegrity::order(void) const
{
	return m_order;
}
uint32_t Tensegrity::order(uint32_t order)
{
	return m_order = order;
}

double Tensegrity::rod_length(void) const
{
	return m_rod_length;
}
double Tensegrity::rod_length(double rod_length)
{
	return m_rod_length = rod_length;
}

double Tensegrity::rod_thickness(void) const
{
	return m_rod_thickness;
}
double Tensegrity::rod_thickness(double rod_thickness)
{
	return m_rod_thickness = rod_thickness;
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

double Tensegrity::plate_diameter(void) const
{
	return m_plate_diameter;
}
double Tensegrity::plate_diameter(double plate_diameter)
{
	return m_plate_diameter = plate_diameter;
}

double Tensegrity::plate_thickness(void) const
{
	return m_plate_thickness;
}
double Tensegrity::plate_thickness(double plate_thickness)
{
	return m_plate_thickness = plate_thickness;
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

Strain::strain_measure Tensegrity::strain_measure(void) const
{
	return m_strain_measure;
}
Strain::strain_measure Tensegrity::strain_measure(Strain::strain_measure strain_measure)
{
	return m_strain_measure = strain_measure;
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
	return 0;
}
double Tensegrity::cable_stiffness(uint32_t index) const
{
	return 0;
}