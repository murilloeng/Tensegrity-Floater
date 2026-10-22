//std
#include <cmath>

//Tensegrity
#include "Tensegrity-Plate/inc/Circle.hpp"

//constructor
Circle::Circle(void) : m_radius{1.50e-01}
{
	return;
}

//data
double Circle::radius(void) const
{
	return m_radius;
}
double Circle::radius(double radius)
{
	return m_radius = radius;
}

//formulation
void Circle::cable_position(double* x, uint32_t index, bool layer, bool configuration) const
{
	//data
	const double R = m_radius;
	const double ht = m_height_total;
	const double hc = m_height_center;
	const double ta = m_twist_angle * layer;
	const double ti = 2 * M_PI * index / m_order;
	//position
	x[0] = index == 0 ? 0 : R * cos(ti + ta);
	x[1] = index == 0 ? 0 : R * sin(ti + ta);
	x[2] = index != 0 ? layer * ht : (hc + ht) / 2 - layer * hc;
	if(layer && configuration) x[2] -= ht, math::Vector(x, 3) = m_state_new.action(math::Vector(x, 3));
}