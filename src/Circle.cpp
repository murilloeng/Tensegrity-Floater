//std
#include <cmath>

//Floater
#include "Tensegrity-Floater/inc/Circle.hpp"

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
uint32_t Circle::cables_count(void) const
{
	return m_order + 1;
}
math::Vec3 Circle::cable_position(uint32_t index, bool layer, bool configuration) const
{
	//data
	math::Vec3 x;
	const double R = m_radius;
	const double ht = m_height_outer;
	const double hc = m_height_inner;
	const double ta = m_twist_angle * layer;
	const double ti = 2 * M_PI * index / m_order;
	//position
	x[0] = index == 0 ? 0 : R * cos(ti + ta);
	x[1] = index == 0 ? 0 : R * sin(ti + ta);
	x[2] = index != 0 ? layer * ht : (hc + ht) / 2 - layer * hc;
	if(layer && configuration) x[2] -= ht, x = m_state_new.action(x);
	//return
	return x;
}