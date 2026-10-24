#pragma once

//Floater
#include "Tensegrity-Floater/inc/Tensegrity.hpp"

class Circle : public Tensegrity
{
public:
	//constructor
	Circle(void);

	//destructor
	~Circle(void) = default;

	//data
	double radius(double);
	double radius(void) const;


private:
	//cables
	uint32_t cables_count(void) const override;
	math::Vec3 cable_position(uint32_t, bool, bool) const override;

	//data
	double m_radius;
};