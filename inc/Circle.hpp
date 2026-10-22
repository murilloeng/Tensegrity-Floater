#pragma once

//Tensegrity
#include "Tensegrity-Plate/inc/Tensegrity.hpp"

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

	//formulation
	void cable_position(double*, uint32_t, bool, bool) const override;

private:
	//data
	double m_radius;
};