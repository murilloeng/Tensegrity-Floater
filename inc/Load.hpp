#pragma once

//std
#include <cstdint>
#include <functional>

//Math
#include "Math/inc/Linear/Matrix.hpp"

//definitions
class Tensegrity;

class Load
{
public:
	//constructor
	Load(Tensegrity*);

	//destructor
	~Load(void) = default;

	//data
	double value(uint32_t) const;
	double value(uint32_t, double);

	double position(uint32_t) const;
	double position(uint32_t, double);

private:
	//analysis
	void compute_static(double, double&, math::Vec6&, math::Mat6&) const;

	//data
	double m_value[3];
	double m_position[2];
	Tensegrity* m_tensegrity;
	std::function<double(double)> m_time_variation;

	//friends
	friend class Tensegrity;
};