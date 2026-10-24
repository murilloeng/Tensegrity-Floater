#pragma once

//std
#include <cstdint>

//definitions
class Tensegrity;

class Cable
{
public:
	//constructor
	Cable(Tensegrity*, uint32_t);

	//destructor
	~Cable(void) = default;

	//data
	double diameter(double);
	double diameter(void) const;

	double yield_stress(double);
	double yield_stress(void) const;

	double elastic_modulus(double);
	double elastic_modulus(void) const;

	double plastic_modulus(double);
	double plastic_modulus(void) const;

private:
	//analysis
	void update(void);
	void restore(void);
	void compute(double&, double*, double*) const;

	//data
	uint32_t m_index;
	double m_diameter;
	double m_yield_stress;
	double m_elastic_modulus;
	double m_plastic_modulus;
	double m_residual_stress;
	double m_plastic_strain_old;
	double m_plastic_strain_new;

	Tensegrity* m_tensegrity;

	//friends
	friend class Tensegrity;
};