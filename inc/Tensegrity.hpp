#pragma once

//Math
#include "Math/inc/Groups/GSE3.hpp"

//Floater
#include "Tensegrity-Floater/inc/Strain.hpp"
#include "Tensegrity-Floater/inc/SolverStatic.hpp"
#include "Tensegrity-Floater/inc/SolverDynamic.hpp"

class Tensegrity
{
public:
	//constructors
	Tensegrity(void);

	//destructor
	virtual ~Tensegrity(void) = default;

	//data
	uint32_t order(uint32_t);
	uint32_t order(void) const;

	double twist_angle(double);
	double twist_angle(void) const;

	double yield_stress(double);
	double yield_stress(void) const;

	double height_inner(double);
	double height_inner(void) const;

	double height_outer(double);
	double height_outer(void) const;

	double cables_diameter(double);
	double cables_diameter(void) const;

	double residual_stress(double);
	double residual_stress(void) const;

	double elastic_modulus(double);
	double elastic_modulus(void) const;

	SolverStatic& solver_static(void);
	SolverDynamic& solver_dynamic(void);

	Strain::strain_measure strain_measure(void) const;
	Strain::strain_measure strain_measure(Strain::strain_measure);

protected:
	//analysis
	void setup(void);

	//formulation
	void inertia(double*) const;
	void damping(double*) const;
	void stiffness(double*) const;
	void internal_force(double*) const;
	void external_force(double*) const;

	double kinetic_energy(void) const;
	double internal_energy(void) const;
	double potential_energy(void) const;

	double cable_force(uint32_t) const;
	double cable_energy(uint32_t) const;
	double cable_stiffness(uint32_t) const;

	virtual void cable_position(double*, uint32_t, bool, bool) const = 0;

	//data
	uint32_t m_order;
	bool m_inelastic;
	double m_twist_angle;
	double m_yield_stress;
	double m_height_inner;
	double m_height_outer;
	double m_cables_diameter;
	double m_residual_stress;
	double m_elastic_modulus;

	math::GSE3 m_state_old;
	math::GSE3 m_state_new;
	SolverStatic m_solver_static;
	SolverDynamic m_solver_dynamic;
	Strain::strain_measure m_strain_measure;
};