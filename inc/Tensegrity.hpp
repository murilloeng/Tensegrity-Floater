#pragma once

//Math
#include "Math/inc/Linear/Vec3.hpp"
#include "Math/inc/Groups/GSE3.hpp"

//Floater
#include "Tensegrity-Floater/inc/Load.hpp"
#include "Tensegrity-Floater/inc/Cable.hpp"
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

	bool inelastic(bool);
	bool inelastic(void) const;

	double diameter(double);
	double diameter(void) const;

	double twist_angle(double);
	double twist_angle(void) const;

	double height_inner(double);
	double height_inner(void) const;

	double height_outer(double);
	double height_outer(void) const;

	double residual_stress(double);
	double residual_stress(void) const;
	
	SolverStatic& solver_static(void);
	SolverDynamic& solver_dynamic(void);

	std::vector<Load>& loads(void);
	const std::vector<Load>& loads(void) const;

	void create_cables(void);
	std::vector<Cable>& cables(void);
	const std::vector<Cable>& cables(void) const;

	Strain::strain_measure strain_measure(void) const;
	Strain::strain_measure strain_measure(Strain::strain_measure);

	//maps
	void map_vertical(const char*, uint32_t, uint32_t, double);

protected:
	//analysis
	void setup(void);
	void compute_modes(void);
	void compute_static(double, double*, double*, double*);

	//positions
	math::Vec3 cable_position(uint32_t, bool, bool) const;

	//data
	uint32_t m_order;
	bool m_inelastic;
	double m_diameter;
	double m_twist_angle;
	double m_height_inner;
	double m_height_outer;
	double m_residual_stress;
	uint32_t m_modes_counter;

	math::GSE3 m_state_old;
	math::GSE3 m_state_new;
	std::vector<Load> m_loads;
	std::vector<Cable> m_cables;
	std::vector<double> m_modes;
	SolverStatic m_solver_static;
	SolverDynamic m_solver_dynamic;
	Strain::strain_measure m_strain_measure;

	//friends
	friend class Load;
	friend class Cable;
	friend class SolverStatic;
	friend class SolverDynamic;
};