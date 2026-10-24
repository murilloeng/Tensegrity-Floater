//std
#include <cmath>
#include <cstdio>
#include <stdexcept>

//Math
#include "Math/inc/Eigen/SVD.hpp"
#include "Math/inc/Linear/Vec3.hpp"
#include "Math/inc/Linear/Mat3.hpp"
#include "Math/inc/Groups/ASO3.hpp"
#include "Math/inc/Groups/ASE3.hpp"

//Floater
#include "Tensegrity-Floater/inc/Tensegrity.hpp"

//constructors
Tensegrity::Tensegrity(void) : 
	m_order{3}, m_inelastic{false}, m_diameter{3.00e-01}, m_twist_angle{0}, m_height_inner{1.50e-01}, m_height_outer{3.00e-01}, 
	m_residual_stress{2.00e+08}, m_solver_static{this}, m_solver_dynamic{this}, m_strain_measure{Strain::strain_measure::quadratic}
{
	return;
}

//data
uint32_t Tensegrity::order(void) const
{
	return m_order;
}
uint32_t Tensegrity::order(uint32_t order)
{
	return m_order = order;
}

bool Tensegrity::inelastic(void) const
{
	return m_inelastic;
}
bool Tensegrity::inelastic(bool inelastic)
{
	return m_inelastic = inelastic;
}

double Tensegrity::diameter(void) const
{
	return m_diameter;
}
double Tensegrity::diameter(double diameter)
{
	return m_diameter = diameter;
}

double Tensegrity::twist_angle(void) const
{
	return m_twist_angle;
}
double Tensegrity::twist_angle(double twist_angle)
{
	return m_twist_angle = twist_angle;
}

double Tensegrity::height_inner(void) const
{
	return m_height_inner;
}
double Tensegrity::height_inner(double height_center)
{
	return m_height_inner = height_center;
}

double Tensegrity::height_outer(void) const
{
	return m_height_outer;
}
double Tensegrity::height_outer(double height_total)
{
	return m_height_outer = height_total;
}

double Tensegrity::residual_stress(void) const
{
	return m_residual_stress;
}
double Tensegrity::residual_stress(double residual_stress)
{
	return m_residual_stress = residual_stress;
}

SolverStatic& Tensegrity::solver_static(void)
{
	return m_solver_static;
}
SolverDynamic& Tensegrity::solver_dynamic(void)
{
	return m_solver_dynamic;
}

std::vector<Load>& Tensegrity::loads(void)
{
	return m_loads;
}
const std::vector<Load>& Tensegrity::loads(void) const
{
	return m_loads;
}

void Tensegrity::create_cables(void)
{
	m_cables.clear();
	m_cables.reserve(m_order + 1);
	for(uint32_t i = 0; i < m_order + 1; i++) m_cables.push_back(Cable(this, i));
}
std::vector<Cable>& Tensegrity::cables(void)
{
	return m_cables;
}
const std::vector<Cable>& Tensegrity::cables(void) const
{
	return m_cables;
}

Strain::strain_measure Tensegrity::strain_measure(void) const
{
	return m_strain_measure;
}
Strain::strain_measure Tensegrity::strain_measure(Strain::strain_measure strain_measure)
{
	return m_strain_measure = strain_measure;
}

//maps
void Tensegrity::map_vertical(const char* file_name, uint32_t nr, uint32_t nt, double F)
{
	//data
	m_loads.clear();
	m_loads.push_back(Load(this));
	const double R = m_diameter / 2;
	const double H = m_height_outer;
	const double& x1 = m_state_new[0];
	const double& x2 = m_state_new[1];
	const double& x3 = m_state_new[2];
	FILE* file = fopen(file_name, "w");
	if(!file) throw std::runtime_error("Unable to open vertical map file!");
	//map
	m_loads[0].value(2, F);
	m_solver_static.silent(true);
	for(uint32_t i = 0; i <= nr; i++)
	{
		for(uint32_t j = 0; j <= nt; j++)
		{
			//position
			const double r = R * i / nr;
			const double t = 2 * M_PI * j / nt;
			m_loads[0].position(0, r * cos(t));
			m_loads[0].position(1, r * sin(t));
			//solve
			m_solver_static.solve();
			const math::ASO3 q = m_state_new.gso3().log();
			if(m_solver_static.step() != m_solver_static.step_max() + 1) printf("failed!\n");
			fprintf(file, "%+.6e %+.6e %d ", r * cos(t), r * sin(t), m_solver_static.status());
			fprintf(file, "%+.6e %+.6e %+.6e %+.6e %+.6e %+.6e\n", x1, x2, x3 - H, q[0], q[1], q[2]);
			printf("Progress: %.2f\n", double(i * (nt + 1) + j) / (nt + 1) / (nr + 1));
		}
		fprintf(file, "\n");
	}
	//close
	fclose(file);
}

//analysis
void Tensegrity::setup(void)
{
	//data
	const uint32_t n = m_order;
	const double sr = m_residual_stress;
	const double dr = m_cables[0].m_diameter;
	//cables
	for(Cable& cable : m_cables)
	{
		const double dc = cable.m_diameter;
		cable.m_residual_stress = cable.m_index == 0 ? sr : sr * dr * dr / (n * dc * dc);
	}
	//state
	m_state_old = m_state_new = math::GSE3(0, 0, m_height_outer);
}
void Tensegrity::compute_modes(void)
{
	//data
	const double cutoff = 1.00e-10;
	const uint32_t m = m_order + 1;
	math::Matrix Ae(6, m_order + 1);
	math::Matrix Nt(6, 3, math::mode::eye);
	const math::Vec3 zr(0, 0, m_height_outer);
	const uint32_t n = std::min(6U, m_order + 1);
	//equilibrium
	for(const Cable& cable : m_cables)
	{
		//positions
		const math::Vec3 z1 = cable_position(cable.m_index, 0, 0);
		const math::Vec3 z2 = cable_position(cable.m_index, 1, 0);
		//direction
		Nt.span(3, 0) = (z2 - zr).spin();
		Ae.span(0, cable.m_index, 6, 1) = Nt * (z2 - z1).unit();
	}
	//decomposition
	uint32_t rank = 0;
	double U[36], s[n], V[m * m];
	math::eigen::SVD(Ae.data(), 6, m, s, U, V).compute();
	for(uint32_t i = 0; i < n; i++) rank += s[i] > cutoff * s[0];
	//modes
	m_modes.clear();
	m_modes_counter = m - rank;
	for(uint32_t i = rank; i < m; i++)
	{
		for(uint32_t j = 0; j < m; j++)
		{
			m_modes.push_back(V[i * m + j]);
		}
	}
	//print
	math::Matrix(m_modes.data(), m, m_modes_counter).print();
}
void Tensegrity::compute_static(double p, double* r, double* fr, double* K)
{
	//data
	double U = 0, V = 0;
	const math::ASE3 ar = (m_state_old.inverse() * m_state_new).log();
	//compute
	math::Vector(r, 6).zeros();
	math::Vector(fr, 6).zeros();
	math::Matrix(K, 6, 6).zeros();
	for(const Cable& cable : m_cables) cable.compute(U, r, K);
	for(const Load& load : m_loads) load.compute_static(p, V, fr, K);
	for(uint32_t i = 0; i < 6; i++) r[i] = p * fr[i] - r[i];
	//parametrization
	const math::Matrix Jr = ar.jacobian();
	const math::Matrix Gr = ar.jacobian_gradient(math::Vector(r, 6));
	math::Vector(r, 6) = Jr.transpose() * math::Vector(r, 6);
	math::Vector(fr, 6) = Jr.transpose() * math::Vector(fr, 6);
	math::Matrix(K, 6, 6) = Jr.transpose() * math::Matrix(K, 6, 6) * Jr - Gr;
}

//positions
math::Vec3 Tensegrity::cable_position(uint32_t index, bool layer, bool configuration) const
{
	//data
	math::Vec3 x;
	const double R = m_diameter / 2;
	const double ht = m_height_outer;
	const double hc = m_height_inner;
	const double ti = 2 * M_PI * index / m_order;
	const double ta = m_twist_angle * layer * (index % 2 ? +1 : -1);
	//position
	x[0] = index == 0 ? 0 : R * cos(ti + ta);
	x[1] = index == 0 ? 0 : R * sin(ti + ta);
	x[2] = index != 0 ? layer * ht : (hc + ht) / 2 - layer * hc;
	if(layer && configuration) x[2] -= ht, x = m_state_new.action(x);
	//return
	return x;
}