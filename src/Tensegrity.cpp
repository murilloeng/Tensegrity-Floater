//std
#include <cmath>
#include <cstdio>
#include <stdexcept>

//Math
#include "Math/inc/Groups/ASO3.hpp"
#include "Math/inc/Groups/ASE3.hpp"
#include "Math/inc/Linear/Matrix.hpp"

//Floater
#include "Tensegrity-Floater/inc/Tensegrity.hpp"

//static
static const math::solvers::StopCriteria::Type load_max = math::solvers::StopCriteria::Type::LoadLimitMaximum;

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
void Tensegrity::map_radial(const char* file_name, uint32_t nr, uint32_t nt, double F)
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
	if(!file) throw std::runtime_error("Unable to open map file!");
	//map
	m_solver_static.silent(true);
	for(uint32_t i = 0; i <= nr; i++)
	{
		for(uint32_t j = 0; j <= nt; j++)
		{
			//data
			const double r = R * i / nr;
			const double t = 2 * M_PI * j / nt;
			//load
			m_loads[0].value(0, F * cos(t));
			m_loads[0].value(1, F * sin(t));
			m_loads[0].position(0, r * cos(t));
			m_loads[0].position(1, r * sin(t));
			//solve
			m_solver_static.solve();
			const math::ASO3 q = m_state_new.gso3().log();
			const bool status = m_solver_static.status() && m_solver_static.stop_criteria().stop_type() == load_max;
			//print
			fprintf(file, "%+.6e %+.6e %d ", r * cos(t), r * sin(t), status);
			fprintf(file, "%+.6e %+.6e %+.6e %+.6e %+.6e %+.6e\n", x1, x2, x3 - H, q[0], q[1], q[2]);
			printf("Progress: %.2f\r", double(i * (nt + 1) + j) / (nt + 1) / (nr + 1));
			fflush(stdout);
		}
		fprintf(file, "\n");
	}
	//close
	fclose(file);
}
void Tensegrity::map_angular(const char* file_name, uint32_t nr, uint32_t nt, double F)
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
	if(!file) throw std::runtime_error("Unable to open map file!");
	//map
	m_solver_static.silent(true);
	for(uint32_t i = 0; i <= nr; i++)
	{
		for(uint32_t j = 0; j <= nt; j++)
		{
			//data
			const double r = R * i / nr;
			const double t = 2 * M_PI * j / nt;
			//load
			m_loads[0].value(0, -F * sin(t));
			m_loads[0].value(1, +F * cos(t));
			m_loads[0].position(0, r * cos(t));
			m_loads[0].position(1, r * sin(t));
			//solve
			m_solver_static.solve();
			const math::ASO3 q = m_state_new.gso3().log();
			const bool status = m_solver_static.status() && m_solver_static.stop_criteria().stop_type() == load_max;
			//print
			fprintf(file, "%+.6e %+.6e %d ", r * cos(t), r * sin(t), status);
			fprintf(file, "%+.6e %+.6e %+.6e %+.6e %+.6e %+.6e\n", x1, x2, x3 - H, q[0], q[1], q[2]);
			printf("Progress: %.2f\r", double(i * (nt + 1) + j) / (nt + 1) / (nr + 1));
			fflush(stdout);
		}
		fprintf(file, "\n");
	}
	//close
	fclose(file);
}
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
	if(!file) throw std::runtime_error("Unable to open map file!");
	//map
	m_loads[0].value(2, F);
	m_solver_static.silent(true);
	for(uint32_t i = 0; i <= nr; i++)
	{
		for(uint32_t j = 0; j <= nt; j++)
		{
			//data
			const double r = R * i / nr;
			const double t = 2 * M_PI * j / nt;
			//load
			m_loads[0].position(0, r * cos(t));
			m_loads[0].position(1, r * sin(t));
			//solve
			m_solver_static.solve();
			const math::ASO3 q = m_state_new.gso3().log();
			const bool status = m_solver_static.status() && m_solver_static.stop_criteria().stop_type() == load_max;
			//print
			fprintf(file, "%+.6e %+.6e %d ", r * cos(t), r * sin(t), status);
			fprintf(file, "%+.6e %+.6e %+.6e %+.6e %+.6e %+.6e\n", x1, x2, x3 - H, q[0], q[1], q[2]);
			printf("Progress: %.2f\r", double(i * (nt + 1) + j) / (nt + 1) / (nr + 1));
			fflush(stdout);
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
void Tensegrity::update(void)
{
	m_state_old = m_state_new;
	for(Cable& cable : m_cables) cable.update();
}
void Tensegrity::restore(void)
{
	m_state_new = m_state_old;
	for(Cable& cable : m_cables) cable.restore();
}
void Tensegrity::compute_static(double p, double* r, double* f, double* K)
{
	//data
	double U = 0, V = 0;
	math::Vec6 rm(math::Mode::zeros);
	math::Vec6 fm(math::Mode::zeros);
	math::Mat6 Km(math::Mode::zeros);
	const math::ASE3 ar = (m_state_old.inverse() * m_state_new).log();
	//compute
	for(const Cable& cable : m_cables) cable.compute(U, rm, Km);
	for(const Load& load : m_loads) load.compute_static(p, V, fm, Km);
	//group
	rm = p * fm - rm;
	const math::Matrix<6, 6> Jr = ar.jacobian();
	const math::Matrix<6, 6> Gr = ar.jacobian_gradient(rm);
	//parametrization
	rm = Jr.transpose() * rm;
	fm = Jr.transpose() * fm;
	Km = Jr.transpose() * Km * Jr - Gr;
	//write
	rm.write(r);
	fm.write(f);
	Km.write(K);
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