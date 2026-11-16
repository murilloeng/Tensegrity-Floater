//Math
#include "Math/inc/Linear/Matrix.hpp"

//Tensegrity
#include "Tensegrity-Floater/inc/Load.hpp"
#include "Tensegrity-Floater/inc/Tensegrity.hpp"

//constructor
Load::Load(Tensegrity* tensegrity) : m_value{0, 0, 0}, m_position{0, 0}, m_tensegrity{tensegrity}
{
	return;
}

//data
double Load::value(uint32_t index) const
{
	return m_value[index];
}
double Load::value(uint32_t index, double value)
{
	return m_value[index] = value;
}

double Load::position(uint32_t index) const
{
	return m_position[index];
}
double Load::position(uint32_t index, double position)
{
	return m_position[index] = position;
}

//analysis
void Load::compute_static(double p, double& V, math::Vec6& fr, math::Mat6& K) const
{
	//data
	const math::Vec3 fk(m_value);
	const math::Vec3 zk(m_position[0], m_position[1], 0);
	const math::Mat3 Rr = m_tensegrity->m_state_new.gso3().matrix();
	math::Matrix<3, 6> Nk(math::Mode::eye), Bk(math::Mode::zeros);
	//energy
	V -= m_tensegrity->m_state_new.action(zk).inner(fk);
	//force
	Nk.span<3, 3>(0, 3) = -zk.spin();
	fr += Nk.transpose() * Rr.transpose() * fk;
	//stiffness
	Bk.span<3, 3>(0, 3).eye();
	K -= p * Nk.transpose() * Rr.transpose() * fk.spin() * Rr * Bk;
}	