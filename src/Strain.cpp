//std
#include <cmath>

//Floater
#include "Tensegrity-Floater/inc/Strain.hpp"

double Strain::strain(double s, strain_measure measure)
{
	double (*methods[])(double) = {
		linear_strain, quadratic_strain, logarithmic_strain
	};
	return methods[uint32_t(measure)](s);
}
double Strain::strain_hessian(double s, strain_measure measure)
{
	double (*methods[])(double) = {
		linear_strain_hessian, quadratic_strain_hessian, logarithmic_strain_hessian
	};
	return methods[uint32_t(measure)](s);
}
double Strain::strain_gradient(double s, strain_measure measure)
{
	double (*methods[])(double) = {
		linear_strain_gradient, quadratic_strain_gradient, logarithmic_strain_gradient
	};
	return methods[uint32_t(measure)](s);
}

double Strain::linear_strain(double s)
{
	return s - 1;
}
double Strain::linear_strain_hessian(double)
{
	return 0;
}
double Strain::linear_strain_gradient(double)
{
	return 1;
}

double Strain::quadratic_strain(double s)
{
	return (s * s - 1) / 2;
}
double Strain::quadratic_strain_hessian(double)
{
	return 1;
}
double Strain::quadratic_strain_gradient(double s)
{
	return s;
}

double Strain::logarithmic_strain(double s)
{
	return log(s);
}
double Strain::logarithmic_strain_hessian(double s)
{
	return -1 / s / s;
}
double Strain::logarithmic_strain_gradient(double s)
{
	return 1 / s;
}