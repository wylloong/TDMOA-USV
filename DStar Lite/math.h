/**
 * Math.
 *
 * @package		DStarLite
 * @author		Aaron Zampaglione <azampagl@gmail.com>
 * @copyright	Copyright (C) 2011 Aaron Zampaglione
 * @license		MIT
 */
#ifndef DSTARLITE_MATH_H
#define DSTARLITE_MATH_H

#include <cmath>
#include <cfloat>

namespace DStarLite
{
	class Math
	{
		public:

			/**
			 * @var  double  INFINITY
			 */
			static const double INF;

			/**
			 * @var  double  SQRT2
			 */
			static const double SQRT2;

			/**
			 * Determines if two doubles are equal based on a precision.
			 *
			 * @param   double              first double
			 * @param   double              second duble
			 * @param   double [optional]   precision
			 * @return  bool
			 */
			//wyl: 在头文件为某个变量赋值，则允许为空
			static bool equals(double a, double b, double precision = 0.000000000000001);

			/**
			 * Determines if a double is greater than compared to another double
			 * based on a precision.
			 *
			 * @param   double              first double
			 * @param   double              second duble
			 * @param   double [optional]   precision
			 * @return  bool
			 */
			static bool greater(double a, double b, double precision = 0.000000000000001);

			/**
			 * Determines if a double is less than compared to another double
			 * based on a precision.
			 *
			 * @param   double              first double
			 * @param   double              second duble
			 * @param   double [optional]   precision
			 * @return  bool
			 */
			static bool less(double a, double b, double precision = 0.000000000000001);
	};
};


#endif // DSTARLITE_MATH_H
