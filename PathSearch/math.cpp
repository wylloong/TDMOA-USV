/**
 * DStarLite.
 *
 * @package		DStarLite
 * @author		Aaron Zampaglione <azampagl@gmail.com>
 * @copyright	Copyright (C) 2011 Aaron Zampaglione
 * @license		MIT
 */
#include "math.h"

using namespace DStarLite;

/**
 * @var  double  INF
 */
const double Math::INF = DBL_MAX;  // max value

/**
 * @var  double  SQRT2
 */
const double Math::SQRT2 = 1.41421356237309504880;

/**
 * Determines if two doubles are equal based on a precision.
 *
 * @param   double              first double
 * @param   double              second double
 * @param   double [optional]   precision
 * @return  bool
 */
bool Math::equals(double a, double b, double precision)
{
	if (a == Math::INF && b == Math::INF)
		return true;
	
	return (fabs(a - b) < precision);
}

/**
 * Determines if a double is greater than compared to another double
 * based on a precision.
 *
 * @param   double              first double
 * @param   double              second double
 * @param   double [optional]   precision
 * @return  bool
 */
bool Math::greater(double a, double b, double precision)
{
	if (a == Math::INF && b == Math::INF)
		return false;
	
	return a - precision > b;
}

/**
 * Determines if a double is less than compared to another double
 * based on a precision.
 *
 * @param   double              first double
 * @param   double              second double
 * @param   double [optional]   precision
 * @return  bool
 */
bool Math::less(double a, double b, double precision)
{
	if (a == Math::INF && b == Math::INF)
		return false;
	
	return a + precision < b;
}
