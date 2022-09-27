#pragma once
#include "constDef.hpp"
#include <math.h>
#include "TimeCoordConvert.hpp"
#include "Matrix.hpp"

namespace NUDTTK
{
	void  Saastamoinen_model(double temperature,double humidity,double pressure,double latitude,double height,double &zpdh,double &zpdw);
	void  GlobalPT(double dmjd,double latitude,double longitude,double height,double &pressure,double &temperature,double &undu);
	void  GlobalPT_IERS(double dmjd,double latitude,double longitude,double height,double &pressure,double &temperature,double &undu);
	void  GlobalMF(double dmjd,double latitude,double longitude,double height,double elevation,double &gmfh,double &gmfw);
	void  Troposphere_correct(double dmjd,POS3D station_position,double elevation, double &temperature,double &pressure,double &trop_delay);

}