#include "SLRPreproc.hpp"
#include <math.h>
#include "LeoGPSObsPreproc.hpp"
#include "TimeCoordConvert.hpp"
#include "OceanTidesLoading.hpp"
#include "SolidTides.hpp"
#include "GNSSBasicCorrectFunc.hpp"

using namespace NUDTTK::SpaceborneGPSPreproc;
namespace NUDTTK
{
	namespace SLR
	{
		SLRPreproc::SLRPreproc(void)
		{
		}

		SLRPreproc::~SLRPreproc(void)
		{
		}

		// 子程序名称： tropCorrect_Marini_IERS2003 
		// 功能：对流层折射修正的 Marini 计算模型
		// 变量类型：Temperature  :  大气温度(K)
		//           Pressure     :  大气压强(mb)
		//           Humidity     :  相对湿度(%)
		//           Wavelength   :  激光的波长(微米)
		//           E            :  卫星仰角(度)
		//           fai          :  大地纬度(度)
		//           h            :  大地高程(米)
		//           Alpha        :  对流层折射修正因子，一般情况下取 1
		// 输入： Temperature，Pressure, Humidity，Wavelength, Alpha
		// 输出： dR_trop
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2007/09/05
		// 版本时间：2007/09/05
		// 修改记录：
		// 备注：
		double SLRPreproc::tropCorrect_Marini_IERS2003(double Temperature, double Pressure, double Humidity, double Wavelength, double E, double fai, double h, double Alpha)
		{
			E   = E * PI / 180;       // 卫星仰角
			fai = fai * PI / 180;     // 大地纬度
			double P  = Pressure;     // 大气压强
			double T  = Temperature;  // 大气温度
			double W  = Humidity;     // 相对湿度
			// 参考 IERS 1996
			// double Pw = ( W / 100 ) * 6.11 * pow(10.0, 7.5 * ( T - 273.15 )/( 273.3 + ( T - 273.15 ))); // 测站的水蒸汽压强（mb）
			// 参考 IERS 2003, 2013/06/11
			double es  = 0.01 * exp(1.2378847E-5 * T * T - 1.9121316E-2 * T + 33.93711047 - 6.3431645E+3 / T);
			double fw  = 1.00062 + 3.14E-6 * P + 5.6E-7 * (T - 273.15) * (T - 273.15);
			double Pw = ( W / 100 ) * es * fw;
			double K  = 1.163 - 0.00968 * cos( 2 * fai ) - 0.00104 * T + 0.00001435 * P;
			double A  = 0.002357 * P + 0.000141 * Pw;
			double B  = 1.084 * 1.0E-8 * P * T * K + 2 * 4.734 * 1.0E-8 *P * P /(T * (3.0 - 1.0 / K));
			double f_lamda = 0.9650 + 0.0164 / pow(Wavelength, 2) + 0.000228 / pow(Wavelength, 4);
			double f_fai_h = 1 - 0.0026 * cos( 2 * fai ) - 3.1E-7 * h;
			double dR_trop = Alpha * ( f_lamda / f_fai_h ) * ( A + B)/( sin(E) + (B / (A + B)) / (sin(E) + 0.01) ) ;
			return dR_trop;
		}

		// 子程序名称： tropCorrect_Marini_IERS2010 
		// 功能：对流层折射修正的 iers 2010 计算模型
		// 变量类型：Temperature  :  大气温度(K)
		//           Pressure     :  大气压强(mb)
		//           Humidity     :  相对湿度(%)
		//           Wavelength   :  激光的波长(微米)
		//           E            :  卫星仰角(度)
		//           fai          :  大地纬度(度)
		//           h            :  大地高程(米)
		//           Alpha        :  对流层折射修正因子, 一般情况下取 1
		// 输入： Temperature，Pressure, Humidity，Wavelength, Alpha
		// 输出： dR_trop
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2013/06/12
		// 版本时间：2013/06/12
		// 修改记录：
		// 备注：参考 FCULZD_HPA.F, FCUL_A.F(来自ftp://tai.bipm.org/iers/conv2010/chapter9)
		double SLRPreproc::tropCorrect_Marini_IERS2010(double Temperature, double Pressure, double Humidity, double Wavelength, double E, double fai, double h, double Alpha)
		{
			// Water vapor pressure in hPa
			// Calculated from the surface temperature (Celsius) and Relative Humidity (% R.H.) at the station.
			double es  = 0.01 * exp(1.2378847E-5 * Temperature * Temperature - 1.9121316E-2 * Temperature + 33.93711047 - 6.3431645E+3 / Temperature);
			double fw  = 1.00062 + 3.14E-6 * Pressure + 5.6E-7 * (Temperature - 273.15) * (Temperature - 273.15);
			double wvp = (Humidity / 100) * es * fw;
			fai = fai * PI / 180; // 大地纬度
			E = E * PI / 180;     // 卫星仰角
			// 测试用例:
			// 输入: fai = 30.67166667 deg 
			//       h   = 2010.344 meters 
			//       Pressure = 798.4188 hPa (August 14, 2009)
			//       wvp = 14.322
			//       Wavelength = 0.532 micrometers
			// 输出: FCUL_ZTD = 1.935225924846803114 m
			//       FCUL_ZHD = 1.932992176591644462 m
			//       FCUL_ZWD = 0.2233748255158703871E-02 m
			//fai = 30.67166667 * PI / 180; h = 2010.344; Pressure = 798.4188; wvp = 14.322; Wavelength = 0.532;
			// co2 content in ppm
			double xc = 375.0;
		    // constant values to be used in Equation (9.5)
		    // k1 and k3 are k1* and k3* 
			double k0 = 238.0185;
			double k1 = 19990.975;   
			double k2 = 57.362;
			double k3 = 579.55174;
            // constant values to be used in Equation (9.7)
			double w0 =  295.235;
			double w1 =  2.6422;
			double w2 = -0.032380;
			double w3 =  0.004028;
            // Wave number
            double sigma = 1.0 / Wavelength;  
            // correction factor - Equation (9.4)
            double f = 1 - 0.00266 * cos(2 * fai) - 0.00028E-3 * h;
            // correction for CO2 content
            double corr_co2 = 1.0 + 0.534E-6*(xc - 450);
            // dispersion equation for the hydrostatic component - Equation (9.5)
		    double fh = 0.01 * corr_co2 * (k1 * (k0 + pow(sigma, 2))/pow(k0 - pow(sigma, 2), 2) + k3 * (k2 + pow(sigma, 2))/pow(k2 - pow(sigma, 2), 2));
			// computation of the hydrostatic component - Equation (9.3)
			// caution: pressure in hectoPascal units
            double fcul_zhd = 2.416579E-3 * fh * Pressure / f;
            // dispersion equation for the non-hydrostatic component - Equation (9.7)
			double fnh = 0.003101 * (w0 + 3.0 * w1 * pow(sigma, 2) + 5.0 * w2 * pow(sigma, 4) + 7.0 * w3 * pow(sigma, 6));
            // computation of the non-hydrostatic component - Equation (9.6)
            // caution: pressure in hectoPascal units
            double fcul_zwd = 1.0E-4 * (5.316 * fnh - 3.759 * fh) * wvp / f; // wvp / f
            // compute the zenith total delay
            double fcul_ztd = fcul_zhd + fcul_zwd;
			// 测试用例:
			// 输入: fai = 30.67166667 deg 
			//       h   = 2075 meters (mean sea level)
			//       T   = 300.15 Kelvin (August 12, 2009)
			//       E   = 15 deg 
			// 输出: fcul_a = 3.800243667312344087
			//E = 15 * PI / 180; fai = 30.67166667 * PI / 180; h = 2075; Temperature = 300.15;
            double SINE = sin(E);
            // Convert temperature to Celsius
            double T_C = Temperature - 273.15;
            double COSFAI = cos(fai);
			// Define coefficients used in the model        
			double A10 =  0.121008E-02;
			double A11 =  0.17295E-05;
			double A12 =  0.3191E-04;
			double A13 = -0.18478E-07;
            double A20 =  0.304965E-02;
            double A21 =  0.2346E-05;
            double A22 = -0.1035E-03;
            double A23 = -0.1856E-07;
            double A30 =  0.68777E-01;
            double A31 =  0.1972E-04;
            double A32 = -0.3458E-02;
            double A33 =  0.1060E-06;
            // a, b, and c in Marini continued fraction (Eq. 9.10)
            double a1 = A10 + A11 * T_C + A12 * COSFAI + A13 * h;
            double a2 = A20 + A21 * T_C + A22 * COSFAI + A23 * h;
            double a3 = A30 + A31 * T_C + A32 * COSFAI + A33 * h;
			// numerator in continued fraction
            double map_zen = (1.0 + a1 / (1.0 + a2 / (1.0 + a3)));
            double fcul_a = map_zen /(SINE + a1 /(SINE + a2 / (SINE + a3)));
			double dR_trop = fcul_ztd * fcul_a;
			return dR_trop;
		}

		// 子程序名称： relativityCorrect 
		// 功能：广义相对论修正, 在一个统一的地心坐标系下进行
		// 变量类型：sunPos          :  太阳在地心系位置（米）
		//           leoPos          :  卫星在地心系位置（米）
		//           staPos          :  测站在地心系位置（米）
		//           Gamma           :  相对论修正因子，一般情况下取1
		// 输入： sunPos, leoPos, staPos
		// 输出： dR_relativity
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2007/09/05
		// 版本时间：2007/09/05
		// 修改记录：
		// 备注：
		double SLRPreproc::relativityCorrect(POS3D sunPos, POS3D leoPos, POS3D staPos, double Gamma)
		{
			double cc = pow(SPEED_LIGHT, 2); // 光速的平方
			// 太阳引力场引起的相对论修正
			POS3D  v_sun_leo = sunPos - leoPos;
			double r1  = sqrt(v_sun_leo.x * v_sun_leo.x + v_sun_leo.y * v_sun_leo.y + v_sun_leo.z * v_sun_leo.z); // 太阳到卫星的距离
			POS3D  v_sun_sta = sunPos - staPos;
			double r2  = sqrt(v_sun_sta.x * v_sun_sta.x + v_sun_sta.y * v_sun_sta.y + v_sun_sta.z * v_sun_sta.z); // 太阳到观测站的距离
			POS3D  v_leo_sta = leoPos - staPos;
			double r   = sqrt(v_leo_sta.x * v_leo_sta.x + v_leo_sta.y * v_leo_sta.y + v_leo_sta.z * v_leo_sta.z); // 卫星到观测站的距离
			double dR1 = (GM_SUN / cc) * log((r1 + r2 + r)/(r1 + r2 - r));
			// 地球引力场引起的相对论修正
			r1 = sqrt(leoPos.x * leoPos.x + leoPos.y * leoPos.y + leoPos.z * leoPos.z); // 地心到卫星的距离
			r2 = sqrt(staPos.x * staPos.x + staPos.y * staPos.y + staPos.z * staPos.z); // 地心到观测站的距离
			double dR2 = (GM_EARTH / cc) * log((r1 + r2 + r)/(r1 + r2 - r));
			//double dR_relativity = (1 + Gamma) * (dR1 + dR2);
			double dR_relativity = (1 + Gamma) * (dR2); // 地心框架下仅需考虑地球这一项, 太阳框架下才需要修正太阳项, 2013/12/05, 谷德峰修改
			return dR_relativity; 
		}

		// 子程序名称： staEccCorrect 
		// 功能：测站偏心修正（与卫星姿态无关）
		// 变量类型：staPos          :  测站在地心系下位置（ITRF, 米）
		//           leoPos          :  卫星在地心系下位置（ITRF, 米）
		//           offset          :  测站天线在测站系(东北天)下偏移（ITRF, 米）
		// 输入： staPos, leoPos, offset
		// 输出： dR_StaARPOffsetCorrect
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2007/09/05
		// 版本时间：2007/09/05
		// 修改记录：
		// 备注：
		double SLRPreproc::staEccCorrect(POS3D staPos, POS3D leoPos, ENU offset)
		{
			// 根据测站位置，计算东北天坐标系
			POS3D S_V;       // 垂直径向
			S_V =  staPos;
			POS3D S_N;       // 北方向
			S_N.x = 0;
			S_N.y = 0;
			S_N.z = EARTH_R; // 北极点
			POS3D S_E;       // 东方向
			vectorCross(S_E,S_N,S_V);
			vectorCross(S_N,S_V,S_E);
			S_E = vectorNormal(S_E);
			S_N = vectorNormal(S_N);
			S_V = vectorNormal(S_V);
			//计算卫星在测站坐标系下的位置矢量
			POS3D  v_leo_sta = leoPos - staPos;
			double r  = sqrt(v_leo_sta.x * v_leo_sta.x + v_leo_sta.y * v_leo_sta.y + v_leo_sta.z * v_leo_sta.z); // 卫星到观测站的距离
			double dE = vectorDot(v_leo_sta, S_E);
			double dN = vectorDot(v_leo_sta, S_N);
			double dV = vectorDot(v_leo_sta, S_V);
			double dR_StaEccCorrect = -(dE * offset.E + dN * offset.N + dV * offset.U)/r;
			return dR_StaEccCorrect;
		}

		// 子程序名称： satMassCenterCorrect_ECEF 
		// 功能：卫星质心修正
		// 变量类型：t            :  轨道时间, 用于计算轨道系
        //           staPos       :  测站在地心系下位置（ITRF, 米）
		//           leoPos       :  卫星在地心系下位置速度（ITRF, 米）
		//           offset       :  卫星激光反射器在星体下偏移量（星体系, 米）
		// 输入： t, staPos, leoPos, offset
		// 输出： dR_LeoMcc
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2007/12/31
		// 版本时间：2007/12/31
		// 修改记录：
		// 备注：调用 GNSSBasicCorrectFunc::correctLeoAntPCO_ECEF
		double SLRPreproc::satMassCenterCorrect_ECEF(UT1 t, POS3D staPos, POS6D leoPos, POS3D offset)
		{
			// 计算天线在 ECEF 系下的偏移量
			POS3D  AntOffset_ECEF = GNSSBasicCorrectFunc::correctLeoAntPCO_ECEF(t, offset, leoPos.getPos(), leoPos.getVel());
			// 计算视线矢量
			POS3D los = vectorNormal(leoPos.getPos() - staPos);
			double dR_LeoMcc = vectorDot(AntOffset_ECEF, los);
			return dR_LeoMcc;
		}

		// 子程序名称： satMassCenterCorrect_J2000 
		// 功能：卫星质心修正
		// 变量类型：t            :  轨道时间, 用于计算轨道系
        //           staPos       :  测站在地心系下位置（J2000, 米）
		//           leoPos       :  卫星在地心系下位置速度（J2000, 米）
		//           offset       :  卫星激光反射器在星体下偏移量（星体系, 米）
		// 输入： staPos, leoPos, offset
		// 输出： dR_LeoMcc
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2015/11/25
		// 版本时间：2015/11/25
		// 修改记录：
		// 备注：调用 GNSSBasicCorrectFunc::satMassCenterCorrect_J2000
		double SLRPreproc::satMassCenterCorrect_J2000(POS3D staPos, POS6D leoPV, POS3D offset)
		{
			// 计算天线在 J2000 系下的偏移量
			POS3D  AntOffset_J2000 = GNSSBasicCorrectFunc::correctLeoAntPCO_J2000(offset, leoPV.getPos(), leoPV.getVel());
			// 计算视线矢量
			POS3D los = vectorNormal(leoPV.getPos() - staPos);
			double dR_LeoMcc = vectorDot(AntOffset_J2000, los);
			return dR_LeoMcc;
		}

		// 子程序名称： satMassCenterCorrect_YawAttitudeModel 
		// 功能：存在偏航姿态控制的卫星的质心修正
		// 变量类型：
        //           staPos       :  测站在地心系下位置（J2000, 米）
		//           leoPos       :  卫星在地心系下位置速度（J2000, 米）
		//           sunPos       :  太阳位置（J2000, 米）
		//           offset       :  卫星激光反射器在星体下偏移量（星体系, 米）
		// 输入： staPos, leoPos, sunPos, offset
		// 输出： dR_LeoMcc
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2013/12/04
		// 版本时间：2013/12/04
		// 修改记录：
		// 备注：调用 GNSSBasicCorrectFunc::correctLeoAntPCO_YawAttitudeModel
		double SLRPreproc::satMassCenterCorrect_YawAttitudeModel(POS3D staPos, POS3D leoPos, POS3D sunPos, POS3D offset)
		{
			// 计算天线在 J2000 系下的偏移量
			POS3D  AntOffset_J2000 = GNSSBasicCorrectFunc::correctLeoAntPCO_YawAttitudeModel(offset, leoPos, sunPos);
			// 计算视线矢量
			POS3D los = vectorNormal(leoPos - staPos);
			double dR_LeoMcc = vectorDot(AntOffset_J2000, los);
			return dR_LeoMcc;
		}


		// 子程序名称： tideCorrect 
		// 功能：海潮修正
		// 变量类型：t            :  参考时刻
		//           sunPos       :  太阳在地心系下位置（ITRF, 米）
		//           moonPos      :  月亮在地心系下位置（ITRF, 米）
		//           staPos       :  测站在地心系下位置（ITRF, 米）
		//           leoPos       :  卫星在地心系下位置（ITRF, 米）
		//           sotDatum     :  测站的海潮振幅和相位
		//           xp           :  极移
		//           yp           :  极移
		// 输入：t, sunPos, moonPos, staPos, leoPos, sotDatum, xp, yp
		// 输出：dR_TideCorrect 
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2007/12/31
		// 版本时间：2007/12/31
		// 修改记录：1. 2013/06/13, 由谷德峰修改, 升级到 IERS2003
		// 备注：调用solidTideCorrect 和 oceanTideLoadingCorrect
		double SLRPreproc::tideCorrect(GPST t, POS3D sunPos, POS3D moonPos, POS3D staPos, POS3D  leoPos, StaOceanTide sotDatum, double xp, double yp)
		{
			POS3D posSolidTide_ECEF = SolidTides::solidTideCorrect(t, sunPos, moonPos, staPos, xp, yp);
			POS3D posOceanTide_ECEF = OceanTidesLoading::oceanTideLoadingCorrect(t, staPos, sotDatum);
			// 计算卫星在测站坐标系下的位置矢量
			POS3D  vec_Leo_Sta = leoPos - staPos;
			double r  = sqrt(vec_Leo_Sta.x * vec_Leo_Sta.x + vec_Leo_Sta.y * vec_Leo_Sta.y + vec_Leo_Sta.z * vec_Leo_Sta.z); 
			double dR_TideCorrect = -vectorDot(posSolidTide_ECEF + posOceanTide_ECEF, vec_Leo_Sta) / r;
			return dR_TideCorrect;
		}
	}
}
