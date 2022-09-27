#pragma once
#include "constDef.hpp"
#include "structDef.hpp"
#include "Matrix.hpp"
#include "tai_utcFile.hpp"
#include "eopc04File.hpp"
#include "eopc04TotalFile.hpp"
#include "eopRapidFileIAU2000.hpp"
#include "jplEphFile.hpp"

//  Copyright 2012, The National University of Defense Technology at ChangSha

namespace NUDTTK
{
	//enum TYPE_IERSCONVENTIONS
	//{
	//	IERSCONVENTIONS_1996     =  1,
	//	IERSCONVENTIONS_2003     =  2,
	//	IERSCONVENTIONS_UNKNOWN  =  0,
	//};

	enum TYPE_IERSEOPFILE
	{
		IERSEOPFILE_C04_1980          =  1,      // C04 for 1980岁差-章动模型
		IERSEOPFILE_C04_2000A         =  2,      // C04 for 2000A岁差-章动模型
		IERSEOPFILE_C04Total_1980     =  3,      // 多年的C04 for 1980岁差-章动模型
		IERSEOPFILE_C04Total_2000A    =  4,      // 多年的C04 for 2000A岁差-章动模型
		IERSEOPFILE_RAPID             =  5,
		IERSEOPFILE_UNKNOWN           =  0,
	};

	class TimeCoordConvert
	{
	public:
		TimeCoordConvert(void);
	public:
		~TimeCoordConvert(void);
	public:
		// 静态时间系统转换部分
		static double      JD2MJD(double jd);
		static double      MJD2JD(double mjd);
		static double      DayTime2JD (DayTime gc);
		static double      DayTime2MJD(DayTime gc, int nFlag = 1);
		static double      DayTime2J2000Year(DayTime gc, int nFlag = 1);
		static DayTime     JD2DayTime(double dJD);
		static TAI         GPST2TAI(GPST T_GPS);
		static GPST        TAI2GPST(TAI T_TAI);
		static TDT         TAI2TDT(TAI T_TAI);
		static TAI         TDT2TAI(TDT T_TDT);
		static TDT         GPST2TDT(GPST T_GPS);
		static GPST        TDT2GPST(TDT T_TDT);
		static TDB         TDT2TDB(TDT T);
		static TDB         GPST2TDB(GPST T);
		static GPSWeekTime GPST2WeekTime(GPST T_GPS);
		static GPST        WeekTime2GPST(GPSWeekTime T_WT);
		static BDWeekTime  BDT2WeekTime(BDT T_BDT);
		static BDT         WeekTime2BDT(BDWeekTime T_WT);
		static TAI         BDT2TAI(BDT T_BDT);
		static BDT         TAI2BDT(TAI T_TAI);
		static BDT         GPST2BDT(GPST T_GPST);
		static GPST        BDT2GPST(BDT T_BDT);
		// 静态坐标系统转换部分
		static Matrix    rotate(double angle, int axis, int dim = 3);
		static Matrix    rotate_dot(double angle, int axis, int dim = 3);
		static bool      J2000_ECEF(double X_J2000[],double X_ECF[],double jY2000_TDT,UT1 ut1,double dUT1_TT_Rate,double dDelt_EPS,double dDelt_PSI,double dXP,double dYP, int iersConventions = IERSCONVENTIONS_2003);
        static bool      ECEF_J2000(double X_J2000[],double X_ECF[],double jY2000_TDT,UT1 ut1,double dUT1_TT_Rate,double dDelt_EPS,double dDelt_PSI,double dXP,double dYP, int iersConventions = IERSCONVENTIONS_2003);
		static bool      ECEF_J2000_PVA(double X_J2000[],double X_ECF[],double jY2000_TDT,UT1 ut1,double dUT1_TT_Rate,double dDelt_EPS,double dDelt_PSI,double dXP,double dYP, int iersConventions = IERSCONVENTIONS_2003);
		static bool      Matrix_J2000_ECEF(Matrix& matPR_NR, Matrix& matER, Matrix& matEP, Matrix &matER_Dot, double jY2000_TDT,UT1 ut1,double dUT1_TT_Rate,double dDelt_EPS,double dDelt_PSI,double dXP,double dYP, int iersConventions = IERSCONVENTIONS_2003);
		static bool      Matrix_J2000_ECEF(Matrix& matH,double jY2000_TDT,UT1 ut1,double dUT1_TT_Rate,double dDelt_EPS,double dDelt_PSI,double dXP,double dYP, int iersConventions = IERSCONVENTIONS_2003);
        static bool      XYZ2BLH(POS3D xyz, BLH& blh, double re = EARTH_R, double f = EARTH_F);
        static void      BLH2XYZ(BLH blh, POS3D& xyz, double re = EARTH_R, double f = EARTH_F);
		static void      ECF2ENU(POS3D pos, POS3D xyz, ENU &enu);

		static void      Local_Terrestrial_Vector(double Xest,double Yest,double Zest, POS3D &Nt, POS3D &Et, POS3D &Ut); // +

		static void      ENU2ECF(POS3D pos, ENU enu, POS3D &xyz);
		static void      Cartesian2Polar(POS3D xyz, POLARCOORD& rfl);
		static void      POS6D2ORBITROOT(POS6D posvel, ORBITROOT& root, double gm_EARTH = 3.986004415E+14);
		static void      ORBITROOT2POS6D(ORBITROOT root, POS6D& posvel, double gm_EARTH = 3.986004415E+14);
		static bool      getCoordinateRTNAxisVector(UT1 t, POS6D posvel, POS3D &axisvec_R, POS3D &axisvec_T, POS3D &axisvec_N);
		static void      J2000_Ecliptic_Equator(POS3D posEcliptic, POS3D &posEquator);
		static void      J2000_Ecliptic_Equator(POS6D pvEcliptic, POS6D &pvEquator);
		static void      J2000_Equator_Ecliptic(POS3D posEquator, POS3D &posEcliptic);
		static void      J2000_Equator_Ecliptic(POS6D pvEquator, POS6D &pvEcliptic);
		static void      Matrix_J2000_ECEF_Moon(double omiga, double i_s, double lamda, Matrix &matH);

		// IAU2000A 部分
		static void      IAU2000A_Nutation_Lunisolar(double jY2000_TDT,double& dpsi_ls,double& deps_ls);
		static void      IAU2000A_Nutation_Planetary(double jY2000_TDT,double& dpsi_plan,double& deps_plan);
        static void      IAU2000A_Nutation_Prec(double jY2000_TDT,double& dpsi_prec,double& deps_prec);
		static void      IAU2000A_Nutation_Total(double jY2000_TDT,double& dpsi_tot,double& deps_tot);
		static double    IAU2000A_GMST(UT1 ut1, double jY2000_TDT);
		static double    IAU2000A_GST(UT1 ut1, double jY2000_TDT, double dpsi, double deps_A);
		static double    IAU2000A_GST(UT1 ut1, double jY2000_TDT);
		static void      IAU2000A_XYUT1_Tidal(UT1 ut1, double jY2000_TDT, double &dxTidal, double &dyTidal, double &dut1Tidal);
		static void      IAU2000A_XY_Nutation(UT1 ut1, double jY2000_TDT, double &dxNutation, double &dyNutation);
		static void      IAU2000A_1976Precession(double jY2000_TDT, Matrix& matPR);

		static double    IAU2000_Table5_1[25][10];
		static double    IAU2000_Table5_2_C_0[33][10];
		static double    IAU2000_Table5_2_C_1[1][10];
		static double    IAU2000_Table5_3_A[678][11];
		static double    IAU2000_Table5_3_B[687][18];
		static double    IAU2000_Table8_2[71][12];

		// IAU1996 部分
		static double    IAU1996_GMST(UT1 ut1);
		static double    IAU1996_GST(UT1 ut1, double jY2000_TDT, double dpsi, double deps_A);
		static double    IAU1996_GMSTRate(UT1 ut1);
		static void      IAU1996_1976Precession(double jY2000_TDT, Matrix& matPR);
		static void      IAU1996_1980Nutation(double jY2000_TDT,double& delta_PSI,double& delta_EPS);

		static double    IAU1996_Table5_1[106][9]; 

		// 动态转换部分, 需要依赖外部文件参数
		// 时间系统
		UTC  TAI2UTC(TAI T_TAI);
	    GPST UTC2GPST(UTC T_UTC);
	    UT1  TAI2UT1(TAI TAI);
	    UT1  GPST2UT1(GPST T_GPS);
		BJT  GPST2BJT(GPST T_GPS);
		GPST BJT2GPST(BJT T_BJT);
		// 坐标系统
		void LoadTimeCoordConvertParameter(GPST t, double &jY2000_TDT, UT1 &ut1, double &ut1_tt_rate, double &delta_eps, double &delta_psi, double &xp, double &yp);
		bool J2000_ECEF(GPST t,double x_j2000[],double x_ECEF[],bool pvflag = true); 
		bool ECEF_J2000(GPST t,double x_j2000[],double x_ECEF[],bool pvflag = true);
		bool ECEF_J2000_PVA(GPST t,double x_j2000[],double x_ECEF[]);
		bool Matrix_J2000_ECEF(GPST t, Matrix &matH);
		bool Matrix_J2000_ECEF(GPST t, Matrix& matPR, Matrix& matNR, Matrix& matER, Matrix& matEP);

		bool Matrix_J2000_ECEF_Moon(GPST t, Matrix &matH);
		bool J2000_Earth_Moon(GPST t,double x_j2000_E[],double x_j2000_M[],bool pvflag = true); 
		bool J2000_Earth_Sun(GPST t,double x_j2000_E[],double x_j2000_S[],bool pvflag = true); 
		bool J2000_Moon_Earth(GPST t,double x_j2000_E[],double x_j2000_M[],bool pvflag = true); 
		bool J2000_ECEF_Moon(GPST t,double x_j2000_M[],double x_ECEF_M[],bool pvflag = true); 
		bool ECEF_J2000_Moon(GPST t,double x_j2000_M[],double x_ECEF_M[],bool pvflag = true);
    public:
        TAI_UTCFile          m_tai_utcFile;        // tai_utc 跳秒数据
		Eopc04File           m_eopc04File;         // 极移数据xp、yp 和 ut1-utc, from Bulletin C
		Eopc04TotalFile      m_eopc04TotalFile;    // 多年的极移数据xp、yp 和 ut1-utc, from Bulletin C
		eopRapidFileIAU2000  m_eopRapidFileIAU2000;// 极移数据xp、yp 和 ut1-utc, from Bulletin A
		JPLEphFile           m_jplEphFile;         // JPL DE405星历数据文件, 2018/09/13增加
		int                  m_iersConventions;
		int                  m_eopFileType;
	};
}