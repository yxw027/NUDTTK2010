#pragma once
#include <string>
#include "constDef.hpp"
//  Copyright 2012, The National University of Defense Technology at ChangSha

using namespace std;
namespace NUDTTK
{
	enum TYPE_IERSCONVENTIONS
	{
		IERSCONVENTIONS_1996     =  1,
		IERSCONVENTIONS_2003     =  2,
        IERSCONVENTIONS_2010     =  3,
		IERSCONVENTIONS_UNKNOWN  =  0,
	};
    struct DayTime
	{
		int    year;
		int    month;
		int    day;
		int    hour;
		int    minute;
		double second;

		DayTime()
		{}

		DayTime(int Year,int Month,int Day,int Hour,int Minute,double Second)
		{
			year   = Year;
			month  = Month;
			day    = Day;
			hour   = Hour;
			minute = Minute;
			second = Second;
		}

		DayTime(const DayTime& other) // 拷贝构造函数
		{
			year   = other.year;
			month  = other.month;
			day    = other.day;
			hour   = other.hour;
			minute = other.minute;
			second = other.second;
		}

		bool operator == (const DayTime& other) const
		{
			if(year == other.year && month == other.month && day == other.day && hour == other.hour && minute == other.minute && second == other.second)
				return true;
			else
				return false;
		}

		string toString(int nFraction = 9);

		DayTime	operator + (double spanSeconds) const;

		DayTime	operator - (double spanSeconds) const;

		double  operator - (const DayTime& other) const;

		bool operator < (const DayTime& other) const;

		void Now();

		int  doy();

	    // Decimal year e.g. 2008.135
        double decimalYear();

		bool ParseTimestamp(string strTimestamp);
	};
	
	typedef DayTime UTC;   // UTC时间
	typedef DayTime GPST;  // GPS时间
	typedef DayTime TAI;   // 国际原子时
	typedef DayTime TDT;   // TDT或1991年以后称为地球时TT
	typedef DayTime TDB;   // 太阳系质心动力学
	typedef DayTime UT1;   // 世界时
	typedef DayTime BDT;   // 北斗时间
	typedef DayTime BJT;   // 北京时间  BJT = UTC + 8h

	struct WeekTime
	{
		unsigned short	week;
		double	second;

		double  operator-(const WeekTime& other) const;
	};

	typedef WeekTime GPSWeekTime;  // GPS周时间
	typedef WeekTime BDWeekTime;   // BD周时间

	struct POS2D
	{
		double x;
		double y;

		POS2D()
		{
			x = 0;
			y = 0;
		}

		POS2D operator - (const POS2D& other) const
		{
			POS2D out;
			out.x = x - other.x;
			out.y = y - other.y;
			return out;
		}

		POS2D operator + (const POS2D& other) const
		{
			POS2D out;
			out.x = x + other.x;
			out.y = y + other.y;
			return out;
		}

		POS2D operator * (const double value) const
		{
			POS2D out;
			out.x = x * value;
			out.y = y * value;
			return out;
		}

		POS2D operator / (const double value) const
		{
			POS2D out;
			out.x = x / value;
			out.y = y / value;
			return out;
		}
	};

	struct POS3D 
	{
		double	x;
		double	y;
		double	z;

		POS3D()
		{
			x = 0;
			y = 0;
			z = 0;
		}

		POS3D operator - (const POS3D& other) const
		{
			POS3D out;
			out.x = x - other.x;
			out.y = y - other.y;
			out.z = z - other.z;
			return out;
		}

		POS3D operator + (const POS3D& other) const
		{
			POS3D out;
			out.x = x + other.x;
			out.y = y + other.y;
			out.z = z + other.z;
			return out;
		}

		POS3D operator * (const double value) const
		{
			POS3D out;
			out.x = x * value;
			out.y = y * value;
			out.z = z * value;
			return out;
		}

		POS3D operator / (const double value) const
		{
			POS3D out;
			out.x = x / value;
			out.y = y / value;
			out.z = z / value;
			return out;
		}
	};

	struct POSCLK
	{
		double	x;       
		double	y;       
		double	z;      
		double  clk;  // 等效单位: 米

		POSCLK()
		{
			x = 0;
			y = 0;
			z = 0;
			clk = 0;
		}

		POS3D getPos()
	{
		POS3D P;
		P.x = x;
		P.y = y;
		P.z = z;
		return P;
	}
	};

	struct POS6D 
	{
		double	x;       
		double	y;       
		double	z;       
		double	vx;      
		double	vy;     
		double	vz; 

		POS6D()
		{
			x  = 0;
			y  = 0;
			z  = 0;
			vx = 0;
			vy = 0;
			vz = 0;
		}

		void setPosVel(POS3D pos, POS3D vel)
		{
			x  = pos.x;
			y  = pos.y;
			z  = pos.z;
			vx = vel.x;
			vy = vel.y;
			vz = vel.z;
		}

		void setPos(POS3D pos)
		{
			x  = pos.x;
			y  = pos.y;
			z  = pos.z;
		}

		void setVel(POS3D vel)
		{
			vx = vel.x;
			vy = vel.y;
			vz = vel.z;
		}

		POS3D getPos()
		{
			POS3D pos;
			pos.x = x;
			pos.y = y;
			pos.z = z;
			return pos;
		}

		POS3D getVel()
		{
			POS3D vel;
			vel.x = vx;
			vel.y = vy;
			vel.z = vz;
			return vel;
		}

		POS6D operator - (const POS6D& other) const
		{
			POS6D out;
			out.x  = x  - other.x;
			out.y  = y  - other.y;
			out.z  = z  - other.z;
			out.vx = vx - other.vx;
			out.vy = vy - other.vy;
			out.vz = vz - other.vz;
			return out;
		}

		POS6D operator + (const POS6D& other) const
		{
			POS6D out;
			out.x  = x  + other.x;
			out.y  = y  + other.y;
			out.z  = z  + other.z;
			out.vx = vx + other.vx;
			out.vy = vy + other.vy;
			out.vz = vz + other.vz;
			return out;
		}

		POS6D operator*(const double value) const
		{
			POS6D out;
			out.x  = x  * value;
			out.y  = y  * value;
			out.z  = z  * value;
			out.vx = vx * value;
			out.vy = vy * value;
			out.vz = vz * value;
			return out;
		}

		POS6D operator / (const double value) const
		{
			POS6D out;
			out.x  = x  / value;
			out.y  = y  / value;
			out.z  = z  / value;
			out.vx = vx / value;
			out.vy = vy / value;
			out.vz = vz / value;
			return out;
		}
	};

	struct TimePosVel
	{
		DayTime t;
		POS3D   pos;
		POS3D   vel;
		
		POS6D getPosVel()
		{
			POS6D pv;
			pv.x  = pos.x;
			pv.y  = pos.y;
			pv.z  = pos.z;
			pv.vx = vel.x;
			pv.vy = vel.y;
			pv.vz = vel.z;
			return pv;
		}
	};

	struct TimePosVelClock
	{
		DayTime t;
		POS3D   pos;
		POS3D   vel;
		double  clk;
		
		POS6D getPosVel()
		{
			POS6D pv;
			pv.x  = pos.x;
			pv.y  = pos.y;
			pv.z  = pos.z;
			pv.vx = vel.x;
			pv.vy = vel.y;
			pv.vz = vel.z;
			return pv;
		}
	};

	struct TimePosVelAcc
	{
		DayTime t;
		POS3D   pos;
		POS3D   vel;
		POS3D   acc; // 加速度
		
		POS6D getPosVel()
		{
			POS6D pv;
			pv.x  = pos.x;
			pv.y  = pos.y;
			pv.z  = pos.z;
			pv.vx = vel.x;
			pv.vy = vel.y;
			pv.vz = vel.z;
			return pv;
		}

		TimePosVel getTimePosVel()
		{
			TimePosVel pvt;
			pvt.t = t;
			pvt.pos = pos;
			pvt.vel = vel;
			return pvt;
		}
	};

	struct TimePosVelAccJerk
	{
		DayTime t;
		POS3D   pos;
		POS3D   vel;
		POS3D   acc;   // 加速度
		POS3D   jerk;  // 加加速度
		
		POS6D getPosVel()
		{
			POS6D pv;
			pv.x  = pos.x;
			pv.y  = pos.y;
			pv.z  = pos.z;
			pv.vx = vel.x;
			pv.vy = vel.y;
			pv.vz = vel.z;
			return pv;
		}

		TimePosVel getTimePosVel()
		{
			TimePosVel pvt;
			pvt.t = t;
			pvt.pos = pos;
			pvt.vel = vel;
			return pvt;
		}
	};

	// 测站大地坐标
	struct BLH
	{
		double B; // 纬度, 单位: 度
		double L; // 经度, 单位: 度
		double H; // 高度, 单位: 米
	};

	// 测站东北天坐标
	struct ENU
	{
		double E; // 东
		double N; // 北
		double U; // 天

		ENU()
		{
			E = 0;
			N = 0;
			U = 0;
		}

		ENU operator * (const double value) const
		{
			ENU out;
			out.E = E * value;
			out.N = N * value;
			out.U = U * value;
			return out;
		}

		ENU operator + (const ENU& other) const
		{
			ENU out;
			out.E = E + other.E;
			out.N = N + other.N;
			out.U = U + other.U;
			return out;
		}
	};

	// 测站极坐标
	struct POLARCOORD
	{
		double	r;     // 距离
		double	fai;   // 纬度, [ -90,  90]
		double	lamda; // 经度, [-180, 180]
	};

	// 定义经典轨道根数
	struct ORBITROOT
	{
		double a;          // semi-major axis
		double e;          // 偏心率  eccentricity
		double i;          // 弧度    inclination
		double omiga;      // 弧度    longitude of ascending node
		double w;          // 弧度    近地点幅角  argument of periapsis
		double M;          // 平近点角, 弧度

		double getEFromM();
		double getfFromM();
		double getMFromf(double f);
		ORBITROOT getEpochOrbitRoot(double t, double gm = GM_EARTH); // 2018/09/14, 谷德峰添加, 中心天体可选择, 便于分析日心轨道
	};

	// 姿态四元素
	struct ATT_Q4
	{
		double q1;//x
		double q2;//y
		double q3;//z
		double q4;//w

		ATT_Q4()
		{
			q1 = 0.0;q2 = 0.0;q3 = 0.0;	q4 = 0.0;
		}

		ATT_Q4 operator*(const ATT_Q4& other) const
		{
		    ATT_Q4 Out;
			Out.q1 = q4 * other.q1 - q3 * other.q2 + q2 * other.q3 + q1 * other.q4;
			Out.q2 = q3 * other.q1 + q4 * other.q2 - q1 * other.q3 + q2 * other.q4;
			Out.q3 =-q2 * other.q1 + q1 * other.q2 + q4 * other.q3 + q3 * other.q4;
			Out.q4 =-q1 * other.q1 - q2 * other.q2 - q3 * other.q3 + q4 * other.q4;
			return Out;
		}
	};

	struct EULERANGLE
	{
		double xRoll;        // 滚动角, 绕x轴(飞行方向轴)旋转, 弧度
		double yPitch;       // 俯仰角, 绕y轴(轨道面法向)旋转, 弧度
		double zYaw;         // 偏航角, 绕z轴(地心轴)旋转, 弧度
		char   szSequence[4];

        EULERANGLE()
		{
			xRoll  = 0;
			yPitch = 0;
			zYaw   = 0;
			strcpy(szSequence, "312");
			szSequence[3] = '\0';
		}

	};

	enum TYPE_OBS_ID
	{
		TYPE_OBS_C1      = 1,
		TYPE_OBS_P1      = 2,
		TYPE_OBS_P2      = 3,
		TYPE_OBS_P3      = 4,
		TYPE_OBS_P5      = 5,
		TYPE_OBS_L1      = 6,
		TYPE_OBS_L2      = 7,
		TYPE_OBS_L3      = 8,
		TYPE_OBS_L5      = 9,
		TYPE_OBS_LP1     = 10,
		TYPE_OBS_D1      = 11,
		TYPE_OBS_D2      = 12,
		TYPE_OBS_T1      = 13,
		TYPE_OBS_T2      = 14,
		TYPE_OBS_SA      = 15,
		TYPE_OBS_S1      = 16,
		TYPE_OBS_S2      = 17,
		TYPE_OBS_S5      = 18,
		TYPE_OBS_PIF     = 19, // 2008/08/20, 虚拟观测量
		TYPE_OBS_LIF     = 20, // 2008/08/20, 虚拟观测量
		TYPE_OBS_MW      = 21, // 2009/12/03, 虚拟观测量
		TYPE_OBS_C2      = 22, // 2014/09/02，Rinex2.11增加了多种观测数据类型
		TYPE_OBS_C5      = 23,
		TYPE_OBS_D5      = 24,		
		TYPE_OBS_C7      = 25,
		TYPE_OBS_L7      = 26,
		TYPE_OBS_D7      = 27,
		TYPE_OBS_S7      = 28,
		TYPE_OBS_C8      = 29,
		TYPE_OBS_L8      = 30,
		TYPE_OBS_D8      = 31,
		TYPE_OBS_S8      = 32,
		TYPE_OBS_UNKNOWN =  0
	};

	enum TYPE_MONTH_ID
	{
		MONTH_JAN     =  1,
		MONTH_FEB     =  2,
		MONTH_MAR     =  3,
		MONTH_APR     =  4,
		MONTH_MAY     =  5,
		MONTH_JUN     =  6,
		MONTH_JUL     =  7,
		MONTH_AUG     =  8,
		MONTH_SEP     =  9,
		MONTH_OCT     = 10,
		MONTH_NOV     = 11,
		MONTH_DEC     = 12,
		MONTH_UNKNOWN =  0
	};

	enum TYPE_BDSTATION_ID
	{
		BDSTATION_CCHU     =  1,
		BDSTATION_CKUN     =  2,
		BDSTATION_CLIN     =  3,
		BDSTATION_CSHA     =  4,
		BDSTATION_CWUQ     =  5,
		BDSTATION_CNJI     =  6,
		BDSTATION_CHA1     =  7,
		BDSTATION_GUA1     =  8,
		BDSTATION_KUN1     =  9,
		BDSTATION_LHA1     =  10,
		BDSTATION_SHA1     =  11,
		BDSTATION_WUH1     =  12,
		BDSTATION_WHU2     =  13,
		BDSTATION_XIA1     =  14,
		BDSTATION_BJF1     =  15,
		BDSTATION_BJF2     =  16,
		BDSTATION_CNY1     =  17,
		BDSTATION_UNKNOWN  =  0,
	};

	enum TYPE_EDITEDMARK_ID
	{
		TYPE_EDITEDMARK_NORMAL    = 1,  // 正常点标记
		TYPE_EDITEDMARK_OUTLIER   = 2,  // 野值标记
		TYPE_EDITEDMARK_SLIP      = 3,  // 周跳标记
		TYPE_EDITEDMARK_UNKNOWN   = 0   // 未标记
	};

	enum TYPE_OBSPREPROC_INFO
	{   
		OBSPREPROC_UNKNOWN              = 00,
		OBSPREPROC_NORMAL               = 10,
		// 野值类型
		OBSPREPROC_OUTLIER_BLANK        = 20,
		OBSPREPROC_OUTLIER_ZERO         = 21,
		OBSPREPROC_OUTLIER_COUNT        = 22, 
		OBSPREPROC_OUTLIER_SNR          = 23, // 信噪比
		OBSPREPROC_OUTLIER_ELEVATION    = 24, // 高度截止角
		OBSPREPROC_OUTLIER_IONOMAXMIN   = 25,
		OBSPREPROC_OUTLIER_MW			= 26,
		OBSPREPROC_OUTLIER_VONDRAK      = 27,
		OBSPREPROC_OUTLIER_L1_L2        = 28,
		OBSPREPROC_OUTLIER_RAIM         = 29,

        // 周跳类型
		OBSPREPROC_NEWARCBEGIN          = 30,
		OBSPREPROC_SLIP_MW              = 31,
		OBSPREPROC_SLIP_IF              = 32,
		OBSPREPROC_SLIP_L1_L2           = 33,
		OBSPREPROC_SLIP_ThrFrePLGIFCom  = 34, // 三频伪距相位GIF组合探测的周跳
		OBSPREPROC_SLIP_ThrFreLGIFCom   = 35, // 三频相位GIF组合探测的周跳
		OBSPREPROC_SLIP_ThrFreLGFCom    = 36  // 三频相位GF组合探测的周跳

	};

	enum TYPE_DORISSTATION_ID
	{
		DORISSTATION_ADEA     =  1,
		DORISSTATION_ADEB     =  2,
		DORISSTATION_ADFB     =  3,
		DORISSTATION_AJAB     =  4,
		DORISSTATION_AMSA     =  5,
		DORISSTATION_AMTB     =  6,
		DORISSTATION_AMUB     =  7,
		DORISSTATION_AMVB     =  8,
		DORISSTATION_AREA     =  9,
		DORISSTATION_AREB     =  10,
		DORISSTATION_ARFB     =  11,
		DORISSTATION_ARMA     =  12,
		DORISSTATION_ASDB     =  13,
		DORISSTATION_BADA     =  14,
		DORISSTATION_BADB     =  15,
		DORISSTATION_BELB     =  16,
		DORISSTATION_BEMB     =  17,
		DORISSTATION_BETB     =  18,
		DORISSTATION_CACB     =  19,
		DORISSTATION_CADB     =  20,
		DORISSTATION_CHAB     =  21,
		DORISSTATION_CIBB     =  22,
		DORISSTATION_CICB     =  23,
		DORISSTATION_CIDB     =  24,
		DORISSTATION_COLA     =  25,
		DORISSTATION_CROB     =  26,
		DORISSTATION_CRPB     =  27,
		DORISSTATION_CRQB     =  28,
		DORISSTATION_DAKA     =  29,
		DORISSTATION_DIOA     =  30,
		DORISSTATION_DIOB     =  31,
		DORISSTATION_DJIA     =  32,
		DORISSTATION_DJIB     =  33,
		DORISSTATION_EASA     =  34,
		DORISSTATION_EASB     =  35,
		DORISSTATION_EVEB     =  36,
		DORISSTATION_FAIA     =  37,
		DORISSTATION_FAIB     =  38,
		DORISSTATION_FLOA     =  39,
		DORISSTATION_FUTB     =  40,
		DORISSTATION_GALA     =  41,
		DORISSTATION_GAVB     =  42,
		DORISSTATION_GOLA     =  43,
		DORISSTATION_GOMA     =  44,
		DORISSTATION_GOMB     =  45,
		DORISSTATION_GR3B     =  46,
		DORISSTATION_GREB     =  47,
		DORISSTATION_GUAB     =  48,
		DORISSTATION_HBKA     =  49,
		DORISSTATION_HBKB     =  50,
		DORISSTATION_HBLA     =  51,
		DORISSTATION_HBMB     =  52,
		DORISSTATION_HELA     =  53,
		DORISSTATION_HELB     =  54,
		DORISSTATION_HEMB     =  55,
		DORISSTATION_HUAA     =  56,
		DORISSTATION_HVOA     =  57,
		DORISSTATION_IQUB     =  58,
		DORISSTATION_JIUB     =  59,
		DORISSTATION_KERA     =  60,
		DORISSTATION_KERB     =  61,
		DORISSTATION_KESB     =  62,
		DORISSTATION_KETB     =  63,
		DORISSTATION_KITA     =  64,
		DORISSTATION_KITB     =  65,
		DORISSTATION_KIUB     =  66,
		DORISSTATION_KOKA     =  67,
		DORISSTATION_KOLB     =  68,
		DORISSTATION_KRAB     =  69,
		DORISSTATION_KRBB     =  70,
		DORISSTATION_KRUB     =  71,
		DORISSTATION_LIBA     =  72,
		DORISSTATION_LIBB     =  73,
		DORISSTATION_LICB     =  74,
		DORISSTATION_LIFB     =  75,
		DORISSTATION_MAHB     =  76,
		DORISSTATION_MALB     =  77,
		DORISSTATION_MANA     =  78,
		DORISSTATION_MANB     =  79,
		DORISSTATION_MARA     =  80,
		DORISSTATION_MARB     =  81,
		DORISSTATION_MATB     =  82,
		DORISSTATION_META     =  83,
		DORISSTATION_METB     =  84,
		DORISSTATION_MIAB     =  85,
		DORISSTATION_MONB     =  86,
		DORISSTATION_MOOB     =  87,
		DORISSTATION_MORA     =  88,
		DORISSTATION_MORB     =  89,
		DORISSTATION_MSOB     =  90,
		DORISSTATION_MSPB     =  91,
		DORISSTATION_NOUA     =  92,
		DORISSTATION_NOUB     =  93,
		DORISSTATION_NOWB     =  94,
		DORISSTATION_ORRA     =  95,
		DORISSTATION_ORRB     =  96,
		DORISSTATION_OTTA     =  97,
		DORISSTATION_OTTB     =  98,
		DORISSTATION_PAPB     =  99,
		DORISSTATION_PAQB     =  100,
		DORISSTATION_PASB     =  101,
		DORISSTATION_PATB     =  102,
		DORISSTATION_PAUB     =  103,
		DORISSTATION_PDLB     =  104,
		DORISSTATION_PDMB     =  105,
		DORISSTATION_PURA     =  106,
		DORISSTATION_RAQB     =  107,
		DORISSTATION_REUA     =  108,
		DORISSTATION_REUB     =  109,
		DORISSTATION_REYA     =  110,
		DORISSTATION_REYB     =  111,
		DORISSTATION_REZB     =  112,
		DORISSTATION_RIDA     =  113,
		DORISSTATION_RIKB     =  114,
		DORISSTATION_RILB     =  115,
		DORISSTATION_RIOA     =  116,
		DORISSTATION_RIOB     =  117,
		DORISSTATION_RIPB     =  118,
		DORISSTATION_RIQB     =  119,
		DORISSTATION_ROTA     =  120,
		DORISSTATION_ROTB     =  121,
		DORISSTATION_ROUB     =  122,
		DORISSTATION_SAKA     =  123,
		DORISSTATION_SAKB     =  124,
		DORISSTATION_SALB     =  125,
		DORISSTATION_SAMB     =  126,
		DORISSTATION_SANA     =  127,
		DORISSTATION_SANB     =  128,
		DORISSTATION_SAOB     =  129,
		DORISSTATION_SCRB     =  130,
		DORISSTATION_SODA     =  131,
		DORISSTATION_SODB     =  132,
		DORISSTATION_SPIA     =  133,
		DORISSTATION_SPIB     =  134,
		DORISSTATION_SPJB     =  135,
		DORISSTATION_STJB     =  136,
		DORISSTATION_SYOB     =  137,
		DORISSTATION_SYPB     =  138,
		DORISSTATION_TANB     =  139,
		DORISSTATION_THUB     =  140,
		DORISSTATION_TLHA     =  141,
		DORISSTATION_TLSA     =  142,
		DORISSTATION_TLSB     =  143,
		DORISSTATION_TRIA     =  144,
		DORISSTATION_TRIB     =  145,
		DORISSTATION_WALA     =  146,
		DORISSTATION_YARA     =  147,
		DORISSTATION_YARB     =  148,
		DORISSTATION_YASB     =  149,
		DORISSTATION_YELA     =  150,
		DORISSTATION_YELB     =  151,
		DORISSTATION_YEMB     =  152,
		DORISSTATION_UNKNOWN =  0
	};

	void    vectorCross(POS3D& out, POS3D v1,POS3D v2);

	double  vectorDot(POS3D v1, POS3D v2);

    POS3D   vectorNormal(POS3D v);

	double  vectorMagnitude(POS3D v);

	int     yearB2toB4(int nB2Year);

	char toLowerCase(char c);
	char toCapital(char c);
	bool isWildcardMatch(const char *stringToCheck, const char *wildcardString , bool caseSensitive = true); 

	int  string2ObsId(string Object);
	string obsId2String(int Id);

	int getIntBit(int nValue,int nBitFlag);

	int string2MonthId(string strMonth);
	string monthId2string(int nMonth);

	int  string2BDStationId(string BDStationName);
	string  bdStationId2String(int nBDStation); 

	int     string2DorisStationId(string DorisStationName);
	string  dorisStationId2String(int nDorisStation);

    int  stringRaplaceA2B(string& strSrc, char A, char B);
	int  stringRaplaceA2B(char*   strSrc, char A, char B);

	void stringEraseFloatZero(string& strFloat);
	void stringEraseFloatZero(const char* szFloat, string& strFloat);

	void swapbit_High_Low(void* data, int n);
}