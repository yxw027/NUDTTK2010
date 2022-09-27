#pragma once
#include <string>
#include <vector>

//  Copyright 2012, The National University of Defense Technology at ChangSha
using namespace std;
namespace NUDTTK
{
	#define JPLEph_NItems                           11
	#define DE200RecordSize                         1652
	#define DE405RecordSize                         2036
	#define DE406RecordSize                         1456
	#define DE421RecordSize                         2036 //添加DE421
	#define DE430RecordSize                         2036
	#define NConstants                              400        
	#define ConstantNameLength                      6
	#define MaxChebyshevCoeffs                      32
	#define LabelSize                               84

	enum JPLEphItem
	{
		JPLEph_Mercury       =  0,
		JPLEph_Venus         =  1,
		JPLEph_EarthMoonBary =  2,
		JPLEph_Mars          =  3,
		JPLEph_Jupiter       =  4,
		JPLEph_Saturn        =  5,
		JPLEph_Uranus        =  6,
		JPLEph_Neptune       =  7,
		JPLEph_Pluto         =  8,
		JPLEph_Moon          =  9,
		JPLEph_Sun           = 10,
	};

	struct JPLEphCoeffInfo
	{
		unsigned int offset;    // coefficients offset from beginning of table for this object in # of double words. 
		unsigned int nCoeffs;   // number of coefficients in the subinterval
		unsigned int nSubinterv;// number of subintervals in the coefficient block for this object                       
	};

	struct JPLEphRecord
	{
		double t0;
		double t1;
		double* coeffs;
		int     count_coeffs;

		JPLEphRecord()
		{
			coeffs = NULL;
			count_coeffs = 0;
		};

		// 重新定义拷贝析构函数, 解决JPLEphFile文件无法拷贝问题
		JPLEphRecord(const JPLEphRecord & A)
		{
			t0 = A.t0;
			t1 = A.t1;
			count_coeffs = A.count_coeffs;
			coeffs = new double [count_coeffs];
			for(int i = 0; i < count_coeffs; i++)
				coeffs[i] = A.coeffs[i];
		};

		~JPLEphRecord()
		{
			if (coeffs != NULL)
				delete[] coeffs;
		};
	};

	class JPLEphFile
	{
	public:
		JPLEphFile(void);
	public:
		~JPLEphFile(void);
		
	public:
		bool         open(string  strJPLEphfileName);
		unsigned int getDENumber() const;
		double       getStartDate() const;
		double       getEndDate() const;
		bool         getPlanetPos(JPLEphItem, double t, double P[]);
		bool         getPlanetPosVel(JPLEphItem, double t, double PV[]);
		bool         getNutation(double t, double& delta_psi, double& delta_eps);
		bool         getMoonLibration(double t, double& omiga, double& i_s, double& lamda);
		bool         getMoonLibration(double t, double& omiga, double& omiga_dot, double& i_s, double& i_s_dot, double& lamda, double& lamda_dot);
		bool         getPlanetPos_EarthCenter(JPLEphItem, double t, double P[]);
		bool         getPlanetPosVel_EarthCenter(JPLEphItem, double t, double PV[]);
        bool         getSunPos_Delay_EarthCenter(double t, double P[], double threshold = 0.1);
		bool         getSunPosVel_Delay_EarthCenter(double t, double PV[], double threshold = 0.1); // +
        bool         getEarthPosVel(double t, double PV[]);
	private:
		char                 constant_name[NConstants][ConstantNameLength+1];
		char                 TTL[3][LabelSize+1];		     // Title lines, CHARACTER * 6  TTL(3, ConstantNameLength * 14)
		JPLEphCoeffInfo      coeffInfo[JPLEph_NItems];       // 记录系数的偏移
		JPLEphCoeffInfo      nutationCoeffInfo;              // 章动的系数
		JPLEphCoeffInfo      librationCoeffInfo;             // libration系数
		double               startDate;
		double               endDate;
		double               daysPerInterval;
		int                  ncon;
		double               au;
		double               emrat;
		double               constant_value[NConstants];
		unsigned int         DENum;                          // 星历的版本信息
		unsigned int         recordSize;                     // 每个大区间的系数块doubles数据记录个数
		vector<JPLEphRecord> records;
	};
}
