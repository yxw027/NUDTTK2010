#pragma once
#include "structDef.hpp"
#include <windows.h>
#include <vector>
#include <map>

using namespace NUDTTK;
namespace NUDTTK
{
	namespace DORIS
	{
		struct Doris2_2_EditedObsLine
		{
			char   Sat_ID[8];                 // 卫星名称
	        int    Meas_type;                 // 测量方法的种类39表示DORIS
            int    Time_location;             // 采用何地的时间
	        int    Time_type;                 // 时间类型
	        char   Station_ID[5];             // 测站名称
	        int    Year;                      // 年(大于90，则表示1900+year；其余表示2000+year)
	        int    Doy;                       // 这年中的第几天, day of year
	        int    Second_int;                // 秒的整数部分
	        int    Second_fra;                // 秒的小数部分
	        int    Iono_apply;                // 是否改正的电离层折射。0表示经过了改正；1表示没有经过改正
	        int    Trop_apply;                // 是否改正的对流层折射。0表示经过了改正；1表示没有经过改正 
	        int    Point_infor;               // 该点的信息。0表示该点是好的；1表示在预处理过程中被编辑；2表示在事后处理过程中被编辑
	        double Cout_interval;             // 采样点的时间间隔。单位是0.1 microseconds=1e-7s
	        double Range_rate;                // 卫星与测站间的径向相对速度，单位micrometers/second=1e-6m/s
	        int    Surface_pressure;          // 测站表面得大气压，单位millibars=100pa
	        int    Surface_temperature;       // 测站表面的温度，单位degrees kelvin
	        int    Rela_humidity;             // 湿度，百分数
	        int    Obsev_RMS;                 // 观测数据的标准差，单位micrometers/second
	        int    Iono_correction;           // 电离层改正量，单位micrometers/second
	        int    Trop_correction;           // 对流层改正量，单位micrometers/second
	        int    Beacon_type;               // 测站的类型，0表示永久测站，1表示野外试验测站，3表示其它
	        int    Meteor_source;             // 气象数据的来源，
	                                          // 0 = measured parameter  
                                              // 1 = pressure from a model  
                                              // 3 = temperature from a model  
                                              // 4 = pressure and temperature from a model  
                                              // 5 = humidity from a model  
                                              // 6 = pressure and humidity from a model 
                                              // 8 = temperature and humidity from a model  
                                              // 9 = pressure, temperature, and humidity from a model
	        int    Channel_ID;                // 通道编号
	        int    mass_correction;           // 质心改正量(包括卫星和测站), 单位 micrometers/second

			Doris2_2_EditedObsLine()
			{
				memset(this, 0, sizeof(Doris2_2_EditedObsLine));
			}

			TAI GetTime()
			{
				int nYear = Year <= 90 ? 2000 + Year : 1900 + Year;
				TAI t0(nYear, 1, 1, 0, 0, 0);
				return t0 + (Doy - 1) * 86400.0 + Second_int + Second_fra * 1.0E-6 ;
			};

			void SetTime(TAI t)
			{
				Year = t.year % 100;
				TAI t0(t.year, 1, 1, 0, 0, 0);
				double seconds_all = t - t0;
				Doy = (int)(seconds_all / 86400.0) + 1;
				Second_int = t.hour * 3600 + t.minute * 60 + (int)t.second;
			    Second_fra = (int)((t.second - (int)t.second)*1.0E+6);
			}
        };

		typedef map<int,  Doris2_2_EditedObsLine> Doris2_2_EditedObsStationMap; // 单个时刻的不同测站观测数据列表
		typedef map<TAI,  Doris2_2_EditedObsLine> Doris2_2_EditedObsEpochMap;   // 单个测站的不同时刻观测数据列表

		// 观测数据时间结构
		struct Doris2_2_EditedObsEpoch
		{
			TAI                          t;
			Doris2_2_EditedObsStationMap obs;
		};

		// 观测数据测站结构
		struct Doris2_2_EditedObsStation
		{
			int                          id_sation; // 测站标号
			Doris2_2_EditedObsEpochMap   obs; 
		};

		class Doris2_2_EditedObsFile
		{
		public:
			Doris2_2_EditedObsFile(void);
		public:
			~Doris2_2_EditedObsFile(void);
		public:
			int   isValidDoris2_2_EditedObsLine(string strLine, FILE * pFile = NULL);
			bool  open(string  strEditedObsFileName);
			bool  write(string strEditedObsFileName);
			bool  cutdata(TAI t0,TAI t1);
			void  split(string strName, double period = 86400.0);
			bool  getObsStationList(vector<Doris2_2_EditedObsStation>& obsStationList);
			bool  getObsEpochList(vector<Doris2_2_EditedObsStation> obsStationList, vector<Doris2_2_EditedObsEpoch>& obsEpochList);
		public:
			vector<Doris2_2_EditedObsLine>  m_data;
		};
	}
}
