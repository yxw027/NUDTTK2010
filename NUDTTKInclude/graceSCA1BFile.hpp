#pragma once
#include "constDef.hpp"
#include "structDef.hpp"
#include <string>
#include <vector>

namespace NUDTTK
{
	namespace SpaceborneGPSPod
	{
		// Star Camera Assembly
        /* SCA1A 数据至多包含两个星敏测量数据：来自主星敏的1s高数据率采样数据和第二个星敏的5s采样数据
           
		   这两个数据流被编辑, 参考四元数通过采用轨道视线被计算, 残差被每天加工一次.
		   野值首先通过一个全局的3sigma准则被删除, 然后进一步通过一个局部的3sigma准则剔除
		   结果四元数对符号模糊进行校验

		   对于主星敏感器, 当数据的间隔小于等于10s, 此时中间丢失的数据采用2次插值填充, 当每边至少有2个点可以利用时
           如果每边不足2个点, 则采用线性插值来填充
		   数据间隔超过10秒钟的空缺将不被填充. 
		   第二个星敏的数据空缺不被填充
		  
		   时间戳校正后的数据，主星敏感器被重采样到1s间隔, 第二个星敏感器被重采样到5s间隔,采用线性插值
		   主星敏感器数据然后被压缩到5s间隔,通过对1s的数据采用2次多项式拟合

		   最终主和第二个星敏感器的四元数被加权组合, 测量数据采用5s间隔进行报告
        */
		#pragma pack(1)
		struct SCA1BRecord
		{// 47个字节
			int           gps_time;
			char          grace_id;
            char          sca_id;
			double        quatangle;
			double        quaticoeff;
			double        quatjcoeff;
			double        quatkcoeff;
			double        qual_rss;
			unsigned char qualflg;
			/*
				Quality flags that indicate data gaps filled, according to severity:
					qualflg bit 0=1 filled data at T
					qualflg bit 1=1 filled data at T +/- 1 second
					qualflg bit 2=1 filled data at T +/- 2 seconds
				Additional quality flags include:
					qualflg bit 3=1 only one star camera enabled
					qualflg bit 4=1 extrapolated clock correction used
					qualflg bit 6=1 low rate data from 2nd star camera
					qualflg bit 7=1 low rate data from 1st star camera
			*/

			GPST gettime()
			{
				GPST t0(2000, 1, 1, 12, 0, 0);
				return t0 + gps_time;
			};
		};
		#pragma pack()

		class graceSCA1BFile
		{
		public:
			graceSCA1BFile(void);
		public:
			~graceSCA1BFile(void);
		public:
			bool open(string strSCA1BfileName);
			//bool grace_B_SRF_XYZ();
			bool exportTimeAttitudeFile(string  strTimeAttitudeFileName);
		public:
			vector<SCA1BRecord> m_data;
		};
	}
}
