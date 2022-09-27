#pragma once
#include <string>
#include <vector>
#include <map>
#include <time.h>
#include <direct.h>
#include "structDef.hpp"
#include "ITRF08AprFile.hpp"
#include "igs05atxFile.hpp"

//  Copyright 2012, The National University of Defense Technology at ChangSha
using namespace std;

namespace NUDTTK
{
	struct AllStaInfoDatum
	{
		char		StaName[4 + 1];		// 测站名称：4c
		char		SatSysType;			// 卫星系统类型
		char		RecType[20 + 1];	// 接收机类型
		char		CPN;				// 接收机类型识别码
		char		SwVer[20 + 1];		// 接收机软件版本
		char		AntType[20 + 1];	// 天线类型
		POS3D       ApproxPos;			// 测站坐标
		ENU			AntPos;				// 天线在参考坐标系下的坐标
		int		    QualityFlag;		// 测站数据质量评级
		int			UseFlag;			// 测站可用性标记
		AllStaInfoDatum()
		{
			memset(this,0,sizeof(AllStaInfoDatum));
		}
	};

	struct StaInfoDatum
	{
		char		StaName[4 + 1];		// 测站名称：4c
		double		RinexVer;			// RINEX 文件版本
		char		SatSysType;			// 卫星系统类型
		char		RecType[20 + 1];	// 接收机类型
		char		AntType[20 + 1];	// 天线类型
		POS3D		ARP;				// 天线参考点坐标(已修正天线高)
		char		StaPosFrom;			// 测站坐标来源标记
		int			UseFlag;			// 测站可用性标记, 0 - 可用， 1 - 不可用
		StaInfoDatum()
		{
			memset(this, 0, sizeof(StaInfoDatum));
		}
	};

	typedef map<string, AllStaInfoDatum>  AllStaInfoMap;
	typedef map<string, StaInfoDatum>	  NetStaInfoMap;

	class stationInfoFile
	{
	public:
		stationInfoFile(void);
	public:
		~stationInfoFile(void);
	public:
		bool open_net(string strStafileName);
		bool open_all(string strStafileName);
		
	public:
		AllStaInfoMap	m_alldata;
		NetStaInfoMap	m_netdata;
	};
}
