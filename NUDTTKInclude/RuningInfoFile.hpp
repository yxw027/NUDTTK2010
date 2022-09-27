#pragma once
#include <string>
#include "structDef.hpp"
#include <vector>
//  Copyright 2014, The National University of Defense Technology at ChangSha

using namespace std;
namespace NUDTTK
{
	struct RuningInfo
	{
		DayTime t;
		string  info;
	};

	// 输出运行时的日志信息文件
	class RuningInfoFile
	{
	public:
		RuningInfoFile(void);
	public:
		~RuningInfoFile(void);
	public:
		static void Clear();
		static void Add(string info);
		static string             m_FilePathName;
		static vector<RuningInfo> m_RunInfoList;
		static bool               m_bAddIn;
	};
}
