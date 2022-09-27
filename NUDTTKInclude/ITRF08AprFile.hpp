#pragma once
#include "structDef.hpp"
#include <string>
#include <vector>
#include <map>

//  Copyright 2012, The National University of Defense Technology at ChangSha
using namespace std;
namespace NUDTTK
{
	struct ITRF08AprLine
	{
		string       name_8c;   // Ãû³Æ
        POS6D        posvel;    // µ¥Î»: m
		double       t0;

		ITRF08AprLine() 
		{
			name_8c   =  "        ";
			posvel.x  = 0;
			posvel.y  = 0;
			posvel.z  = 0;
			posvel.vx = 0;
			posvel.vy = 0;
			posvel.vz = 0;
		}
	};

	struct ITRF08AprStation
	{
		string                  name_4c;     
        vector<ITRF08AprLine>   posvelList;
	};

	typedef map<string, ITRF08AprStation> ITRF08AprStationMap;

	class ITRF08AprFile
	{
	public:
		ITRF08AprFile(void);
	public:
		~ITRF08AprFile(void);
		bool isValidNewLine(string strLine, ITRF08AprLine &aprLine);
		bool open(string  strITRF08AprFileName);
		bool getPosVel(string name, UTC t, POS6D& posvel);
	public:
		ITRF08AprStationMap m_data;
	};
}
