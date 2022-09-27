#pragma once
#include "constDef.hpp"
#include <string>
#include <map>

//  Copyright 2014, The National University of Defense Technology at ChangSha
using namespace std;
namespace NUDTTK
{
	struct RecCode
	{
		char szMITRecCod[ 6 + 1]; // MIT 的 6 位码接收机标记
		char szIGSRecCod[20 + 1]; // IGS 20-char code
		char cRecType_CPN;        // CPN
	};

	struct AntCode
	{
		char szMITAntCod[ 6 + 1]; // MIT 的 6 位码天线标记
		char szIGSAntCod[20 + 1]; // IGS 20-char code
	};

    typedef map<string, RecCode> RecCodeMap;
	typedef map<string, AntCode> AntCodeMap;

	// GAMIT rcvant.dat 文件解析
	class gamitRcvAntFile
	{
	public:
		gamitRcvAntFile(void);
	public:
		~gamitRcvAntFile(void);
	public:
		bool open(string rcvantFileName);
		bool getRecType(string strIGSRecCod, char &cRecType_CPN);
	public:
		RecCodeMap m_mapRecCode;
		AntCodeMap m_mapAntCode;
	};
}
