#include "RuningInfoFile.hpp"

namespace NUDTTK
{
	string RuningInfoFile::m_FilePathName = "c:\\NUDTTK.log";
	bool   RuningInfoFile::m_bAddIn = true;

	RuningInfoFile::RuningInfoFile(void)
	{
	}

	RuningInfoFile::~RuningInfoFile(void)
	{
	}

	void RuningInfoFile::Clear()
	{
		FILE* pFile = fopen(m_FilePathName.c_str(), "w+");
		if(pFile)
			fclose(pFile);
	}

	void RuningInfoFile::Add(string info)
	{
		FILE* pFile = 0;
		if(m_bAddIn)
		{
			pFile = fopen(m_FilePathName.c_str(), "a+");
			if(!pFile)
				return;
			/*DayTime T;
			T.Now();
			fprintf(pFile, "%04d//%02d//%02d %02d:%02d:%02d>> %s\n", T.year,
																	 T.month,
																	 T.day,
																	 T.hour,
																	 T.minute,
																	 int(T.second),
																	 info.c_str());*/
			fprintf(pFile, "%s\n", info.c_str());
			fclose(pFile);
		}
	}
}
