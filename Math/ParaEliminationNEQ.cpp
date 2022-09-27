#include "ParaEliminationNEQ.hpp"


namespace NUDTTK
{
	namespace Math
	{
		ParaEliminationNEQ::ParaEliminationNEQ(void)
		{
		}

		ParaEliminationNEQ::~ParaEliminationNEQ(void)
		{
		}

		// 子程序名称： init  
		// 功能：根据 m_intervalList 进行排序
		// 变量类型：
		// 输入：
		// 输出：
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2018/8/28
		// 版本时间：2018/8/28
		// 修改记录：
		// 备注：依赖 m_sequenceList
		void ParaEliminationNEQ::init()
		{
			// 第一步: 排序, 根据 m_intervalList, 生成 m_sequenceList 和 m_idList
			int n = int(m_intervalList.size());
			m_sequenceList.resize(n);
			m_idList.resize(n);
			m_data.resize(n);
			for(int i = 0; i < n; i++)
			{
				for(int j = i + 1; j < n; j++)
				{
					if(m_intervalList[j].t1 - m_intervalList[i].t1 < 0.0)  
					{
						PE_INTERVAL temp  = m_intervalList[i];
						m_intervalList[i] = m_intervalList[j];
						m_intervalList[j] = temp;
					}
				}
				m_idList[i] = m_intervalList[i].id;
				m_sequenceList[m_intervalList[i].id] = i;

				m_data[i].nx = 0.0;
				m_data[i].Nxx_row.clear();
			}
		}

		// 子程序名称： plusElement_Nxx  
		// 功能：对法方程Nxx矩阵赋值，只赋值非零元素
		// 变量类型：id_row               : 行数
		//           id_col               : 列数
		//           value                : 值
		// 输入：id_row, id_col, value
		// 输出：
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2018/8/28
		// 版本时间：2018/8/28
		// 修改记录：
		// 备注： 
		void ParaEliminationNEQ::plusElement_Nxx(int id_row, int id_col, double value)
		{
			int i,j;
			// 每个行元素用 Nxx_row 记录，double 只记录上三角值，即列号大于等于行号(i <= j)的元素（对称）
			if(m_sequenceList[id_row] <= m_sequenceList[id_col])
			{
				i = m_sequenceList[id_row];
				j = m_sequenceList[id_col];
			}
			else
			{
				j = m_sequenceList[id_row];
				i = m_sequenceList[id_col];
			}
			map<int, double>::iterator it_j = m_data[i].Nxx_row.find(j);
			if(it_j != m_data[i].Nxx_row.end())
				it_j->second += value; 
			else
				m_data[i].Nxx_row.insert(map<int, double>::value_type(j, value));
		}

		void ParaEliminationNEQ::plusElement_nx(int id_row, double value)
		{
			int i = m_sequenceList[id_row];
			m_data[i].nx += value;
		}

		// 子程序名称： main_ParaElimination  
		// 功能：法方程参数消去主函数
		// 变量类型：
		// 输入：
		// 输出：dx
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2018/8/28
		// 版本时间：2018/8/28
		// 修改记录：
		// 备注： 依赖于 m_intervalList
		bool ParaEliminationNEQ::main_ParaElimination(Matrix &dx)
		{
			int n = int(m_intervalList.size());
			// 参数消去
			for(int i = 0; i < n-1; i++)
			{
				// 先消去a，所有与a有关参数，对应在Nbb和nb中的位置元素都会被修正
				double Naa = m_data[0].Nxx_row[i]; // m_data[0] 每消去一行，m_data减小一维
				if(Naa == 0.0)
					return false;

				// 更新Nbb
				for(map<int, double>::iterator it_i = m_data[0].Nxx_row.begin(); it_i != m_data[0].Nxx_row.end(); ++it_i)
				{
					for(map<int, double>::iterator it_j = m_data[0].Nxx_row.begin(); it_j != m_data[0].Nxx_row.end(); ++it_j)
					{
						// 每个行元素用Nxx_row记录，double只记录列号大于等于行号(i <= j)的元素（对称）
						if(it_i->first <= it_j->first)
						{
							int i_row = it_i->first - i; // 新行号
							map<int, double>::iterator it = m_data[i_row].Nxx_row.find(it_j->first);
							double value = -it_i->second * (1.0 / Naa) * it_j->second;
							if(it != m_data[i_row].Nxx_row.end())
								it->second += value; 
							else
								m_data[i_row].Nxx_row.insert(map<int, double>::value_type(it_j->first, value));
						}
					}
				}
				double na = m_data[0].nx;
				// 更新nb
				for(map<int, double>::iterator it_i = m_data[0].Nxx_row.begin(); it_i != m_data[0].Nxx_row.end(); ++it_i)
				{
					int i_row = it_i->first - i; // 新行号
					double value = -it_i->second * (1.0 / Naa) * na;
				    m_data[i_row].nx += value; 
				}
				// Nxx 动态更新维数删除一列
				//for(int j = i; j < n; j++)
				//{
				//	map<int, double>::iterator it_j = m_data[j - i].Nxx_row.find(j);
				//	if(it_j !=  m_data[j - i].Nxx_row.end())
				//		m_data[j - i].Nxx_row.erase(it_j);
				//}
				m_data[0].Nxx_row.erase(m_data[0].Nxx_row.begin());
				// 保存 Naa Nab na
				PE_ELIEDROW pe_eliedrow;
				pe_eliedrow.Naa = Naa;
				pe_eliedrow.na = na;
				for(map<int, double>::iterator it_i = m_data[0].Nxx_row.begin(); it_i != m_data[0].Nxx_row.end(); ++it_i)
				{
					pe_eliedrow.Nab.insert(map<int, double>::value_type(it_i->first, it_i->second));
				}
				m_EliedDataList.push_back(pe_eliedrow);
				// Nxx, nx 动态更新维数删除一行
				m_data.erase(m_data.begin());
			}
			// 待消除的参数逐个处理后，进行参数求解和参数还原
			double db = (1.0/m_data[0].Nxx_row.begin()->second)*m_data[0].nx;
			m_dx.insert(map<int, double>::value_type(n-1, db)); // 第n个参数解
			// 需动态更新 db
			for(int i = n-2; i >= 0; i--)
			{
				double value = 0.0;
				for(map<int, double>::iterator it_i = m_EliedDataList[i].Nab.begin(); it_i != m_EliedDataList[i].Nab.end(); ++it_i)
				{
					map<int, double>::iterator it = m_dx.find(it_i->first);
					value += it_i->second * it->second;
				}
				double da = (1.0/m_EliedDataList[i].Naa)*(m_EliedDataList[i].na - value);
				m_dx.insert(map<int, double>::value_type(i, da));
			}
			// 输出参数解,并调整到原始顺序
			dx.Init(n,1);
			for(int i = 0; i < n; i++)
			{
				map<int, double>::iterator it = m_dx.find(m_sequenceList[i]);
				dx.SetElement(i,0,it->second);
			}
			return true;
		}
	}
}
