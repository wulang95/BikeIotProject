IOT项目相关代码


一.系统分区
参考修改partinfo_8910_8m_opencpu_novolte.json

2024/11/2
六轴调试：
1.调试了算法库，出厂参数校准，获取欧拉角。
2.震动检测逻辑调试，传感器休眠逻辑。

硬件调试：
一.GPIO
1.蓝牙连接


GPS算法
1.数据有效
2.数据无效

at+qdbgcfg="tracecfg",0,2 切换USB输出
at+qdbgcfg="tracecfg",0,1 切换debug输出