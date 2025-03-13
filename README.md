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

***需要加换行
at+qdbgcfg="tracecfg",0,2 切换USB输出
at+qdbgcfg="tracecfg",0,1 切换debug输出

1/11
IOT平台协议问题：
1.里程清除？
2.充电信息？
3.卡路里不清晰,
4.踏频数据没有,
5.密码设置？

下周计划
调试震动功能逻辑， 无感解锁控制逻辑及算法， 


2/7
IOT开发
GPS测试


计划
整体代码逻辑优化
1.错误报警
2.激活逻辑(使用信号量阻塞，取消激活需要重启)
3.摔倒震动检测
4.GPS回传
5.蓝牙协议

2/11
更新IOT端蓝牙无感解锁功能
摔倒和扶起检测(已加)
船运模式
升级加超时处理

2/13
MQTT协议这块，搭建协议测试环境

2/19
调试逻辑
1.休眠逻辑



2/20计划
主要1.调试休眠这块，移动，摔倒，扶起检测这些功能。
2.MQTT指令这些，蓝牙APP联调。

2/21
进入休眠时cat1无法烧录
bug:
cat1到时没有进入休眠，蓝牙休眠重启（加了RTC加喂狗已正常）
cat1唤醒后出现重启（中断里面不能有信号量这些）
把cat1低功耗逻辑重点测试下
调试摔倒扶起检测算法

sensor正反面需要设置参数，不仍角度异常

休眠，移动，摔倒扶起已验证OK

MQTT协议
每一个指令进行调试，打印出来
网络信息，考虑直接用AT透传


2/21
BUG，实时操作应答seq不对

CAT1几个问题：
1.切换到其他小区是否会重新注网。
不会


2/22
平台协议
1.针对数据上报
2.完善OTA
代码写完需要测试。
优化HTTPOTA升级状态机

下周工作计划：
MQTT协议版本信息，配置信息设置，心跳。车端数据确认。签到
找XD确认版本格式
加心跳代码

2/23
进入船运模式
1.关锁
2.蓝牙断电 （硬件增加对BLE断电）
3.MCU进入船运（硬件增加MCU对48V电源检测）

4.车辆状态
抽时间把代码备注下

1.账号提交代码管控
2.安装AI插件


梳理IOT功能测试用例
2/26
测试车辆状态和车配置功能


2/27
1.解决MQTT收不到平台数据
2.测试状态


2/28
a5a6 b100 14 00001000 0000 01 0005 67c1 0ee6 00 d0 干崩系统

a5a6b1001400001000000907010567c11b1e00de


3/3
OTA功能调试
完善IOT功能
梳理IOT测试项 并测试
不提交SDK部分

3/4
快充断开检测
遗留问题：
1.激活 待激活状态
2.小电池电量（ADC 12位 4096）
3.船运模式

3/5
解决围栏坐标保存异常，CRC效验失败，字节对齐导致，需要8字节对齐
解决获取围栏坐标系统重启，内存异常导致
解决GNSS位置不准，坐标转换问题导致

待解决
1.报警，多个报警同时出现，正常时

3/6
用app出去骑行测试

3/7
蓝牙连接问题,通讯问题


3/8
完善蓝牙异常处理，增加蓝牙心跳包，还有蓝牙重启后发送报文。
时间溢出
cat1运行几个小时后出现重启，


3/10
MQTT重新连接成功后重启

计划抓重启log
压测OTA，持续测试IOT软件功能及异常


3/11
cat1出现烧录不了，切换电脑USB口
dump配置
at+qdbgcfg="dumpcfg",0

3/13
mqtt的keep_alive需要设置为60，太小容易掉线。申请内存会出现蓝屏。
