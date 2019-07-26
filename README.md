# UAV-Vision-Servo 

我们给出了MBZIRC 2017年无人机挑战赛，挑战1算法部分相关的源码及数据等资料。

***数据集，算法验证准则，论文对应数据相关的内容，我们会尽可能在今年9月份前补充完成。***



## src: 算法源码

本文件夹提供了论文中提到的SEDLines, YAED, RVIBE, C2_FTD, C1_FTD这5个算法的源代码。

- src
 + SEDLines: 简化的直线段检测算法，调用 *SEDLines/Simplified_EDLines.h* 即可。
 + YAED: 经典的椭圆检测算法
 + RVIBE
 + C2_FTD
 + C1_FTD

## projects: 工程文件

编译算法源码对应的工程，其中所有算法均被封装为动态链接库，使用控制台工程调用测试。每个算法均给出了对应的测试函数。**(在使用时候，注意头文件目录，库目录的配置)**

- 工程文件
  + vs2017工程，基于VS2017，使用的是VS2015的开发工具集，OpenCV3.1。
  + ***QT5工程，方便跨平台开发，将在后续给出***
  + ***Linux工程，基于CMake，将在后续给出***
  + ***Python接口，使用Cython封装为Python可调用的包，方便测试使用，将在后续给出***
  + ***Matlab接口，使用mex封装为matlab可调用函数，方便测试使用，将在后续给出***

挑战1相关的所有代码均封装为动态链接库，然后使用控制台工程调用测试，每个算法均给出测试函数。目前给出了VS2015下的编译工程，使用时候需要注意头文件目录和库目录的配置。

算法检测结果：

- SEDLines, YAED在单张图片下的检测结果

<img src="results\\ori.jpg" width=33%> <img src="results\\SEDLines.jpg" width=33%> <img src="results\\YAED.jpg" width=33%>

- RVIBE, C2\_FTD, C1\_FTD在视频序列下的检测结果

<img src="results\\rvibe.jpg" width=33%> <img src="results\\c2_ftd.jpg" width=33%> <img src="results\\c1_ftd.jpg" width=33%>

## 参考文献

    @article{Li2019Fast,
	author = {Li, Zhaoxi and Meng, Cai and Zhou, Fugen and Ding, Xilun and Wang, Xueqiang and Zhang, Huan and Guo, Pin and Meng, Xin},
	title = {Fast vision-based autonomous detection of moving cooperative target for unmanned aerial vehicle landing},
	journal = {Journal of Field Robotics},
	volume = {36},
	number = {1},
	pages = {34-48},
	month={Jan},
	keywords = {flight strategy, moving cooperative target, target detection, unmanned aerial vehicle},
	doi = {10.1002/rob.21815},
	year = {2019}
	}