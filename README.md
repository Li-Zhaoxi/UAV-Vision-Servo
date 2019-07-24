# UAV-Vision-Servo 


这个工程包含了MBZIRC 2017年无人机挑战赛，挑战1算法部分相关的所有源代码。

源码目前配置VS2015，opencv3.1


算法封装为库


数据集，算法验证准则，论文对应数据相关的所有需要的内容，将会在2019年9月前全部整理完成。

## src: 源码

挑战1所有相关源码

## projects: 算法库工程

挑战1相关的所有代码均封装为动态链接库，然后使用控制台工程调用测试，每个算法均给出测试函数。目前给出了VS2015下的编译工程，使用时候需要注意头文件目录和库目录的配置。




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