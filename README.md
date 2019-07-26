# UAV-Vision-Servo 

我们给出了MBZIRC 2017年无人机挑战赛，挑战1算法部分相关的源码及数据等资料。

***数据集，算法验证准则，论文对应数据相关的内容，我们会尽可能在今年9月份前补充完成。***

## datasets：数据集

我们公开所有使用的数据集。

***1. 之前数据集是将视频转换为图片序列进行处理的，导致数据集占用空间很大，现在需要对数据集格式进行重新处理，直接检测视频序列，来减少空间占用。***

***2. 18m高左右的数据是直接拟合外围椭圆得到的中心点来作为真值中心点，后续将会在此基础上人工标注十字中心点，来提高标注精度。***

***3. 数据集相关会尽量在9月份之前补充完成***

## src: 算法源码

本文件夹提供了论文中提到的SEDLines, YAED, RVIBE, C2_FTD, C1_FTD这5个算法的源代码。下面给出代码的使用方法。

- src
 + SEDLines: 简化的直线段检测算法，调用头文件 *SEDLines/Simplified_EDLines.h* 即可。
 + YAED: 经典的椭圆检测算法
 + RVIBE
 + C2_FTD
 + C1_FTD

**1. SEDLines**: 简化的直线段检测算法，调用头文件 *SEDLines/Simplified_EDLines.h* 即可。

    Simplified_EDLines sedlines(1000, 1000);                        // 构造函数，输入最大行数及最大列数，输入的数值一定要大于等于图片的行宽
    sedlines.setParame(40, 2);                                      // 设置参数，i) 表示最小的直线段长度，ii) 表示直线段的最大拟合误差
    sedlines.runSimplified_EDLines(imgSmall);                       // 检测直线段，给出直线段的像素级别的收尾点
	sedlines.getFittedLineSegments(sedlines.FittedLineSegments);    // 给出亚像素直线段的信息   (可选)
	sedlines.getLineGradients(imgSmallOpp, sedlines.LineGradients); // 给出直线段两端的梯度信息 (可选)
	sedlines.drawLineSegments4i(imgSmall);                          // 绘制检测出的直线段

**5. C1_FTD**: 合作目标的十字部分的检测算法，调用头文件 *C1_FTD/C1_FTD.h* 即可。





## projects: 工程文件

编译算法源码对应的工程，其中所有算法均被封装为动态链接库，使用控制台工程调用测试。每个算法均给出了对应的测试函数。**(在使用时候，注意头文件目录，库目录的配置)**

**1. mbzirc2017-win**： vs2017工程，基于VS2017，使用的是VS2015的开发工具集，OpenCV3.1。文件夹包含的是整个解决方案，里面有两个工程。

**第一次使用时候一定要检查 *"配置属性->VC++目录->头文件目录/库目录"* 的路径问题。**



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