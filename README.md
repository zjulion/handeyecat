# HandEyeCat: Easy Hand-eye calibration tools.
* 基于[Eigen](http://eigen.tuxfamily.org)库的手眼标定算法
## `新增博客文章` 具体概念阐述可点击如下链接【中文】(In Chinese) [https://www.cnblogs.com/zjulion/p/10969576.html](https://www.cnblogs.com/zjulion/p/10969576.html)

## Description 算法描述
- 输入离散采集的机器人和相机位姿，计算出二者转移矩阵。Solving $AX=XB$.
- 实现算法： The "classic" Tsai way, [reference](http://campar.in.tum.de/view/Chair/HandEyeCalibration)：
	- *Tsai, R.Y., Lenz, R.K.*
        
        [Real Time Versatile Robotics Hand/Eye Calibration using 3D Machine Vision](http://ieeexplore.ieee.org/iel4/202/541/00012110.pdf?arnumber=12110)
        Robotics and Automation, 1988. Proceedings., 1988 IEEE International Conference on
        24-29 April 1988 Page(s):554 - 561 vol.1

        [A new technique for fully autonomous and efficient 3D robotics hand/eye calibration](http://ieeexplore.ieee.org/iel4/70/1436/00034770.pdf?arnumber=34770)
        IEEE Transactions onRobotics and Automation, 5 (3) 1989 p.345-358
    - *Shiu, Y.C., Ahmad, S.*
        [Calibration of wrist-mounted robotic sensors by solving homogeneous transform equations of the form AX=XB](http://ieeexplore.ieee.org/iel4/70/2874/00088014.pdf?isnumber=2874?=STD&arnumber=88014&arnumber=88014&arSt=16&ared=29&arAuthor=Shiu%2C+Y.C.%3B+Ahmad%2C+S)
        IEEE Transactions on Robotics and Automation, 5 (1) 1989, p.16-29


## Dependency 依赖库
- [Eigen](http://eigen.tuxfamily.org) > 3.0

## Usage 用法
```
	mkdir build
	cd build
	cmake ..
	make -j4
```
* main.cpp中表示的是一个虚拟生产的示例，可自行修改测试，并可以加入一些噪声。
