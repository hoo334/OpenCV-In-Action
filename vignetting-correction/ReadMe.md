# vignetting-correction

来自论文[Revisiting Image Vignetting Correction by Constrained Minimization of Log-Intensity Entropy](https://academic.microsoft.com/paper/1144003762/reference/search?q=Revisiting Image Vignetting Correction by Constrained Minimization of Log-Intensity Entropy&qe=Or(Id%3D2133665775%2CId%3D1981673675%2CId%3D2139179951%2CId%3D2139851675%2CId%3D2122742285%2CId%3D2008387908%2CId%3D2129224689%2CId%3D2167566599%2CId%3D2000325779%2CId%3D1987443165%2CId%3D2161758457%2CId%3D2960552986%2CId%3D1599223134%2CId%3D2020951447%2CId%3D2198922120%2CId%3D2063698375%2CId%3D2137798898%2CId%3D2286171112)&f=&orderBy=0)。

代码与论文中实现有些差异：

- 将图像的中心直接作为摄像机光学中心
- 将熵的平滑处理部分由高斯平滑替换为简单的线性平滑



下面这些博客可能有助于更好理解这篇论文：

https://blog.csdn.net/u011776903/article/details/74637080

https://www.jianshu.com/p/34d5f2cd17aa

https://blog.csdn.net/lz0499/article/details/82054651

https://www.cnblogs.com/Imageshop/p/6166394.html

参考代码：

https://github.com/HJCYFY/Vignetting-Correction