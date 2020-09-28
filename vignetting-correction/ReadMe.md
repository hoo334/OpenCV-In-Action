# vignetting-correction

来自论文[Revisiting Image Vignetting Correction by Constrained Minimization of Log-Intensity Entropy]。

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