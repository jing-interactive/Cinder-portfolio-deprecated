TuioGateway
===========

Cinder based multi-purpose TUIO server/client

开发语言：C++/Cinder

有三种工作模式 
  * Client，仅仅接收TUIO信号，并进行可视化，可以取代Ventuz来验证TUIO信号
  * Server，仅仅发送TUIO信号，输入源可以是鼠标，也可以是 Win7 的触摸消息，可以用来测试Ventuz
  * Gateway，转发器，接收TUIO信号，并且以OSC信号 /cursor/n/x 与 /cursor/n/y 进行转发，n 为从 0 开始的光标编号

![](/doc/screenshot.png "运行时截屏")

