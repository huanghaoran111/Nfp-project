事件驱动+ringbuffer

json库 https://github.com/nlohmann/json
日志库 https://github.com/gabime/spdlog

"VS中使用format头文件"点击查看元宝的回答
https://yuanbao.tencent.com/bot/app/share/chat/Lf84U7AZU7rr

算法实现放在data_warp.h中，里面给出了示例代码实现(画线功能)
需要在Shape.cpp中加入部分代码(具体查看Shape.cpp中的TODO项 仅需填充算法 必要部分已做说明)
如果有需要加的UI功能写在这里
编译过程：
git clone https://github.com/huanghaoran111/Nfp-project.git
git submodule update --init --recursive
cd Nfp-project
mkdir build
cd build
cmake ..
cmake --build .
找一下nfpTest.exe运行
