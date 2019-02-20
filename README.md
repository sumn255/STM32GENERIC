# Arduino for Mecarun
Generic implementation of Arduino for STM32 boards using STM32 HAL. This is an alternative to the [Official implementation](https://github.com/stm32duino/Arduino_Core_STM32) 

Documentation: https://danieleff.github.io/STM32GENERIC/

## Installation - Users

TODO create boards manager package

## Installation - Developers
1. Download the latest version from [https://github.com/huaweiwx/STM32GENERIC](https://github.com/huaweiwx/STM32GENERIC)
2. Unzip it into [Arduino]/hardware folder (Arduino must be ver1.8.5 or newer)
3. Move the files in hardware/STM32GENERIC/tools folder to hardware/tools folder for allow other arduino arm can sharing it.
4. Install Arduino Zero or Arduino Sam from the board manager for the arm-none-eabi-gcc compiler
5. Download the [GNU ARM Embedded Toolchain](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads).
   Recommended use GCC ver 6.3.1-2017q2,Change compiler.path in [platform.txt](https://github.com/danieleff/STM32GENERIC/blob/master/STM32/platform.txt#L21) to point to that you downloaded.
   Example for default position:  C:\Users\Administrator\AppData\Local\Arduino15\packages\arduino\tools\arm-none-eabi-gcc\
6. Create a folder c:\Tmp
7. Modify environment variables TEMP and TMP as: 
   TEMP=c:\Tmp
   TMP=c:\Tmp
   Because of the Windows command line length limit, we need to modify the environment variables to compile more files (Such as uCGUI)

## 安装教程
1. 下载最新版的仓库[https://github.com/Sumn255/STM32GENERIC]，使用git clone命令同步仓库或者页面右上角点击"clone or download"选择"Download ZIP",并解压
2. 找到arduino的安装目录，默认是"C:\Program Files (x86)\Arduino"，将克隆或者解压出来的"STM32"目录解压到[Arduino安装目录]/hardware目录
3. 将克隆或者解压下来的"tools"目录下的文件复制去[Arduino安装目录]/hardware/tools下（linux、linux64和macos目录可以不复制）.
4. 打开arduino ide，找到开发板管理器，安装Arduino Zero 或者 Arduino Sam ，这是为了安装arm编译环境
5. 下载新版本的工具链arm-none-eabi，推荐2018-q2版本的，（2018-q4版本有bug不能使用！） (https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads).以该版本为例，win7以后的系统选择gcc-arm-none-eabi-7-2018-q2-update-win32-sha2.exe，xp和vista选择gcc-arm-none-eabi-7-2018-q2-update-win32-sha1.exe 
   下载并安装，默认安装目录在compiler.path=C:\Program Files (x86)\GNU Tools Arm Embedded\7 2018-q2-update\bin\ 如果不是这个版本或者目录，则需修改 [STM32/platform.txt]第22行为你编译工具链的目录
6. 将[STM32\libraries\MecaRun_V2]目录复制到[Arduino安装目录]\libraries里，方可在菜单中找到Mecarun的库
7. 为了解决文件过多时候编译失败的问题（windows系统限制），建议修改环境变量，方法是先新建C:\Tmp文件夹，目录名字尽总长度可能短，然后去控制面板里相关设置界面修改 TEMP 和 TMP 均为
   TEMP=c:\Tmp
   TMP=c:\Tmp
8. 为麦克纳姆车主板编译固件时的参数请选择如下参数
	开发板：F103RC/D/E(72MHz)
	处理器：GENRERIC_F103RC/D/E[LED PD2]
	Startup adr: flash(512k+64k)
	OSC Speed(MHz): 8M(HSE)_72M
	USB: Disable USB
	Serial communciation: SerialUARTs[PA2/PA3]
	JTAG/SWD: SWD
	Extern lib: NONE
	Upload method:Jlink 或stlink（这两个均已测试过，暂不支持串口下载）
	Debug Log_level: Release(no-exceptions)
