# Arduino for Mecarun

## where the project from
the support for MecaRun_V2 in Arduino ide is developed from the project *STM32GENERIC*, powered by huaweiwx.

Generic implementation of Arduino for STM32 boards using STM32 HAL. This is an alternative to the [Official implementation](https://github.com/stm32duino/Arduino_Core_STM32) 

Documentation: https://danieleff.github.io/STM32GENERIC/

[for more details about huaweiwx](https://github.com/huaweiwx/STM32GENERIC)

## Installation - Users

1. Download the latest version from [https://github.com/Sumn255/STM32GENERIC](https://github.com/Sumn255/STM32GENERIC)
2. make a new directroy call *stm32* (or as you like) under your Arduino's install directory(is *"C:\Program Files (x86)\Arduino"* by default). Unzip STM32 into [Arduino]/hardware/stm32 folder (Arduino must be ver1.8.5 or newer)
3. Move the files in hardware/STM32GENERIC/tools folder to hardware/tools folder for allow other arduino arm can sharing it(if you use windows,you don't need to move *linux*, *linux64* or *macos* directory).
4. Change tmp directory in windows to prevent the file name exceeding limit. Create a folder c:\Tmp
   Modify environment variables TEMP and TMP as: 
   TEMP=c:\Tmp
   TMP=c:\Tmp
   Because of the Windows command line length limit, we need to modify the environment variables to compile more files (Such as uCGUI)
5. if you use the MecaRun board, please set the Arduino ide as follow:
    Board：F103RC/D/E(72MHz)
	CPU：MECARUN_F103RC/D/E[LED PD2]
	Startup adr: flash(256k+48k)
	OSC Speed(MHz): 8M(HSE)_72M
	USB: Disable USB
	Serial communciation: SerialUARTs[PA2/PA3]
	JTAG/SWD: SWD
	Extern lib: NONE
	Upload method:Jlink 或stlink（这两个均已测试过，暂不支持串口下载）
	Debug Log_level: Release(no-exceptions)

## 安装教程
1. 下载最新版的仓库[https://github.com/Sumn255/STM32GENERIC]，使用git clone命令同步仓库或者页面右上角点击"clone or download"选择"Download ZIP",并解压
2. 找到arduino的安装目录，默认是"C:\Program Files (x86)\Arduino"，在其中的hardware目录下新建目录stm32，将克隆或者解压出来的"STM32"目录解压到创建的stm32目录
3. 将克隆或者解压下来的"tools"目录下的文件移动去[Arduino安装目录]/hardware/tools下（linux、linux64和macos目录可以不复制）.
4. 为了解决文件过多时候编译失败的问题（windows系统限制），建议修改环境变量，方法是先新建C:\Tmp文件夹，目录名字尽总长度可能短，然后去控制面板里相关设置界面修改 TEMP 和 TMP 均为
   TEMP=c:\Tmp
   TMP=c:\Tmp
5. 为麦克纳姆车主板V2编译固件时的参数请选择如下参数
	开发板：F103RC/D/E(72MHz)
	处理器：MECARUN_F103RC/D/E[LED PD2]
	Startup adr: flash(256k+48k)
	OSC Speed(MHz): 8M(HSE)_72M
	USB: Disable USB
	Serial communciation: SerialUARTs[PA2/PA3]
	JTAG/SWD: SWD
	Extern lib: NONE
	Upload method:Jlink 或stlink（这两个均已测试过，暂不支持串口下载）
	Debug Log_level: Release(no-exceptions)
