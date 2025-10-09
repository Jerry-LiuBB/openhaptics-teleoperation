# touch遥操环境配置说明

touch可以在windows及linux平台下进行遥操开发，下面介绍针对于Ubuntu平台开发的环境配置说明。

## ubuntu环境安装

安装平台需为Ubuntu 20.04 LTS version 64-bit及以上，CPU架构需要为X86。
注意1：目前不支持虚拟机

注意2：如安装过之前驱动，请先删除3dsystems/config里的缓存文件，再进行后续操作。

## touch驱动安装

首先下载[TouchDriver_2024_09_19.tgz.gz](https://github.com/Jerry-LiuBB/openhaptics-teleoperation/blob/Ubuntu20.04/TouchDriver_2024_09_19.tgz.gz)到你的home路径下，然后解压该文件，解压好的文件夹里需要包含 bin 和 usr 文件夹。

1.进入文件夹，点击鼠标右键选择在终端打开，然后输入

```
./install_haptic_driver
```

然后按照提示确认安装完成。

2.安装完成后，进入bin文件夹路径下，选择在终端打开，然后输入

```
./Touch_AdvancedConfig
```

然后根据提示进行安装，安装完成后会弹出信息界面，能够正常识别到设备，即证明安装成功。

## openhaptics开发功能包安装

首先下载[Openhaptics for Linux v3.4](https://s3.amazonaws.com/dl.3dsystems.com/binaries/support/downloads/KB+Files/Open+Haptics/openhaptics_3.4-0-developer-edition-amd64.tar.gz)，然后解压到home路径下，进入解压后的文件夹，点击鼠标右键选择在终端打开，然后输入

```
sudo ./install
```

安装过程中可能会报错缺失库文件，对应的安装库文件就可以，例如

```
sudo apt-get install libncurses5-dev freeglut3 build-essential
```

功能包将被安装在opt目录下，为OpenHaptics，进入目录，有\openhaptics_3.4-0-developer-edition-amd64\openhaptics_3.4-0-developer-edition-amd64\opt\OpenHaptics\Developer\3.4-0\examples\HD\console，里面有很多deom示例，随便进入一个示例，执行make编译，就会在当前目录下生成可执行文件，运行可执行文件即可运行touch演示demo