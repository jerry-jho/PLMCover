RS485 窗帘控制器
===============

安装
----

    pip install esphome
    platformio lib --global install plerup/EspSoftwareSerial


硬件
----

ESP32主控+RS485电平转换器，将plmcover.h中的如下语句替换成你实际的串口接口

    #define MYPORT_TX 19
    #define MYPORT_RX 22

WiFi
----

创建wifi.private.yaml，内容（前面不留空格）

    ssid: "你的SSID"
    password: "你的密码"


构建
----

    make


协议
----

目前只支持杜亚窗帘协议，可以根据你自己的窗帘协议对码