## 通讯格式规定

上位机通讯采用 RS485 协议，数据格式为 8N1, 波特率为 115200。

通讯采用请求应答式，指令为一帧 8 位数据，定义如下：

指令|说明
---|---
0x01|读三个编码器的绝对位置，数据格式见下文

### 应答数据格式

#### 0x01 读绝对位置

字节 | byte0 | byte1 | byte2 | byte3 | byte4 | byte5 | byte6 | byte7 | byte8
---- | ----- | ----- | ----- | ----- | ----- | ----- | ----- | ----- | -----
数据帧 | AS00 | AS01 | AS02 | AS10 | AS11 | AS12 | AS20 | AS21 | AS22 

AS<sub>ij</sub> 表示第 i 个编码器的第 j 字节数据，在总共 24 位的数据中 AS<sub>i0</sub> 位于低位，AS<sub>i2</sub> 位于高位，AS<sub>i2</sub> 的高 3 位始终为 `0`，即有效位数为 21 位。

若 AS<sub>i2</sub> 的高 3 位为 `1`, 表示板子尚未读到数据，或发生其它错误，可以据此判断数据的有效性。

> 通讯器向上位机发送数据时，byte0 先发送，byte8 后发送。