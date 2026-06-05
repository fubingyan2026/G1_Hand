HPM6E8YIGN1 MCU 关键外设接线表

| MCU 引脚 | 原理图信号名     | 外设接口类型 | 说明/备注                             |
| :------- | :--------------- | :----------- | :------------------------------------ |
| **PB30** | MOTOR\_UAR15\_RX | **RS485**    | 电机通讯485 接收 (经 SIT3088ETK 转换) |
| **PB31** | MOTOR\_UAR15\_TX | **RS485**    | 电机通讯485 发送 (经 SIT3088ETK 转换) |
| **PB29** | MOTOR\_UAR15\_EN | GPIO         | 电机通讯485 使能控制                  |
| **PB24** | MOTOR\_UART14\_TX | **RS485**    | 电机通讯485 发送 (经 SIT3088ETK 转换) |
| **PB25** | MOTOR\_UART14\_RX | **RS485**    | 电机通讯485 接收 (经 SIT3088ETK 转换) |
| **PB26** | MOTOR\_UART14\_EN | GPIO         | 电机通讯485 使能控制                  |
| **PB00** | EXT_UART8_TXD    | **RS485**    | 外部通讯485 发送 (经 SIT3088ETK 转换) |
| **PB01** | EXT_UART8_RXD    | **RS485**    | 外部通讯485 接收 (经 SIT3088ETK 转换) |
| **PB02** | EXT_UART8_EN     | GPIO         | 外部通讯485 使能控制                  |
| **PA16** | EXT\_CAN4\_TXD   | CAN          | CAN4 发送 (经 TCAN1044 转换)          |
| **PA17** | EXT\_CAN4\_RXD   | CAN          | CAN4 接收 (经 TCAN1044 转换)          |
| **PA18** | EXT\_CAN4\_STB   | GPIO         | CAN4 待机控制                         |
| **PA20** | EXT\_P0\_RXN     | EtherCAT     | P0 接收负端                           |
| **PA21** | EXT\_P0\_RXP     | EtherCAT     | P0 接收正端                           |
| **PA22** | EXT\_P0\_TXN     | EtherCAT     | P0 发送负端                           |
| **PA23** | EXT\_P0\_TXP     | EtherCAT     | P0 发送正端                           |
| **PA27** | P0\_RBIAS        | 模拟偏置     | EtherCAT PHY 偏置电阻                 |
| **PA00** | UART0\_TXD       | UART         | 调试串口发送                          |
| **PA01** | UART0\_RXD       | UART         | 调试串口接收                          |
| **PF02** | LED\_PF02        | GPIO         | LED501 状态指示灯                     |

