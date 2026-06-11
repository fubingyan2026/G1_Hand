HPM6E8YIGN1 MCU 关键外设接线表
根据原理图（包含 MCU_PA&PB、MCU_PC&PD、MCU_PE&PF、Charge、IMU、Connector 等图纸），以下是为您整理提取的 MCU 引脚定义对照表：

| MCU 引脚 | 原理图信号名 | 外设接口类型 | 说明/备注 |
| :--- | :--- | :--- | :--- |
| **PA0** | UART0_TXD_PA0 | **UART** | 调试串口0 发送 |
| **PA1** | UART0_RXD_PA1 | **UART** | 调试串口0 接收 |
| **PA2** | BOOT0_PA02 | **BOOT** | 启动模式配置引脚 0 |
| **PA3** | BOOT1_PA03 | **BOOT** | 启动模式配置引脚 1 |
| **PA4** | JTAG_TDO_PA4 | **JTAG** | 调试接口 数据输出 TDO |
| **PA5** | JTAG_TDI_PA5 | **JTAG** | 调试接口 数据输入 TDI |
| **PA6** | JTAG_TCK_PA6 | **JTAG** | 调试接口 时钟 TCK |
| **PA7** | JTAG_TMS_PA7 | **JTAG** | 调试接口 模式选择 TMS |
| **PA8** | JTAG_TRST_PA8 | **JTAG** | 调试接口 复位 TRST |
| **PA16** | EXT_CAN4_TXD_PA16 | **CAN** | CAN4 调试发送 (经 TCAN1044VDDFR 转换) |
| **PA17** | EXT_CAN4_RXD_PA17 | **CAN** | CAN4 调试接收 (经 TCAN1044VDDFR 转换) |
| **PA18** | EXT_CAN4_STB_PA18 | **GPIO** | CAN4 收发器待机控制 (STB) |
| **PA19** | SPI3_CS_5_PA19 | **SPI** | SPI3 片选引脚 5 (引出至 CN901) |
| **PA20** | EXT_P0_RXN_PA20 | **EtherCAT** | EtherCAT 物理层接收端 P0_RXN |
| **PA21** | EXT_P0_RXP_PA21 | **EtherCAT** | EtherCAT 物理层接收端 P0_RXP |
| **PA22** | EXT_P0_TXN_PA22 | **EtherCAT** | EtherCAT 物理层发送端 P0_TXN |
| **PA23** | EXT_P0_TXP_PA23 | **EtherCAT** | EtherCAT 物理层发送端 P0_TXP |
| **PA24** | SPI3_CS_2_PA24 | **SPI** | SPI3 片选引脚 2 (引出至 CN901/CN902) |
| **PA25** | SPI3_CS_1_PA25 | **SPI** | SPI3 片选引脚 1 (引出至 CN901/CN902) |
| **PA26** | SPI3_SCLK_PA26 | **SPI** | SPI3 时钟 SCLK (引出至 CN901/CN902) |
| **PA27** | P0_RBIAS_PA27 / SPI3_CS_0_PA27 | **EtherCAT/SPI** | EtherCAT 偏置电阻极 (2.49kΩ 接地) / SPI3 片选 0 (复用) |
| **PA28** | SPI3_MISO_PA28 | **SPI** | SPI3 主入从出 MISO (引出至 CN901/CN902) |
| **PA29** | SPI3_MOSI_PA29 | **SPI** | SPI3 主出从入 MOSI (引出至 CN901/CN902) |
| **PA30** | MDIO_PA30 / SPI3_CS_3_PA30 | **SMI/SPI** | EtherCAT 物理层管理数据 (MDIO) / SPI3 片选 3 (复用) |
| **PA31** | MDC_PA31 / SPI3_CS_4_PA31 | **SMI/SPI** | EtherCAT 物理层管理时钟 (MDC) / SPI3 片选 4 (复用) |
| **PB00** | EXT_UART8_TXD_PB00 | **RS485** | 外部通讯485 发送 (经 SIT3088ETK 转换) |
| **PB01** | EXT_UART8_RXD_PB01 | **RS485** | 外部通讯485 接收 (经 SIT3088ETK 转换) |
| **PB02** | EXT_UART8_DE_PB02 | **GPIO** | 外部通讯485 使能控制 |
| **PB24** | MOTOR_UART18_TX_PB24 | **RS485** | 电机通讯485_2 发送 (经 SIT3088ETK 转换) |
| **PB25** | MOTOR_UART18_RX_PB25 | **RS485** | 电机通讯485_2 接收 (经 SIT3088ETK 转换) |
| **PB26** | MOTOR_UART18_EN_PB26 | **GPIO** | 电机通讯485_2 使能控制 |
| **PB29** | MOTOR_UART15_EN_PB29 | **GPIO** | 电机通讯485_1 使能控制 |
| **PB30** | MOTOR_UART15_RX_PB30 | **RS485** | 电机通讯485_1 接收 (经 SIT3088ETK 转换) |
| **PB31** | MOTOR_UART15_TX_PB31 | **RS485** | 电机通讯485_1 发送 (经 SIT3088ETK 转换) |
| **PC00** | SPI0_CS_4_PC00 | **SPI** | SPI0 片选引脚 4 (引出至 CN903) |
| **PC01** | SPI0_CS_3_PC01 | **SPI** | SPI0 片选引脚 3 (引出至 CN903) |
| **PC02** | SPI0_CS_2_PC02 | **SPI** | SPI0 片选引脚 2 (引出至 CN903) |
| **PC03** | SPI0_CS_1_PC03 | **SPI** | SPI0 片选引脚 1 (引出至 CN903) |
| **PC04** | SPI0_SCLK_PC04 | **SPI** | SPI0 时钟 SCLK (引出至 CN903) |
| **PC05** | SPI0_CS_0_PC05 | **SPI** | SPI0 片选引脚 0 (引出至 CN903) |
| **PC06** | SPI0_MISO_PC06 | **SPI** | SPI0 主入从出 MISO (引出至 CN903) |
| **PC07** | SPI0_MOSI_PC07 | **SPI** | SPI0 主出从入 MOSI (引出至 CN903) |
| **PC08** | SPI0_CS_5_PC08 | **SPI** | SPI0 片选引脚 5 (引出至 CN903) |
| **PC19** | SPI2_CS_5_PC19 | **SPI** | SPI2 片选引脚 5 (引出至 CN902) |
| **PC22** | SPI2_CS_4_PC22 | **SPI** | SPI2 片选引脚 4 (引出至 CN902) |
| **PC23** | SPI2_CS_3_PC23 | **SPI** | SPI2 片选引脚 3 (引出至 CN902) |
| **PC24** | SPI2_CS_2_PC24 | **SPI** | SPI2 片选引脚 2 (引出至 CN902) |
| **PC25** | SPI2_CS_1_PC25 | **SPI** | SPI2 片选引脚 1 (引出至 CN902) |
| **PC26** | SPI2_SCLK_PC26 | **SPI** | SPI2 时钟 SCLK (引出至 CN902) |
| **PC27** | SPI2_CS_0_PC27 | **SPI** | SPI2 片选引脚 0 (引出至 CN902) |
| **PC28** | SPI2_MISO_PC28 | **SPI** | SPI2 主入从出 MISO (引出至 CN902) |
| **PC29** | SPI2_MOSI_PC29 | **SPI** | SPI2 主出从入 MOSI (引出至 CN902) |
| **PF02** | LED_PF02 | **GPIO** | 状态指示灯 LED (控制 LED501) |