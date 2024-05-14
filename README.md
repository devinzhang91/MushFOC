# MushFOC
A simple FOC with stm32 hal project. With PCB schematic and Python.
Reference from [CawFOC](https://github.com/GUAIK-ORG/CawFOC) and [DengFOC](https://github.com/ToanTech/DengFOC_Lib)

## STM32
- STM32G0 series, Flash 64KB and Ram 8KB.
- Control with I2C or UART (more?).
- HAL library.
- FREERTOS.

## PCB
- MCU: STM32G031F8
- Driver: DRV8313
- Current Sensor: INA180
- Angle Sensor: AS5600
- Reserve peripherals: SSD1306(OLED) MPU6050(Gyroscope)

## Python
- With smbus tools