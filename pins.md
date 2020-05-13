## STM32 PINS

The following table describes the usage of each pin on the STM32F405
on the v2 PCB.

The [STM32F4 datasheet](https://www.st.com/resource/en/datasheet/dm00037051.pdf)
describes more than you could ever possibly want to know about the capability of
each peripheral and the pins to which it can be mapped.


| Num | Pin  | Peripheral | Usage                        | Notes               |
| --- | ---- | ---------- | -----------------------------|----------------------
| 02  | PC13 | GPIOC      | 74HC595 SHIFT ADC pot select | |
| 03  | PC14 | GPIOC      | 74HC595 DATA ADC pot select  | |
| 04  | PC15 | GPIOC      | 74HC595 STORE ADC pot select | |
| 08  | PC0  | GPIOC      | HC165 SW_DAT  Switches       | |
| 09  | PC1  | GPIOC      | HC165 SW_CP Switches         | |
| 10  | PC2  | GPIOC      | HC165 SW_CE Switches         | |
| 11  | PC3  | GPIOC      | HC165 SW_PL Switches         | |
| 14  | PA0  | ADC1 IN0   | Analog in 1 sliders and pots | |
| 15  | PA1  | ADC2 IN1   | Analog in 2 external inputs  | |
| 20  | PA4  | DAC1 OUT   | Analog voltage out AFG1      | |
| 21  | PA5  | DAC2 OUT   | Analog voltage out AFG2      | |
| 24  | PC4  | GPIOC      | HC595 SHIFT Mode LEDs        | |
| 25  | PC5  | GPIOC      | HC595 DATA Mode LEDs         | |
| 27  | PB1  | GPIOB EXTI | AFG2 Stop banana             | |
| 33  | PB12 | SPI2       | MAX5135 DAC NSS              | Is this used ??? |
| 34  | PB13 | SPI2       | MAX5135 DAC SCK              | ??? |
| 35  | PB14 | SPI2       | MAX5135 DAC MISO             | ??? |
| 36  | PB15 | SPI2       | MAX5135 DAC MOSI             | ??? |
| 37  | PC6  | GPIOC      | 74HC595 STORE Mode LEDs      | |
| 38  | PC7  | GPIOC      | 74HC595 SHIFT Step LEDs      | |
| 39  | PC8  | GPIOC      | 74HC595 DATA Step LEDs       | |
| 40  | PC9  | GPIOC      | 74HC595 STORE Step LEDs      | |
| 41  | PA8  | GPIOA      | DIP Pin 4                    | ??? |
| 44  | PA11 | GPIOA      | DIP Pin 1                    | ??? |
| 50  | PA15 | GPIOA      | DIP Pin 2                    | Maybe SPI3 NSS ? |
| 51  | PC10 | SPI3       | CAT25512 EPROM SCK           | ??? |
| 52  | PC11 | SPI3       | CAT25512 EPROM MISO          | ??? |
| 53  | PC12 | SPI3       | CAT25512 EPROM MOSI          | ??? |
| 54  | PD2  | GPIOD      | CAT25512 EPROM CS            | What is this? |
| 57  | PB5  | GPIOB EXTI | AFG1 Strobe banana           | ??? |
| 58  | PB6  | GPIOB EXTI | AFG2 Start banana            | ??? |
| 59  | PB7  | GPIOB      | AFG2 Strobe banana           | ??? |
| 61  | PB8  | GPIOB      | AFG1 Start Banana            | ??? |
