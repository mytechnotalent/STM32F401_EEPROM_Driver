<img src="https://raw.githubusercontent.com/mytechnotalent/STM32F401_EEPROM_Driver/refs/heads/main/STM32F401_EEPROM_Driver.png">

## FREE Reverse Engineering Self-Study Course [HERE](https://github.com/mytechnotalent/Reverse-Engineering-Tutorial)

<br>

# STM32F401 EEPROM Driver
An STM32F401 EEPROM driver written entirely in Assembler.

<br>

# Code
```
/**
 * FILE: main.s
 *
 * DESCRIPTION:
 * An STM32F401 EEPROM driver written entirely in Assembler.
 *
 * AUTHOR: Kevin Thomas
 * CREATION DATE: March 7, 2024
 * UPDATE DATE: June 22, 2025
 */

.syntax unified                                       // use unified assembly syntax
.cpu cortex-m4                                        // target Cortex-M4 core
.fpu softvfp                                          // use software floating point
.thumb                                                // use Thumb instruction set

/**
 * The start address for the .data section defined in linker script.
 */
.word _sdata                                          // start of .data

/**
 * The end address for the .data section defined in linker script.
 */
.word _edata                                          // end of .data

/**
 * The start address for the initialization values of the .data section defined in
 * linker script.
 */
.word _sidata                                         // start of .data init values

/**
 * The start address for the .bss section defined in linker script.
 */
.word _sbss                                           // start of .bss

/**
 * The end address for the .bss section defined in linker script.
 */
.word _ebss                                           // end of .bss

/**
 * Provide weak aliases for each Exception handler to the Default_Handler. As they
 * are weak aliases, any function with the same name will override this definition.
 */
.macro weak name
  .global \name                                       // make symbol global
  .weak \name                                         // mark as weak
  .thumb_set \name, Default_Handler                   // set to Default_Handler
  .word \name                                         // vector entry
.endm

/**
 * Initialize the .isr_vector section. The .isr_vector section contains vector 
 * table.
 */
.section .isr_vector, "a"                             // vector table section

/**
 * The STM32F401RE vector table. Note that the proper constructs must be placed 
 * on this to ensure that it ends up at physical address 0x00000000.
 */
.global isr_vector                                    // export vector table
.type isr_vector, %object                             // object type
isr_vector:
  .word _estack                                       // Initial Stack Pointer
  .word Reset_Handler                                 // Reset Handler
   weak NMI_Handler                                   // NMI Handler
   weak HardFault_Handler                             // HardFault Handler
   weak MemManage_Handler                             // MemManage Handler
   weak BusFault_Handler                              // BusFault Handler
   weak UsageFault_Handler                            // UsageFault Handler
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
   weak SVC_Handler                                   // SVC Handler
   weak DebugMon_Handler                              // Debug Monitor Handler
  .word 0                                             // Reserved
   weak PendSV_Handler                                // PendSV Handler
   weak SysTick_Handler                               // SysTick Handler
  .word 0                                             // Reserved
   weak EXTI16_PVD_IRQHandler                         // EXTI Line 16 Int PVD
   weak TAMP_STAMP_IRQHandler                         // Tamper/TimeStamp Int
   weak EXTI22_RTC_WKUP_IRQHandler                    // RTC Wakeup Int
   weak FLASH_IRQHandler                              // FLASH Global Int
   weak RCC_IRQHandler                                // RCC Global Int
   weak EXTI0_IRQHandler                              // EXTI Line0 Int
   weak EXTI1_IRQHandler                              // EXTI Line1 Int
   weak EXTI2_IRQHandler                              // EXTI Line2 Int
   weak EXTI3_IRQHandler                              // EXTI Line3 Int
   weak EXTI4_IRQHandler                              // EXTI Line4 Int
   weak DMA1_Stream0_IRQHandler                       // DMA1 Stream0 Global Int
   weak DMA1_Stream1_IRQHandler                       // DMA1 Stream1 Global Int
   weak DMA1_Stream2_IRQHandler                       // DMA1 Stream2 Global Int
   weak DMA1_Stream3_IRQHandler                       // DMA1 Stream3 Global Int
   weak DMA1_Stream4_IRQHandler                       // DMA1 Stream4 Global Int
   weak DMA1_Stream5_IRQHandler                       // DMA1 Stream5 Global Int
   weak DMA1_Stream6_IRQHandler                       // DMA1 Stream6 Global Int
   weak ADC_IRQHandler                                // ADC1 Global Int
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
   weak EXTI9_5_IRQHandler                            // EXTI Line[9:5] Ints
   weak TIM1_BRK_TIM9_IRQHandle                       // TIM1 Break/TIM9 Global Int
   weak TIM1_UP_TIM10_IRQHandler                      // TIM1 Update/TIM10 Global Int
   weak TIM1_TRG_COM_TIM11_IRQHandler                 // TIM1 T/C/TIM11 Global Int
   weak TIM1_CC_IRQHandler                            // TIM1 Capture Compare Int
   weak TIM2_IRQHandler                               // TIM2 Global Int
   weak TIM3_IRQHandler                               // TIM3 Global Int
   weak TIM4_IRQHandler                               // TIM4 Global Int
   weak I2C1_EV_IRQHandler                            // I2C1 Event Int
   weak I2C1_ER_IRQHandler                            // I2C1 Error Int
   weak I2C2_EV_IRQHandler                            // I2C2 Event Int
   weak I2C2_ER_IRQHandler                            // I2C2 Error Int
   weak SPI1_IRQHandler                               // SPI1 Global Int
   weak SPI2_IRQHandler                               // SPI2 Global Int
   weak USART1_IRQHandler                             // USART1 Global Int
   weak USART2_IRQHandler                             // USART2 Global Int
  .word 0                                             // Reserved
   weak EXTI15_10_IRQHandler                          // EXTI Line[15:10] Ints
   weak EXTI17_RTC_Alarm_IRQHandler                   // RTC Alarms EXTI
   weak EXTI18_OTG_FS_WKUP_IRQHandler                 // USB OTG FS Wakeup EXTI
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
   weak DMA1_Stream7_IRQHandler                       // DMA1 Stream7 Global Int
  .word 0                                             // Reserved
   weak SDIO_IRQHandler                               // SDIO Global Int
   weak TIM5_IRQHandler                               // TIM5 Global Int
   weak SPI3_IRQHandler                               // SPI3 Global Int
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
   weak DMA2_Stream0_IRQHandler                       // DMA2 Stream0 Global Int
   weak DMA2_Stream1_IRQHandler                       // DMA2 Stream1 Global Int
   weak DMA2_Stream2_IRQHandler                       // DMA2 Stream2 Global Int
   weak DMA2_Stream3_IRQHandler                       // DMA2 Stream3 Global Int
   weak DMA2_Stream4_IRQHandler                       // DMA2 Stream4 Global Int
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
   weak OTG_FS_IRQHandler                             // USB OTG FS Global Int
   weak DMA2_Stream5_IRQHandler                       // DMA2 Stream5 Global Int
   weak DMA2_Stream6_IRQHandler                       // DMA2 Stream6 Global Int
   weak DMA2_Stream7_IRQHandler                       // DMA2 Stream7 Global Int
   weak USART6_IRQHandler                             // USART6 Global Int
   weak I2C3_EV_IRQHandler                            // I2C3 Event Int
   weak I2C3_ER_IRQHandler                            // I2C3 Error Int
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
  .word 0                                             // Reserved
   weak SPI4_IRQHandler                               // SPI4 Global Int

/**
 * @brief   This code is called when processor starts execution.
 *
 * @details This is the code that gets called when the processor first
 *          starts execution following a reset event. We first define and init 
 *          the bss section and then define and init the data section, after which
 *          the application supplied main routine is called.
 *
 * @param   None
 * @retval  None
 */
.type Reset_Handler, %function                        // function type
.global Reset_Handler                                 // export symbol
Reset_Handler:
.Reset_Handler_Setup:
  LDR   R4, =_estack                                  // load addr at end of stack R4
  MOV   SP, R4                                        // move addr at end of stack SP
  LDR   R4, =_sdata                                   // copy data seg init flash to SRAM
  LDR   R5, =_edata                                   // copy data seg init flash to SRAM
  LDR   R6, =_sidata                                  // copy data seg init flash to SRAM
  MOVS  R7, #0                                        // zero offset
  B     .Reset_Handler_Loop_Copy_Data_Init            // branch
.Reset_Handler_Copy_Data_Init:
  LDR   R8, [R6, R7]                                  // copy data seg init to regs
  STR   R8, [R4, R7]                                  // copy data seg init tp regs
  ADDS  R7, R7, #4                                    // increment offset
.Reset_Handler_Loop_Copy_Data_Init:
  ADDS  R8, R4, R7                                    // initialize the data segment
  CMP   R8, R5                                        // compare
  BCC   .Reset_Handler_Copy_Data_Init                 // branch if carry is clear
  LDR   R6, =_sbss                                    // copy bss seg init flash to SRAM
  LDR   R8, =_ebss                                    // copy bss seg init flash to SRAM
  MOVS  R7, #0                                        // zero offset
  B     .Reset_Handler_Loop_Fill_Zero_BSS             // branch
.Reset_Handler_Fill_Zero_BSS:
  STR   R7, [R6]                                      // zero fill the bss segment
  ADDS  R6, R6, #4                                    // increment pointer
.Reset_Handler_Loop_Fill_Zero_BSS:
  CMP   R6, R8                                        // compare
  BCC   .Reset_Handler_Fill_Zero_BSS                  // branch if carry is clear
.Reset_Handler_Call_Main:
  BL    main                                          // call main

/**
 * @brief   This code is called when the processor receives an unexpected Int.
 *
 * @details This simply enters an infinite loop, preserving the system state for  
 *          examination by a debugger.
 *
 * @param   None
 * @retval  None
 */
.type Default_Handler, %function                      // function type
.global Default_Handler                               // export symbol
Default_Handler:
  BKPT                                                // set processor into debug state
  B.N   Default_Handler                               // infinite loop

/**
 * Initialize the .text section. 
 * The .text section contains executable code.
 */
.section .text                                        // code section

/**
 * @brief   Entry point for EEPROM driver initialization and main loop.
 *
 * @details This function initializes the GPIO and I2C peripherals for EEPROM
 *          communication and enters an infinite loop. All code and comments 
 *          are tailored for EEPROM.
 *
 * @param   None
 * @retval  None
 */
.type main, %function
.global main
main:
.Push_Registers:
  PUSH  {R4-R12, LR}                                  // push regs R4-R12, LR stack
.GPIOB_Enable:
  BL    GPIOB_Enable                                  // enable GPIOB peripheral
.GPIOB_PB8_Alt_Function_Mode_Enable:
  BL    GPIOB_PB8_Alt_Function_Mode_Enable            // set PB8 to alt func (I2C1 SCL)
.GPIOB_PB8_Open_Drain_Enable:
  BL    GPIOB_PB8_Open_Drain_Enable                   // set PB8 to open-drain
.GPIOB_PB9_Alt_Function_Mode_Enable:
  BL    GPIOB_PB9_Alt_Function_Mode_Enable            // set PB9 to alt func (I2C1 SDA)
.GPIOB_PB9_Open_Drain_Enable:
  BL    GPIOB_PB9_Open_Drain_Enable                   // set PB9 to open-drain
.I2C1_Enable:
  BL    I2C1_Enable                                   // enable I2C1 peripheral
.I2C1_Init:
  BL    I2C1_Init                                     // initialize I2C1 for EEPROM
.EEPROM_Write_Byte_16bit:
  MOV   R0, #0x50                                     // EEPROM I2C device address
  MOV   R1, #0x00                                     // EEPROM memory address high byte
  MOV   R2, #0x00                                     // EEPROM memory address low byte
  MOV   R3, #0x11                                     // data byte to write
  BL    EEPROM_Write_Byte_16bit                       // write byte to EEPROM
  BL    Thirty_Microsecond_Delay                      // wait for write cycle
.EEPROM_Read_Byte_16bit:
  MOV   R0, #0x50                                     // EEPROM I2C device address
  MOV   R1, #0x00                                     // EPROM memory address high byte
  MOV   R2, #0x00                                     // EEPROM memory address low byte
  BL    EEPROM_Read_Byte_16bit                        // read byte from EEPROM
.Loop:
  BL    Loop                                          // enter infinite loop
.Pop_Registers:
  POP   {R4-R12, LR}                                  // pop regs R4-R12, LR stack
  BX    LR                                            // return to caller

/**
 * @brief   Enables the GPIOB peripheral by setting the corresponding RCC_AHB1ENR bit.
 *
 * @details This function enables the GPIOB peripheral by setting the corresponding 
 *          RCC_AHB1ENR bit. It loads the address of the RCC_AHB1ENR register, retrieves
 *          the current value of the register, sets the GPIOBEN bit, and stores the 
 *          updated value back into the register.
 *
 * @param   None
 * @retval  None
 */
GPIOB_Enable:
.GPIOB_Enable_Push_Registers:
  PUSH  {R4-R12, LR}                                  // push regs R4-R12, LR stack
.GPIOB_Enable_Load_RCC_AHB1ENR:
  LDR   R4, =0x40023830                               // RCC_AHB1ENR register address
  LDR   R5, [R4]                                      // load value from RCC_AHB1ENR
  ORR   R5, #(1<<1)                                   // set GPIOBEN bit
  STR   R5, [R4]                                      // store value back
.GPIOB_Enable_Pop_Registers:
  POP   {R4-R12, LR}                                  // pop regs R4-R12, LR stack
  BX    LR                                            // return to caller

/**
 * @brief   Configures PB8 as I2C1 SCL (alternate function, open-drain).
 *
 * @details Sets PB8 to alternate function mode and open-drain for I2C1 SCL line.
 *
 * @param   None
 * @retval  None
 */
GPIOB_PB8_Alt_Function_Mode_Enable:
.GPIOB_PB8_Alt_Function_Mode_Enable_Push_Registers:
  PUSH  {R4-R12, LR}                                  // push regs R4-R12, LR stack
.GPIOB_PB8_Alt_Function_Mode_Enable_Set_Moder:
  LDR   R4, =0x40020400                               // GPIOB_MODER register address
  LDR   R5, [R4]                                      // load value from GPIOB_MODER
  ORR   R5, #(1<<17)                                  // set MODER8[1]
  BIC   R5, #(1<<16)                                  // clear MODER8[0]
  STR   R5, [R4]                                      // store value back
.GPIOB_PB8_Alt_Function_Mode_Enable_Set_AFRH:
  LDR   R4, =0x40020424                               // GPIOB_AFRH register address
  LDR   R5, [R4]                                      // load value from GPIOB_AFRH
  BIC   R5, #(1<<3)                                   // clear AFRH8[3]
  ORR   R5, #(1<<2)                                   // set AFRH8[2]
  BIC   R5, #(1<<1)                                   // clear AFRH8[1]
  BIC   R5, #(1<<0)                                   // clear AFRH8[0]
  STR   R5, [R4]                                      // store value back
.GPIOB_PB8_Alt_Function_Mode_Enable_Pop_Registers:
  POP   {R4-R12, LR}                                  // pop regs R4-R12, LR stack
  BX    LR                                            // return to caller

/**
 * @brief   Configures PB8 as open-drain for I2C1 SCL.
 *
 * @details Sets PB8 to open-drain output type for I2C1 SCL line.
 *
 * @param   None
 * @retval  None
 */
GPIOB_PB8_Open_Drain_Enable:
.GPIOB_PB8_Open_Drain_Enable_Push_Registers:
  PUSH  {R4-R12, LR}                                  // push regs R4-R12, LR stack
.GPIOB_PB8_Open_Drain_Enable_Set_OTYPER:
  LDR   R4, =0x40020404                               // GPIOB_OTYPER register address
  LDR   R5, [R4]                                      // load value from GPIOB_OTYPER
  ORR   R5, #(1<<8)                                   // set OT8 bit
  STR   R5, [R4]                                      // store value back
.GPIOB_PB8_Open_Drain_Enable_Pop_Registers:
  POP   {R4-R12, LR}                                  // pop regs R4-R12, LR stack
  BX    LR                                            // return to caller

/**
 * @brief   Configures PB9 as I2C1 SDA (alternate function, open-drain).
 *
 * @details Sets PB9 to alternate function mode and open-drain for I2C1 SDA line.
 *
 * @param   None
 * @retval  None
 */
GPIOB_PB9_Alt_Function_Mode_Enable:
.GPIOB_PB9_Alt_Function_Mode_Enable_Push_Registers:
  PUSH  {R4-R12, LR}                                  // push regs R4-R12, LR stack
.GPIOB_PB9_Alt_Function_Mode_Enable_Set_Moder:
  LDR   R4, =0x40020400                               // GPIOB_MODER register address
  LDR   R5, [R4]                                      // load value from GPIOB_MODER
  ORR   R5, #(1<<19)                                  // set MODER9[1]
  BIC   R5, #(1<<18)                                  // clear MODER9[0]
  STR   R5, [R4]                                      // store value back
.GPIOB_PB9_Alt_Function_Mode_Enable_Set_AFRH:
  LDR   R4, =0x40020424                               // GPIOB_AFRH register address
  LDR   R5, [R4]                                      // load value from GPIOB_AFRH
  BIC   R5, #(1<<7)                                   // clear AFRH9[3]
  ORR   R5, #(1<<6)                                   // set AFRH9[2]
  BIC   R5, #(1<<5)                                   // clear AFRH9[1]
  BIC   R5, #(1<<4)                                   // clear AFRH9[0]
  STR   R5, [R4]                                      // store value back
.GPIOB_PB9_Alt_Function_Mode_Enable_Pop_Registers:
  POP   {R4-R12, LR}                                  // pop regs R4-R12, LR stack
  BX    LR                                            // return to caller

/**
 * @brief   Configures PB9 as open-drain for I2C1 SDA.
 *
 * @details Sets PB9 to open-drain output type for I2C1 SDA line.
 *
 * @param   None
 * @retval  None
 */
GPIOB_PB9_Open_Drain_Enable:
.GPIOB_PB9_Open_Drain_Enable_Push_Registers:
  PUSH  {R4-R12, LR}                                  // push regs R4-R12, LR stack
.GPIOB_PB9_Open_Drain_Enable_Set_OTYPER:
  LDR   R4, =0x40020404                               // GPIOB_OTYPER register address
  LDR   R5, [R4]                                      // load value from GPIOB_OTYPER
  ORR   R5, #(1<<9)                                   // set OT9 bit
  STR   R5, [R4]                                      // store value back
.GPIOB_PB9_Open_Drain_Enable_Pop_Registers:
  POP   {R4-R12, LR}                                  // pop regs R4-R12, LR stack
  BX    LR                                            // return to caller

/**
 * @brief   Enables I2C1 Peripheral for EEPROM communication.
 *
 * @details Enables the I2C1 peripheral by setting the I2C1EN bit in the RCC_APB1ENR 
 *          register.
 *
 * @param   None
 * @retval  None
 */
I2C1_Enable:
.I2C1_Enable_Push_Registers:
  PUSH  {R4-R12, LR}                                  // push regs R4-R12, LR stack
.I2C1_Enable_Set_APB1ENR:
  LDR   R4, =0x40023840                               // RCC_APB1ENR register address
  LDR   R5, [R4]                                      // load value from RCC_APB1ENR
  ORR   R5, #(1<<21)                                  // set I2C1EN bit
  STR   R5, [R4]                                      // store value back
.I2C1_Enable_Pop_Registers:
  POP   {R4-R12, LR}                                  // pop regs R4-R12, LR stack
  BX    LR                                            // return to caller

/**
 * @brief   Initializes I2C1 peripheral for EEPROM operation.
 *
 * @details Configures I2C1 registers for standard EEPROM I2C operation (100kHz, 7-bit 
 *          addr).
 *
 * @param   None
 * @retval  None
 */
I2C1_Init:
.I2C1_Init_Push_Registers:
  PUSH  {R4-R12, LR}                                  // push regs R4-R12, LR stack
.I2C1_Init_Reset_CR1:
  LDR   R4, =0x40005400                               // I2C1_CR1 register address
  LDR   R5, [R4]                                      // load value from I2C1_CR1
  ORR   R5, #(1<<15)                                  // set SWRST bit (software reset)
  STR   R5, [R4]                                      // store value back
  BIC   R5, #(1<<15)                                  // clear SWRST bit
  STR   R5, [R4]                                      // store value back
.I2C1_Init_Set_CR2:
  LDR   R4, =0x40005404                               // I2C1_CR2 register address
  LDR   R5, [R4]                                      // load value from I2C1_CR2
  ORR   R5, #(1<<5)                                   // set FREQ[5] 50 MHz (PIML)
  ORR   R5, #(1<<4)                                   // set FREQ[5] 50 MHz (PIML)
  BIC   R5, #(1<<3)                                   // clear FREQ[5] 50 MHz (PIML)
  BIC   R5, #(1<<2)                                   // clear FREQ[5] 50 MHz (PIML)
  ORR   R5, #(1<<1)                                   // set FREQ[5] 50 MHz (PIML)
  BIC   R5, #(1<<0)                                   // clear FREQ[5] 50 MHz (PIML)
  STR   R5, [R4]                                      // store value back
.I2C1_Init_Set_CCR:
  LDR   R4, =0x4000541C                               // I2C1_CCR register address
  LDR   R5, [R4]                                      // load value from I2C1_CCR
  ORR   R5, #(1<<15)                                  // set F/S bit (fast/std mode)
  ORR   R5, #(1<<14)                                  // set DUTY bit
  ORR   R5, #(1<<1)                                   // set CCR[1] (clock control)
  STR   R5, [R4]                                      // store value back
  LDR   R4, =0x40005420                               // I2C1_TRISE register address
  LDR   R5, [R4]                                      // load value from I2C1_TRISE
  BIC   R5, #(1<<5)                                   // clear TRISE[5] (max rise time)
  ORR   R5, #(1<<4)                                   // set TRISE[4] (max rise time)           
  BIC   R5, #(1<<3)                                   // clear TRISE[5] (max rise time)
  ORR   R5, #(1<<2)                                   // set TRISE[4] (max rise time) 
  BIC   R5, #(1<<1)                                   // clear TRISE[5] (max rise time)
  BIC   R5, #(1<<0)                                   // clear TRISE[4] (max rise time)
  STR   R5, [R4]                                      // store value back
  LDR   R4, =0x40005400                               // I2C1_CR1 register address
  LDR   R5, [R4]                                      // load value from I2C1_CR1
  ORR   R5, #(1<<0)                                   // set PE bit (peripheral enable)
  STR   R5, [R4]                                      // store value back
.I2C1_Init_Pop_Registers:
  POP   {R4-R12, LR}                                  // pop regs R4-R12, LR stack
  BX    LR                                            // return to caller

/**
 * @brief   Writes a byte to the EEPROM via I2C1 with 16-bit addressing.
 *
 * @details This function writes a byte to the EEPROM using I2C1 with 16-bit memory 
 *          addressing. It sends the device address, high byte of memory address, 
 *          low byte of memory address, and data byte, then generates a stop condition.
 *
 * @param   R0: EEPROM I2C device address (7-bit, e.g., 0x50 for 0xA0)
 * @param   R1: EEPROM memory address high byte
 * @param   R2: EEPROM memory address low byte  
 * @param   R3: Data byte to write
 * @retval  None
 */
EEPROM_Write_Byte_16bit:
.EEPROM_Write_Byte_16bit_Push_Registers:
  PUSH  {R4-R12, LR}                                  // push regs R4-R12, LR stack
.EEPROM_Write_Byte_16bit_Init_Load_Registers:
  LDR   R4, =0x40005400                               // I2C1_CR1 register
  LDR   R5, =0x40005410                               // I2C1_DR register
  LDR   R6, =0x40005414                               // I2C1_SR1 register
  LDR   R7, =0x40005418                               // I2C1_SR2 register
.EEPROM_Write_Byte_16bit_Generate_Start:
  LDR   R8, [R4]                                      // read CR1
  ORR   R8, #(1<<8)                                   // set START
  STR   R8, [R4]                                      // write CR1
.EEPROM_Write_Byte_16bit_Wait_Start_Bit:
  LDR   R8, [R6]                                      // read SR1
  TST   R8, #1                                        // test SB
  BEQ   .EEPROM_Write_Byte_16bit_Wait_Start_Bit       // wait until set
.EEPROM_Write_Byte_16bit_Send_Device_Address:
  LSL   R8, R0, #1                                    // left-align address, write=0
  STR   R8, [R5]                                      // write DR
.EEPROM_Write_Byte_16bit_Wait_For_Addr:
  LDR   R8, [R6]                                      // read SR1
  TST   R8, #(1<<1)                                   // test ADDR
  BEQ   .EEPROM_Write_Byte_16bit_Wait_For_Addr        // wait until set
  LDR   R8, [R7]                                      // clear ADDR by reading SR2
.EEPROM_Write_Byte_16bit_Send_Memory_Address_High_Byte:
  STR   R1, [R5]                                      // write DR
.EEPROM_Write_Byte_16bit_Wait_For_TxE_1:
  LDR   R8, [R6]                                      // read SR1
  TST   R8, #(1<<7)                                   // test TxE
  BEQ   .EEPROM_Write_Byte_16bit_Wait_For_TxE_1       // wait until set
.EEPROM_Write_Byte_16bit_Send_Memory_Address_Low_Byte:
  STR   R2, [R5]                                      // write DR
.EEPROM_Write_Byte_16bit_Wait_For_TxE_2:
  LDR   R8, [R6]                                      // read SR1
  TST   R8, #(1<<7)                                   // test TxE
  BEQ   .EEPROM_Write_Byte_16bit_Wait_For_TxE_2       // wait until set
.EEPROM_Write_Byte_16bit_Send_Data_Byte:
  STR   R3, [R5]                                      // write DR
.EEPROM_Write_Byte_16bit_Wait_For_TxE_3:
  LDR   R8, [R6]                                      // read SR1
  TST   R8, #(1<<7)                                   // test TxE
  BEQ   .EEPROM_Write_Byte_16bit_Wait_For_TxE_3       // wait until set
.EEPROM_Write_Byte_16bit_Generate_Stop:
  LDR   R8, [R4]                                      // read CR1
  ORR   R8, #(1<<9)                                   // set STOP
  STR   R8, [R4]                                      // write CR1
.EEPROM_Write_Byte_16bit_Pop_Registers:
  POP   {R4-R12, LR}                                  // pop regs R4-R12, LR stack
  BX    LR                                            // return to caller

/**
 * @brief   Reads a byte from the EEPROM via I2C1 with 16-bit addressing.
 *
 * @details This function reads a byte from the EEPROM using I2C1 with 16-bit memory 
 *          addressing. It sends the device address, high byte of memory address, low 
 *          byte of memory address, then a repeated start and device address (read), 
 *          and reads the data byte with proper NACK generation for single byte read.
 *
 * @param   R0: EEPROM I2C device address (7-bit, e.g., 0x50 for 0xA0)
 * @param   R1: EEPROM memory address high byte
 * @param   R2: EEPROM memory address low byte
 * @retval  R0: Data byte read
 */
EEPROM_Read_Byte_16bit:
.EEPROM_Read_Byte_16bit_Push_Registers:
  PUSH  {R4-R12, LR}                                  // push regs R4-R12, LR stack
.EEPROM_Read_Byte_16bit_Init_Load_Registers:
  LDR   R4, =0x40005400                               // I2C1_CR1 register
  LDR   R5, =0x40005410                               // I2C1_DR register
  LDR   R6, =0x40005414                               // I2C1_SR1 register
  LDR   R7, =0x40005418                               // I2C1_SR2 register
.EEPROM_Read_Byte_16bit_Generate_Start:
  LDR   R8, [R4]                                      // read CR1
  ORR   R8, #(1<<8)                                   // set START
  STR   R8, [R4]                                      // write CR1
.EEPROM_Read_Byte_16bit_Wait_Start_Bit_1:
  LDR   R8, [R6]                                      // read SR1
  TST   R8, #1                                        // test SB
  BEQ   .EEPROM_Read_Byte_16bit_Wait_Start_Bit_1      // wait until set
.EEPROM_Read_Byte_16bit_Send_Device_Address_1:
  LSL   R8, R0, #1                                    // left-align address, write=0
  STR   R8, [R5]                                      // write DR
.EEPROM_Read_Byte_16bit_Wait_For_Addr_1:
  LDR   R8, [R6]                                      // read SR1
  TST   R8, #(1<<1)                                   // test ADDR
  BEQ   .EEPROM_Read_Byte_16bit_Wait_For_Addr_1       // wait until set
  LDR   R8, [R7]                                      // clear ADDR by reading SR2
.EEPROM_Read_Byte_16bit_Send_Memory_Address_High_Byte:
  STR   R1, [R5]                                      // write DR
.EEPROM_Read_Byte_16bit_Wait_For_TxE_1:
  LDR   R8, [R6]                                      // read SR1
  TST   R8, #(1<<7)                                   // test TxE
  BEQ   .EEPROM_Read_Byte_16bit_Wait_For_TxE_1        // wait until set
.EEPROM_Read_Byte_16bit_Send_Memory_Address_Low_Byte:
  STR   R2, [R5]                                      // write DR
.EEPROM_Read_Byte_16bit_Wait_For_TxE_2:
  LDR   R8, [R6]                                      // read SR1
  TST   R8, #(1<<7)                                   // test TxE
  BEQ   .EEPROM_Read_Byte_16bit_Wait_For_TxE_2        // wait until set
.EEPROM_Read_Byte_16bit_Generate_Repeated_Start:
  LDR   R8, [R4]                                      // read CR1
  ORR   R8, #(1<<8)                                   // set START
  STR   R8, [R4]                                      // write CR1
.EEPROM_Read_Byte_16bit_Wait_Start_Bit_2:
  LDR   R8, [R6]                                      // read SR1
  TST   R8, #1                                        // test SB
  BEQ   .EEPROM_Read_Byte_16bit_Wait_Start_Bit_2      // wait until set
.EEPROM_Read_Byte_16bit_Send_Device_Address_2:
  LSL   R8, R0, #1                                    // left-align address
  ORR   R8, #1                                        // set read=1
  STR   R8, [R5]                                      // write DR
.EEPROM_Read_Byte_16bit_Wait_For_Addr_2:
  LDR   R8, [R6]                                      // read SR1
  TST   R8, #(1<<1)                                   // test ADDR
  BEQ   .EEPROM_Read_Byte_16bit_Wait_For_Addr_2       // wait until set
.EEPROM_Read_Byte_16bit_Disable_ACK_For_Single_Byte_Read: 
  LDR   R8, [R4]                                      // read CR1
  BIC   R8, #(1<<10)                                  // clear ACK bit
  STR   R8, [R4]                                      // write CR1
  LDR   R8, [R7]                                      // clear ADDR by reading SR2
.EEPROM_Read_Byte_16bit_Generate_Stop_Before_Read:
  LDR   R8, [R4]                                      // read CR1
  ORR   R8, #(1<<9)                                   // set STOP
  STR   R8, [R4]                                      // write CR1
.EEPROM_Read_Byte_16bit_Wait_For_RxNE:
  LDR   R8, [R6]                                      // read SR1
  TST   R8, #(1<<6)                                   // test RxNE
  BEQ   .EEPROM_Read_Byte_16bit_Wait_For_RxNE         // wait until set
.EEPROM_Read_Byte_16bit_Read_Data_Byte:
  LDR   R0, [R5]                                      // read DR
.EEPROM_Read_Byte_16bit_Re_Enable_ACK:
  LDR   R8, [R4]                                      // read CR1
  ORR   R8, #(1<<10)                                  // set ACK bit
  STR   R8, [R4]                                      // write CR1
.EEPROM_Read_Byte_16bit_Pop_Registers:
  POP   {R4-R12, LR}                                  // pop regs R4-R12, LR stack
  BX    LR                                            // return to caller

/**
 * @brief   Delay for approximately 30 microseconds.
 *
 * @details This function creates a delay of approximately 30 microseconds at 16 MHz 
 *          system clock.
 *
 * @param   None
 * @retval  None
 */
Thirty_Microsecond_Delay:
.Thirty_Microsecond_Delay_Push_Registers:
  PUSH  {R4-R12, LR}                                  // push regs R4-R12, LR stack
  MOV   R4, #7                                        // number of loops
.Thirty_Microsecond_Delay_Outer_Loop:
  MOV   R5, #0xA0                                     // set initial delay cnt
.Thirty_Microsecond_Delay_Inner_Loop:
  SUB   R5, #1                                        // decrement delay cnt
  CMP   R5, #0                                        // chk delay cnt reached zero
  BNE   .Thirty_Microsecond_Delay_Inner_Loop          // cont loop delay cnt not zero
  SUB   R4, #1                                        // decrement loop cnter
  CMP   R4, #0                                        // chk if delay cnt zero
  BNE   .Thirty_Microsecond_Delay_Outer_Loop          // cont outer loop if more
.Thirty_Microsecond_Delay_Pop_Registers:
  POP   {R4-R12, LR}                                  // pop regs R4-R12, LR stack
  BX    LR                                            // return to caller

/**
 * @brief   Infinite loop function.
 *
 * @details This function implements an infinite loop using an unconditional branch
 *          (B) statement. It is designed to keep the program running indefinitely 
 *          by branching back to itself.
 *
 * @param   None
 * @retval  None
 */
Loop:
  B     .                                             // branch infinite loop

/**
 * Test data and constants.
 * The .rodata section is used for constants and static data.
 */
.section .rodata                                      // read-only data section

/**
 * Initialized global data.
 * The .data section is used for initialized global or static variables.
 */
.section .data                                        // data section

/**
 * Uninitialized global data.
 * The .bss section is used for uninitialized global or static variables.
 */
.section .bss                                         // BSS section
```

<br>

## License
[MIT](https://github.com/mytechnotalent/STM32F401_EEPROM_Driver/blob/main/LICENSE)
