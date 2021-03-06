TARGET=main
EXECUTABLE=main.elf

CC=arm-none-eabi-gcc
LD=arm-none-eabi-gcc
AR=arm-none-eabi-ar
AS=arm-none-eabi-as
CP=arm-none-eabi-objcopy
OD=arm-none-eabi-objdump

BIN=$(CP) -O ihex 

# Select the appropriate option for your device, the available options are listed below
# with a description copied from stm32f10x.h
# Make sure to set the startup code file to the right device family, too!
#
# STM32F10X_LD 		STM32F10X_LD: STM32 Low density devices
# STM32F10X_LD_VL	STM32F10X_LD_VL: STM32 Low density Value Line devices
# STM32F10X_MD		STM32F10X_MD: STM32 Medium density devices
# STM32F10X_MD_VL	STM32F10X_MD_VL: STM32 Medium density Value Line devices 
# STM32F10X_HD		STM32F10X_HD: STM32 High density devices
# STM32F10X_HD_VL	STM32F10X_HD_VL: STM32 High density value line devices
# STM32F10X_XL		STM32F10X_XL: STM32 XL-density devices
# STM32F10X_CL		STM32F10X_CL: STM32 Connectivity line devices 
#
# - Low-density devices are STM32F101xx, STM32F102xx and STM32F103xx microcontrollers
#   where the Flash memory density ranges between 16 and 32 Kbytes.
# 
# - Low-density value line devices are STM32F100xx microcontrollers where the Flash
#   memory density ranges between 16 and 32 Kbytes.
# 
# - Medium-density devices are STM32F101xx, STM32F102xx and STM32F103xx microcontrollers
#   where the Flash memory density ranges between 64 and 128 Kbytes.
# 
# - Medium-density value line devices are STM32F100xx microcontrollers where the 
#   Flash memory density ranges between 64 and 128 Kbytes.   
# 
# - High-density devices are STM32F101xx and STM32F103xx microcontrollers where
#   the Flash memory density ranges between 256 and 512 Kbytes.
# 
# - High-density value line devices are STM32F100xx microcontrollers where the 
#   Flash memory density ranges between 256 and 512 Kbytes.   
# 
# - XL-density devices are STM32F101xx and STM32F103xx microcontrollers where
#   the Flash memory density ranges between 512 and 1024 Kbytes.
# 
# - Connectivity line devices are STM32F105xx and STM32F107xx microcontrollers.
#
# HSE_VALUE sets the value of the HSE clock, 8MHz in this case 

DEFS = -DUSE_STDPERIPH_DRIVER -DSTM32F207xx -DHSE_VALUE=8000000
STARTUP = ../../Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F20x/startup/gcc/startup_stm32f207xx.s

MCU = cortex-m3
MCFLAGS = -mcpu=$(MCU) -mthumb -mlittle-endian -mthumb-interwork

STM32_INCLUDES = -I../../Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F20x/ \
	-I../../Libraries/CMSIS/CM3/CoreSupport/ \
	-I../../Libraries/STM32F20x_StdPeriph_Driver/inc/

OPTIMIZE       = -Os

CFLAGS	= $(MCFLAGS)  $(OPTIMIZE)  $(DEFS) -I. -I./ $(STM32_INCLUDES)  -Wl,-T,stm32_flash.ld
AFLAGS	= $(MCFLAGS) 

SRC = main.c \
	stm32f2xx_it.c \
	system_stm32f2xx.c \
	../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal.c \
	../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_adc.c \
	../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_adc_ex.c \
	../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_can.c \
	../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_cortex.c \
	../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_crc.c \
	../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_cryp.c \
	../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_dac.c \
	../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_dac_ex.c \
	../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_dcmi.c \
	../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_dma.c \
	../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_dma_ex.c \
	../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_eth.c \
	../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_flash.c \
	../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_flash_ex.c \
	../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_gpio.c \
	../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_hash.c \
	../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_hcd.c \
	../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_i2c.c \
	../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_i2s.c \
	../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_irda.c \
	../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_iwdg.c \
	../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_msp_template.c \
	../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_nand.c \
	../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_nor.c \
	../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_pccard.c \
	../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_pcd.c \
	../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_pwr.c \
	../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_pwr_ex.c \
	../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_rcc.c \
	../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_rcc_ex.c \
	../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_sd.c \
	../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_smartcard.c \
	../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_spi.c \
	../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_sram.c \
	../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_tim.c \
	../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_tim_ex.c \
	../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_uart.c \
	../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_usart.c \
	../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_wwdg.c \
	../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_ll_fsmc.c \
	../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_ll_sdmmc.c \
	../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_ll_usb.c
	
	
OBJDIR = .
OBJ = $(SRC:%.c=$(OBJDIR)/%.o) 
OBJ += Startup.o

all: $(TARGET).hex

$(TARGET).hex: $(EXECUTABLE)
	$(CP) -O ihex $^ $@

$(EXECUTABLE): $(SRC) $(STARTUP)
	$(CC) $(CFLAGS) $^ -lm -lc -lnosys -o $@

clean:
	rm -f Startup.lst  $(TARGET)  $(TARGET).lst $(OBJ) $(AUTOGEN)  $(TARGET).out  $(TARGET).hex  $(TARGET).map \
	 $(TARGET).dmp  $(EXECUTABLE)
