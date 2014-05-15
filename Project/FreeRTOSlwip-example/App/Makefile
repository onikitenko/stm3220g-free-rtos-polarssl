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
STARTUP = ../../../Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F20x/startup/gcc/startup_stm32f207xx.s

MCU = cortex-m3
MCFLAGS = -mcpu=$(MCU) -mthumb -mlittle-endian -mthumb-interwork

STM32_INCLUDES = -I../../../Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F20x/ \
	-I../../../Libraries/CMSIS/CM3/CoreSupport/ \
	-I../../../Libraries/STM32F20x_StdPeriph_Driver/inc/ \
	-I../../../Utilities/STM32_EVAL/STM3220G_EVAL/ \
	-I../FreeRTOS/src/portable \
	-I../FreeRTOS/inc \
	-I./include \
	-I../LwIP/port/arch/ \
	-I../LwIP/include \
	-I../LwIP/port \
	-I../LwIP/include/ipv4 \
	-I../polarssl/include

OPTIMIZE       = -Os

CFLAGS	= $(MCFLAGS)  $(OPTIMIZE)  $(DEFS) -I. -I./ $(STM32_INCLUDES)  -Wl,-T,stm32_flash.ld
AFLAGS	= $(MCFLAGS) 

POLARSSL_PREFIX=../polarssl/library

POLARSSL_SRC = \
    	$(POLARSSL_PREFIX)/aes.c	$(POLARSSL_PREFIX)/aesni.c	$(POLARSSL_PREFIX)/arc4.c	\
		$(POLARSSL_PREFIX)/asn1parse.c								\
		$(POLARSSL_PREFIX)/asn1write.c $(POLARSSL_PREFIX)/base64.c	$(POLARSSL_PREFIX)/bignum.c		\
		$(POLARSSL_PREFIX)/blowfish.c	$(POLARSSL_PREFIX)/camellia.c					\
		$(POLARSSL_PREFIX)/certs.c		$(POLARSSL_PREFIX)/cipher.c	$(POLARSSL_PREFIX)/cipher_wrap.c	\
		$(POLARSSL_PREFIX)/ctr_drbg.c	$(POLARSSL_PREFIX)/debug.c		$(POLARSSL_PREFIX)/des.c			\
		$(POLARSSL_PREFIX)/dhm.c		$(POLARSSL_PREFIX)/ecdh.c		$(POLARSSL_PREFIX)/ecdsa.c			\
		$(POLARSSL_PREFIX)/ecp.c		$(POLARSSL_PREFIX)/ecp_curves.c				\
		$(POLARSSL_PREFIX)/entropy.c	$(POLARSSL_PREFIX)/entropy_poll.c				\
		$(POLARSSL_PREFIX)/error.c		$(POLARSSL_PREFIX)/gcm.c		$(POLARSSL_PREFIX)/havege.c		\
		$(POLARSSL_PREFIX)/hmac_drbg.c								\
		$(POLARSSL_PREFIX)/md.c		$(POLARSSL_PREFIX)/md_wrap.c	$(POLARSSL_PREFIX)/md2.c			\
		$(POLARSSL_PREFIX)/md4.c		$(POLARSSL_PREFIX)/md5.c						\
		$(POLARSSL_PREFIX)/memory_buffer_alloc.c	$(POLARSSL_PREFIX)/net.c			\
		$(POLARSSL_PREFIX)/oid.c									\
		$(POLARSSL_PREFIX)/padlock.c	$(POLARSSL_PREFIX)/pbkdf2.c	$(POLARSSL_PREFIX)/pem.c			\
		$(POLARSSL_PREFIX)/pkcs5.c		$(POLARSSL_PREFIX)/pkcs11.c	$(POLARSSL_PREFIX)/pkcs12.c		\
		$(POLARSSL_PREFIX)/pk.c		$(POLARSSL_PREFIX)/pk_wrap.c	$(POLARSSL_PREFIX)/pkparse.c		\
		$(POLARSSL_PREFIX)/pkwrite.c	$(POLARSSL_PREFIX)/platform.c	$(POLARSSL_PREFIX)/ripemd160.c		\
		$(POLARSSL_PREFIX)/rsa.c		$(POLARSSL_PREFIX)/sha1.c		$(POLARSSL_PREFIX)/sha256.c		\
		$(POLARSSL_PREFIX)/sha512.c	$(POLARSSL_PREFIX)/ssl_cache.c	$(POLARSSL_PREFIX)/ssl_cli.c		\
		$(POLARSSL_PREFIX)/ssl_srv.c   $(POLARSSL_PREFIX)/ssl_ciphersuites.c			\
		$(POLARSSL_PREFIX)/ssl_tls.c	$(POLARSSL_PREFIX)/threading.c	$(POLARSSL_PREFIX)/timing.c		\
		$(POLARSSL_PREFIX)/version.c								\
		$(POLARSSL_PREFIX)/x509.c		$(POLARSSL_PREFIX)/x509_create.c				\
		$(POLARSSL_PREFIX)/x509_crl.c	$(POLARSSL_PREFIX)/x509_crt.c	$(POLARSSL_PREFIX)/x509_csr.c		\
		$(POLARSSL_PREFIX)/x509write_crt.c			$(POLARSSL_PREFIX)/x509write_csr.c	\
		$(POLARSSL_PREFIX)/xtea.c

SRC = main.c \
	app_ethernet.c \
	ethernetif.c \
	tcp_polarclient.c \
	system_stm32f2xx.c \
	stm32f2xx_hal_msp.c \
	usart.c \
	$(POLARSSL_SRC) \
	../../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal.c \
	../../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_adc.c \
	../../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_adc_ex.c \
	../../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_can.c \
	../../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_cortex.c \
	../../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_crc.c \
	../../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_cryp.c \
	../../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_dac.c \
	../../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_dac_ex.c \
	../../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_dcmi.c \
	../../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_dma.c \
	../../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_dma_ex.c \
	../../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_eth.c \
	../../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_flash.c \
	../../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_flash_ex.c \
	../../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_gpio.c \
	../../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_hash.c \
	../../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_hcd.c \
	../../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_i2c.c \
	../../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_i2s.c \
	../../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_irda.c \
	../../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_iwdg.c \
	../../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_msp_template.c \
	../../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_nand.c \
	../../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_nor.c \
	../../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_pccard.c \
	../../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_pcd.c \
	../../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_pwr.c \
	../../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_pwr_ex.c \
	../../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_rcc.c \
	../../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_rcc_ex.c \
	../../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_sd.c \
	../../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_smartcard.c \
	../../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_spi.c \
	../../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_sram.c \
	../../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_tim.c \
	../../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_tim_ex.c \
	../../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_uart.c \
	../../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_usart.c \
	../../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_hal_wwdg.c \
	../../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_ll_fsmc.c \
	../../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_ll_sdmmc.c \
	../../../Libraries/STM32F20x_StdPeriph_Driver/src/stm32f2xx_ll_usb.c \
	../FreeRTOS/src/croutine.c \
	../FreeRTOS/src/list.c \
	../FreeRTOS/src/queue.c \
	../FreeRTOS/src/tasks.c \
	../FreeRTOS/src/timers.c \
	../FreeRTOS/src/heap_1.c \
	../FreeRTOS/src/portable/port.c \
	../LwIP/api/api_lib.c \
	../LwIP/api/api_msg.c \
	../LwIP/api/err.c \
	../LwIP/api/netbuf.c \
	../LwIP/api/netdb.c \
	../LwIP/api/netifapi.c \
	../LwIP/api/sockets.c \
	../LwIP/api/tcpip.c \
	../LwIP/core/ipv4/autoip.c \
	../LwIP/core/ipv4/icmp.c \
	../LwIP/core/ipv4/igmp.c \
	../LwIP/core/ipv4/inet_chksum.c \
	../LwIP/core/ipv4/inet.c \
	../LwIP/core/ipv4/ip_addr.c \
	../LwIP/core/ipv4/ip_frag.c \
	../LwIP/core/ipv4/ip.c \
	../LwIP/core/snmp/asn1_dec.c \
	../LwIP/core/snmp/asn1_enc.c \
	../LwIP/core/snmp/mib_structs.c \
	../LwIP/core/snmp/mib2.c \
	../LwIP/core/snmp/msg_in.c \
	../LwIP/core/snmp/msg_out.c \
	../LwIP/core/def.c \
	../LwIP/core/dhcp.c \
	../LwIP/core/dns.c \
	../LwIP/core/init.c \
	../LwIP/core/mem.c \
	../LwIP/core/memp.c \
	../LwIP/core/netif.c \
	../LwIP/core/pbuf.c \
	../LwIP/core/raw.c \
	../LwIP/core/stats.c \
	../LwIP/core/sys.c \
	../LwIP/core/tcp_in.c \
	../LwIP/core/tcp_out.c \
	../LwIP/core/tcp.c \
	../LwIP/core/timers.c \
	../LwIP/core/udp.c \
	../LwIP/netif/etharp.c \
	../LwIP/netif/ethernetif.c \
	../LwIP/netif/slipif.c 
#	../LwIP/port/arch/sys_arch.c \
#	../LwIP/port/bethsetup.c \
#	../LwIP/port/stm32x_ethernetif.c
	
	
	
	
OBJDIR = .
OBJ = $(SRC:%.c=$(OBJDIR)/%.o) 
OBJ += Startup.o

all: $(TARGET).hex

$(TARGET).hex: $(EXECUTABLE)
	$(CP) -O ihex $^ $@

$(EXECUTABLE): $(SRC) $(STARTUP)
	$(CC) $(CFLAGS) $^ -lm -lc -lnosys -o $@

clean:
	rm -f Startup.lst  $(TARGET)  $(TARGET).lst $(OBJ) $(AUTOGEN)  $(TARGET).out  \
	$(TARGET).hex $(TARGET).bin $(TARGET).map $(TARGET).dmp  $(EXECUTABLE)
