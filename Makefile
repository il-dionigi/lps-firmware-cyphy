OPENOCD           ?= openocd
OPENOCD_INTERFACE ?= interface/stlink-v2.cfg
REV               ?= B
PYTHON2           ?= python2
# CFLAGS          += -fdiagnostics-color=auto
# CFLAGS += -DUSE_FTDI_UART

BOOTLOAD          ?= 0

ifeq ($(strip $(REV)),A)
$(error Rev.A not supported anymore)
else ifeq ($(strip $(REV)),B)
HAL_ROOT=hal/stm32f0xx
CPU=f0
PROCESSOR=-mthumb -mcpu=cortex-m0 -DHSI48_VALUE="((uint32_t)48000000)" -DSTM32F072xB
OPENOCD_TARGET    ?= target/stm32f0x_stlink.cfg
else
$(error Rev.$(REV) unknown)
endif

#WOLFSSL = src/lib/WolfSSL
WOLFSSLDEFINES = -DNO_WRITEV -DNO_FILESYSTEM -DNO_DEV_RANDOM -DWOLFSSL_USER_IO -DSINGLE_THREADED -DNO_INLINE

WOLFSSLDEFINES += -DNO_WOLFSSL_CLIENT -DNO_WOLFSSL_SERVER -DNO_DES3 -DNO_DSA -DNO_HMAC -DNO_MD4
WOLFSSLDEFINES += -DNO_MD5 -DNO_PWDBASED -DNO_RC4 -DNO_SESSION_CACHE
WOLFSSLDEFINES += -DNO_TLS -DNOWC_NO_RSA_OAEP -DNO_OLD_TLS
WOLFSSLDEFINES += -DNO_ERROR_STRINGS -DNO_WOLFSSL_MEMORY -DNO_DH -DNO_CODING
WOLFSSLDEFINES += -DNO_HC128 -DNO_SHA -DNO_RABBIT -DWOLFCRYPT_ONLY


WOLFSSLDEFINES += -DHAVE_AESGCM -DHAVE_ECC -DRSA_LOW_MEM -DUSE_FAST_MATH

LIB = src/lib

# WolfSSL
VPATH += $(LIB)/WolfSSL/wolfssl
VPATH += $(LIB)/WolfSSL/wolfssl/wolfcrypt
VPATH += $(LIB)/WolfSSL/wolfcrypt/src
VPATH += $(LIB)/WolfSSL/src
#VPATH += $(LIB)/WolfSSL

INCLUDES=-Iinc -Iinc/$(CPU) -I$(HAL_ROOT)/Inc -IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc

# FreeRTOS
FREERTOS_OBJS=list queue timers tasks port event_groups
OBJS+=$(foreach mod, $(FREERTOS_OBJS), lib/freertos/src/$(mod).o)
INCLUDES+=-Ilib/freertos/inc

# Platform specific files
OBJS+=src/f0/startup_stm32f072xb.o src/f0/system_stm32f0xx.o src/f0/stm32f0xx_it.o src/f0/stm32f0xx_hal_msp.o
OBJS+=src/f0/gpio.o src/f0/i2c.o src/f0/spi.o src/f0/system.o src/f0/usart.o
OBJS+=src/f0/usbd_conf.o src/eeprom.o src/bootmode.o
HALS+=i2c_ex

OBJS+=src/main.o
OBJS+=src/usb_device.o src/usbd_cdc_if.o src/usbd_desc.o src/lps25h.o src/led.o src/button.o
OBJS+=src/cfg.o src/usbcomm.o src/test_support.o src/production_test.o
OBJS+=src/uwb.o src/uwb_twr_anchor.o src/uwb_sniffer.o src/uwb_twr_tag.o
OBJS+=src/lpp.o src/uwb_tdoa_anchor2.o src/uwb_tdoa_anchor3.o

# Wolfssl 
#PROJ_OBJ += aes.o ssl.o #internal.o error-ssl.o coding.o dirent.o stat.o asn.o dh.o
#PROJ_OBJ += -l$(WOLFSSL)/src #-l$(WOLFSSL)/wolfssl -l$(WOLFSSL)/wolfssl/wolfcrypt
#PROJ_OBJ += -l$(WOLFSSL)/wolfcrypt/src
# removed ones
# crl.o internal.o io.o keys.o ocsp.o sniffer.o ssl.o tls.o
PROJ_OBJ += ssl.o aes.o #WolfSSL/src
PROJ_OBJ += arc4.o asm.o asn.o async.o blake2b.o camellia.o chacha.o
PROJ_OBJ += chacha20_poly1305.o cmac.o coding.o compress.o curve25519.o
PROJ_OBJ += des3.o dh.o dsa.o ecc_fp.o ecc.o ed25519.o error.o fe_low_mem.o fe_operations.o
PROJ_OBJ += ge_low_mem.o ge_operations.o hash.o hc128.o hmac.o idea.o 

PROJ_OBJ += crl.o

PROJ_OBJ += logging.o md2.o
PROJ_OBJ += md4.o md5.o memory.o misc.o pkcs12.o pkcs7.o poly1305.o pwdbased.o
PROJ_OBJ += rabbit.o random.o ripemd.o rsa.o sha.o sha256.o sha512.o signature.o srp.o
PROJ_OBJ += tfm.o wc_encrypt.o wc_port.o wolfevent.o

PROJ_OBJ += io.o ocsp.o sniffer.o tls.o keys.o internal.o integer.o

HALS+=gpio rcc cortex i2c pcd dma pcd_ex rcc_ex spi uart pwr
OBJS+=$(foreach mod, $(HALS), $(HAL_ROOT)/Src/stm32$(CPU)xx_hal_$(mod).o)
OBJS+=$(HAL_ROOT)/Src/stm32$(CPU)xx_hal.o

USB_CORES=core ctlreq ioreq
USB_CDC=cdc
OBJS+=$(foreach mod, $(USB_CORES), Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_$(mod).o)
OBJS+=$(foreach mod, $(USB_CDC), Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_$(mod).o)

#libdw1000
INCLUDES+=-Ivendor/libdw1000/inc
INCLUDES+= -I$(LIB)/WolfSSL
INCLUDES+= -I$(LIB)/WolfSSL/src
INCLUDES+= -I$(LIB)/WolfSSL/wolfssl/
INCLUDES+= -I$(LIB)/WolfSSL/wolfssl/wolfcrypt
INCLUDES+= -I$(LIB)/WolfSSL/wolfcrypt/src
OBJS+=vendor/libdw1000/src/libdw1000.o vendor/libdw1000/src/libdw1000Spi.o

OBJS+=src/dwOps.o

CFLAGS+=$(PROCESSOR) $(INCLUDES) $(WOLFSSLDEFINES) -O3 -g3 -Wall -Wno-pointer-sign -std=gnu11
LDFLAGS+=$(PROCESSOR) --specs=nano.specs --specs=nosys.specs -lm -lc -u _printf_float

ifeq ($(strip $(BOOTLOAD)),0)
LDFLAGS+=-Ttools/make/stm32f072.ld
LOAD_ADDRESS = 0x8000000
else
LDFLAGS+=-Ttools/make/stm32f072_bootload.ld
LOAD_ADDRESS = 0x8005000
endif

# Remove un-used functions and global variables from output file
CFLAGS += -ffunction-sections -fdata-sections
LDFLAGS+=-Wl,-Map=bin/$(PROG).map,--cref,--gc-sections


PREFIX=arm-none-eabi-

CC=$(PREFIX)gcc
LD=$(PREFIX)gcc
AS=$(PREFIX)as
OBJCOPY=$(PREFIX)objcopy
SIZE=$(PREFIX)size

all: check_submodules bin/lps-node-firmware.elf bin/lps-node-firmware.dfu

bin/lps-node-firmware.elf: $(OBJS)
	$(LD) -o $@ $^ $(LDFLAGS)
	$(SIZE) $@
	@echo BOOTLOADER Support: $(BOOTLOAD)

clean:
	rm -f bin/lps-node-firmware.elf bin/lps-node-firmware.dfu bin/.map $(OBJS)

flash:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) -f $(OPENOCD_TARGET) -c init -c targets -c "reset halt" \
	           -c "flash write_image erase bin/lps-node-firmware.elf" -c "verify_image bin/lps-node-firmware.elf" -c "reset run" -c shutdown
erase:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) -f $(OPENOCD_TARGET) -c init -c targets -c "reset halt" \
	           -c "stm32f1x mass_erase 0" -c shutdown

openocd:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) -f $(OPENOCD_TARGET) -c init -c targets

dfu:
	dfu-util -d 0483:df11 -a 0 -D bin/lps-node-firmware.dfu -s :leave

reset_and_dfu:
	tools/make/reset-to-dfu.py
	dfu-util -d 0483:df11 -a 0 -D bin/lps-node-firmware.dfu -s :leave

# Generic rules
%.bin: %.elf
	$(OBJCOPY) $^ -O binary $@

%.dfu: %.bin
	$(PYTHON2) tools/make/dfu-convert.py -b $(LOAD_ADDRESS):$^ $@

check_submodules:
	$(PYTHON2) tools/make/check-for-submodules.py
