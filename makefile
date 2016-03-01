

# setup



#ifndef __PROJECT_ROUTER_CONF_H__
#define __PROJECT_ROUTER_CONF_H__
__PROJECT_ROUTER_CONF_H__=1

UIP_FALLBACK_INTERFACE=rpl_interface
CFLAGS+= -D__PROJECT_ROUTER_CONF_H__ -DUIP_FALLBACK_INTERFACE=rpl_interface
#ifndef UIP_FALLBACK_INTERFACE
#define UIP_FALLBACK_INTERFACE rpl_interface
#endif

#ifndef QUEUEBUF_CONF_NUM
#define QUEUEBUF_CONF_NUM          4
#endif

#ifndef UIP_CONF_BUFFER_SIZE
#define UIP_CONF_BUFFER_SIZE    140
#endif

#ifndef UIP_CONF_RECEIVE_WINDOW
#define UIP_CONF_RECEIVE_WINDOW  60
#endif

#ifndef WEBSERVER_CONF_CFS_CONNS
#define WEBSERVER_CONF_CFS_CONNS 2
#endif

#endif /* __PROJECT_ROUTER_CONF_H__ */


WITH_UIP6=1
UIP_CONF_IPV6=1
CFLAGS+= -DUIP_CONF_IPV6_RPL

#linker optimizations
SMALL=1

CFLAGS += -DPROJECT_CONF_H=\"contiki-2.6/examples/ipv6/rpl-border-router/project-conf.h\"
PROJECT_SOURCEFILES += contiki-2.6/examples/ipv6/rpl-border-router/slip-bridge.c
PROJECT_SOURCEFILES += contiki-2.6/examples/ipv6/rpl-border-router/httpd-simple.c
CFLAGS += -DWEBSERVER=1

WITH_WEBSERVER=1

ifeq ($(PREFIX),)
 PREFIX = aaaa::1/64
endif

#WITH_UIP=1
WITH_UIP6=1
UIP_CONF_IPV6=1
ifdef UIP_CONF_IPV6
CFLAGS += -DWITH_UIP6=1 -DUIP_CONF_IPV6=1
endif

CFLAGS+= -DUIP_CONF_IPV6_RPL=1
#UIP_UDP=1
UIP_CONF_IPV6=1
COMPILE_OPTS = -mcpu=cortex-m0 -mthumb -Wall -g -O0 -c -DINCLUDE_PA -DUSE_STDPERIPH_DRIVER -D__ASSEMBLY__ -DSTM32F103RC -fno-builtin-printf -DSTM32F10X_MD -DMCK=72000000 -DAUTOSTART_ENABLE
INCLUDE_DIRS = -I . -I stm_lib/inc -I cmsis_boot -I cmsis -I contiki-2.6/platform/stm32test -I contiki-2.6/platform/stm32test/dev
INCLUDE_DIRS += -I contiki-2.6/core/lib -I contiki-2.6/examples/hello-world -I syscalls
INCLUDE_DIRS += -I stm_contiki -Icontiki-2.6/core/lib -Icontiki-2.6/core/net -Icontiki-2.6/core/net/mac -Icontiki-2.6/core/net/rime -Icontiki-2.6/core/net/rpl
INCLUDE_DIRS += -Icontiki-2.6/core/sys -Icontiki-2.6/core/cfs -Icontiki-2.6/core/ctk -Icontiki-2.6/core/lib/ctk -Icontiki-2.6/core/loader -Icontiki-2.6/core/
INCLUDE_DIRS += -Icontiki-2.6/apps/shell -Icontiki-2.6/apps/telnetd -Icontiki-2.6/apps/webbrowser -Icontiki-2.6/apps/webserver -Icontiki-2.6/apps/irc -Icontiki-2.6/apps/telnet -Icontiki-2.6/apps/twitter -Icontiki-2.6/apps/collect-view -Icontiki-2.6/core/dev
DEFINES=UIP_CONF_IPV6=1 WITH_UIP6=1 WITH_UIP=1 UIP_CONF_IPV6_RPL=1
#DEFINES=WITH_UIP=1
UIP_CONF_IPV6_RPL=1

-include contiki-2.6/Makefile.include

#INCLUDE_DIRS += ${addprefix -I contiki-2.6/core/,dev lib net net/mac net/rime \
#                 net/rpl sys cfs ctk lib/ctk loader . }
#INCLUDE_DIRS += ${addprefix -I contiki-2.6/core/,$(SOURCEDIRS)}
#INCLUDE_DIRS += ${addprefix -I ,$(SOURCEDIRS)}

LIBRARY_DIRS = -L stm_lib

CC = arm-none-eabi-gcc
CFLAGS += $(COMPILE_OPTS) $(INCLUDE_DIRS)

CXX = arm-none-eabi-g++
CXXFLAGS = $(COMPILE_OPTS) $(INCLUDE_DIRS)

AS = arm-none-eabi-gcc
ASFLAGS = $(COMPILE_OPTS) -c

LD = arm-none-eabi-gcc
LDFLAGS = -mcpu=cortex-m0 -mthumb -nostartfiles -Wl,--gc-sections,-Map=$@.map,-cref,-u,Reset_Handler $(INCLUDE_DIRS) $(LIBRARY_DIRS) -T link.ld
#LDFLAGS = -nostartfiles -Wl,-Map=$@.map,-u,Reset_Handler -mcpu=cortex-m3 -mthumb -Wl,--gc-sections $(INCLUDE_DIRS) $(LIBRARY_DIRS) -T link.ld

OBJCP = arm-none-eabi-objcopy
OBJCPFLAGS = -O binary

AR = arm-none-eabi-ar
ARFLAGS = cr

MAIN_OUT = contiki-main
MAIN_OUT_ELF = $(MAIN_OUT).elf
MAIN_OUT_BIN = $(MAIN_OUT).bin

# all

all: $(MAIN_OUT_ELF) $(MAIN_OUT_BIN)

# main

$(MAIN_OUT_ELF): contiki-2.6/platform/stm32test/contiki-main.o stm_lib/libstm32.a stm_lib/libcontiki.a
	$(LD) $(LDFLAGS) contiki-2.6/platform/stm32test/contiki-main.o stm_lib/libstm32.a stm_lib/libcontiki.a \
	   syscalls/syscalls.o  --output $@

$(MAIN_OUT_BIN): $(MAIN_OUT_ELF)
	$(OBJCP) $(OBJCPFLAGS) $< $@


# flash

flash: $(MAIN_OUT)
	@cp $(MAIN_OUT_ELF) jtag/flash
	@cd jtag; openocd -f flash.cfg
	@rm jtag/flash


# libstm32.a
LIBSTM32_OUT = stm_lib/libstm32.a
# libcontiki.a
LIBCONTIKI_OUT = stm_lib/libcontiki.a

LIBCONTIKI_OBJS = \
 $(CONTIKI_OBJECTFILES)
LIBCONTIKI_OBJS += contiki-2.6/examples/$(EXP)/$(EX).o stm_contiki/clock.o stm_contiki/slip_uart1.o contiki-2.6/platform/stm32test/dev/button-sensor.o contiki-2.6/core/lib/sensors.o contiki-2.6/core/net/mac/uzmac.o  contiki-2.6/core/dev/slip.o contiki-2.6/examples/ipv6/rpl-border-router/httpd-simple.o contiki-2.6/examples/ipv6/rpl-border-router/slip-bridge.o

LIBSTM32_OBJS = \
 stm_lib/src/stm32f10x_adc.o \
 stm_lib/src/stm32f10x_exti.o \
 stm_lib/src/stm32f10x_flash.o \
 stm_lib/src/stm32f10x_gpio.o \
 stm_lib/src/stm32f10x_i2c.o \
 stm_lib/src/stm32f10x_pwr.o \
 stm_lib/src/stm32f10x_rcc.o \
 stm_lib/src/stm32f10x_spi.o \
 stm_lib/src/stm32f10x_tim.o \
 stm_lib/src/misc.o \
 uz2400/uz2400.o \
 uz2400/uz_isr.o \
 uz2400/uz_spif.o \
 uz2400/uz_srf.o \
 uz2400/rf_package.o \
 syscalls/syscalls.o \
 stdio/printf.o \
 sensor/mma7660.o \
 cmsis_boot/startup/startup_stm32f10x_md.o \
 cmsis_boot/system_stm32f10x.o \
 cmsis/core_cm3.o \
 ntu32_config.o \

$(LIBSTM32_OUT): $(LIBSTM32_OBJS)
	$(AR) $(ARFLAGS) $@ $(LIBSTM32_OBJS)
	
$(LIBCONTIKI_OUT): $(LIBCONTIKI_OBJS)
	$(AR) $(ARFLAGS) $@ $(LIBCONTIKI_OBJS)

#$(LIBSTM32_OBJS): stm32f10x_conf.h


clean: 
	-rm syscalls/*.o uz2400/*.o cc2520/*.o *.o stm_lib/src/*.o $(LIBSTM32_OUT) $(LIBCONTIKI_OUT) $(MAIN_OUT_ELF) $(MAIN_OUT_BIN) stm_contiki/clock.o stm_contiki/slip_uart1.o contiki-2.6/examples/hello-world/hello-world.o contiki-2.6/platform/stm32test/dev/button-sensor.o  contiki-2.6/core/lib/sensors.o contiki-2.6/platform/stm32test/contiki-main.o contiki-2.6/core/dev/slip.o
	find . -name "*.o" -exec rm -rf {} \;