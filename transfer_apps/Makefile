CROSS_COMPILE=arm-none-linux-gnueabi-

KERNSRC=/media/work/mesurex/sdk/pcie_slave_sdk/kernel/linux-2.6.37-psp04.04.00.01
#KERNSRC=

CFLAGS = -I $(KERNSRC)

CFLAGS += -DINTERRUPT_CAPABILITY

# Uncomment to turn verbose mode on
#CFLAGS += -DDISPLAY
#CFLAGS += -DDEBUG
# Uncomment to perform integrity test
#CFLAGS += -DINTEGRITY
# Uncomment to perform throughput test
CFLAGS += -DTHPT

all: client server

client: static_libs
	@make -C app_cli "CC=$(CROSS_COMPILE)" "CFLAGS=$(CFLAGS)" $@

server: static_libs
	@make -C app_srv "CC=$(CROSS_COMPILE)" "CFLAGS=$(CFLAGS)" $@

static_libs:
ifeq ($(KERNSRC),)
	$(error Pass full path to kernel source directory for RC or EP as KERNSRC=<full-path>)
endif
	@make -C lib "CC=$(CROSS_COMPILE)" "CFLAGS=$(CFLAGS)" static_libs

shared_libs:
ifeq ($(KERNSRC),)
	$(error Pass full path to kernel source directory for RC or EP as KERNSRC=<full-path>)
endif
	@make -C lib "CC=$(CROSS_COMPILE)" "CFLAGS=$(CFLAGS)" shared_libs


clean:
	@make -C lib clean 
	@make -C app_cli clean
	@make -C app_srv clean
