CROSS_COMPILE=arm-none-linux-gnueabi-

#KERNSRC=/media/work/sdk/pcie_sdk_rc/kernel/linux-2.6.37-psp04.04.00.01
KERNSRC=

CFLAGS = -I $(KERNSRC)

CFLAGS += -DINTERRUPT_CAPABILITY

# Uncomment to turn verbose mode on
CFLAGS += -DDISPLAY
#CFLAGS += -DDEBUG
# Uncomment to perform integrity test
#CFLAGS += -DINTEGRITY
# Uncomment to perform throughput test
#CFLAGS += -DTHPT

all: rc ep

rc \
ep:
ifeq ($(KERNSRC),)
	$(error Pass full path to kernel source directory for RC or EP as KERNSRC=<full-path>)
endif

	@make -C lib "CC=$(CROSS_COMPILE)" "CFLAGS=$(CFLAGS)"
	@make -C application "CC=$(CROSS_COMPILE)" "CFLAGS=$(CFLAGS)" $@

clean:
	@make -C lib clean 
	@make -C application clean
