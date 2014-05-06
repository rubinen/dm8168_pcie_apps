MODE := DISPLAY
CFLAGS = -D$(MODE) -I $(KERNSRC)

# Uncomment to turn verbose mode on
#CFLAGS += -DDEBUG

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
