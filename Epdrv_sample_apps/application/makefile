INC=../include
INCLIB=../lib
INTCFLAGS =-I$(INC)

OBJS_EP=ti81xx_ep_app.o
OBJS_RC=ti81xx_rc_app.o
LIBS=$(INCLIB)/lib_ti81xx.a

ti81xx_ep_app.o: ti81xx_ep_app.c
	$(CC)gcc $(INTCFLAGS) $(CFLAGS) -c $< 

ti81xx_rc_app.o: ti81xx_rc_app.c
	$(CC)gcc $(INTCFLAGS) $(CFLAGS) -c ti81xx_rc_app.c

ep: $(OBJS_EP) $(LIBS)
	$(CC)gcc $(INTCFLAGS) $(CFLAGS) -o ep_app $(OBJS_EP) -L. $(LIBS) -lpthread 

rc: $(OBJS_RC) $(LIBS)
	$(CC)gcc $(INTCFLAGS) $(CFLAGS) -o rc_app $(OBJS_RC) -L. $(LIBS) -lpthread 

$(OBJS_EP): $(INC)/ti81xx_mgmt_lib.h $(INC)/ti81xx_ep_lib.h $(INC)/ti81xx_trans.h $(INC)/debug_msg.h
$(OBJS_RC): $(INC)/ti81xx_mgmt_lib.h $(INC)/ti81xx_pci_info.h $(INC)/ti81xx_trans.h $(INC)/debug_msg.h

clean:
	-rm -f *.o ep_app rc_app
