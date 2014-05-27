INC=../int_include
OBJS=ti81xx_mgmt_lib.o ti81xx_ep_lib.o ti81xx_trans.o ti81xx_pci_info.o

INTCFLAGS=-I$(INC) 

.c.o:
	$(CC)gcc -c $<  $(INTCFLAGS) $(CFLAGS) 

objects: $(OBJS)


$(OBJS): $(INC)/ti81xx_mgmt_lib.h $(INC)/ti81xx_ep_lib.h $(INC)/ti81xx_trans.h $(INC)/ti81xx_pci_info.h $(INC)/debug_msg.h

clean:
	-rm -f *.o *.a

