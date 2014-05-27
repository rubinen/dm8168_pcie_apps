INC=../include
INCLIB=../lib
INTCFLAGS =-I$(INC)

OBJS_CLIENT=pcie_client.o
LIBS=$(INCLIB)/lib_pcie_client.a

pcie_client.o: pcie_client.c
	$(CC)gcc $(INTCFLAGS) $(CFLAGS) -c $< 

client: $(OBJS_CLIENT) $(LIBS)
	$(CC)gcc $(INTCFLAGS) $(CFLAGS) -o pcie_client $(OBJS_CLIENT) -L. $(LIBS) -lpthread 


$(OBJS_CLIENT): $(INC)/pcie_client_lib.h

clean:
	-rm -f *.o pcie_client
