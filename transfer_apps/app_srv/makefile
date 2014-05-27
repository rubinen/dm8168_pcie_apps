INC=../include
INCLIB=../lib
INTCFLAGS =-I$(INC)

OBJS_SERVER=pcie_server.o
LIBS=$(INCLIB)/lib_pcie_server.a

pcie_server.o: pcie_server.c
	$(CC)gcc $(INTCFLAGS) $(CFLAGS) -c $< 

server: $(OBJS_SERVER) $(LIBS)
	$(CC)gcc $(INTCFLAGS) $(CFLAGS) -o pcie_server $(OBJS_SERVER) -L. $(LIBS) -lpthread 


$(OBJS_CLIENT): $(INC)/pcie_server_lib.h

clean:
	-rm -f *.o pcie_server
