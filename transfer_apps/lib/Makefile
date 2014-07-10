INTINC=./int_include
INTLIB=./int_lib
INTCFLAGS =-I$(INTINC) -I../include

INT_OBJS_CLIENT=$(INTLIB)/ti81xx_mgmt_lib.o $(INTLIB)/ti81xx_trans.o $(INTLIB)/ti81xx_ep_lib.o
OBJ_CLIENT=lib_pcie_client.o
LIB_CLIENT=lib_pcie_client.a
LIB_CLIENT_SO=lib_pcie_client.so


OBJ_SERVER=lib_pcie_server.o
INT_OBJS_SERVER=$(INTLIB)/ti81xx_mgmt_lib.o $(INTLIB)/ti81xx_trans.o $(INTLIB)/ti81xx_pci_info.o
LIB_SERVER=lib_pcie_server.a
LIB_SERVER_SO=lib_pcie_server.so

#int_lib:
#	@make -C int_lib "CC=$(CROSS_COMPILE)" "CFLAGS=$(CFLAGS)"

lib_pcie_client.o: lib_pcie_client.c
	$(CC)gcc $(INTCFLAGS) $(CFLAGS) -c lib_pcie_client.c

lib_pcie_server.o: lib_pcie_server.c
	$(CC)gcc $(INTCFLAGS) $(CFLAGS) -c lib_pcie_server.c

static_libs: $(OBJ_CLIENT) $(OBJ_SERVER)
	@make -C int_lib "CC=$(CC)" "CFLAGS=$(CFLAGS)" objects
	$(CC)ar cr $(LIB_CLIENT) $(OBJ_CLIENT) $(INT_OBJS_CLIENT)
	$(CC)ar cr $(LIB_SERVER) $(OBJ_SERVER) $(INT_OBJS_SERVER)

shared_libs: $(OBJ_CLIENT) $(OBJ_SERVER)
	@make -C int_lib "CC=$(CC)" "CFLAGS=$(CFLAGS)" objects
	$(CC)gcc -shared -o $(LIB_CLIENT_SO) $(OBJ_CLIENT) $(INT_OBJS_CLIENT)
	$(CC)gcc -shared -o $(LIB_SERVER_SO) $(OBJ_SERVER) $(INT_OBJS_SERVER)

$(OBJS_CLIENT): $(INTINC)/ti81xx_mgmt_lib.h $(INTINC)/ti81xx_ep_lib.h $(INTINC)/ti81xx_trans.h $(INTINC)/debug_msg.h
$(OBJS_SERVER): $(INTINC)/ti81xx_mgmt_lib.h $(INTINC)/ti81xx_pci_info.h $(INTINC)/ti81xx_trans.h $(INTINC)/debug_msg.h

clean:
	@make -C int_lib clean
	-rm -f *.o *.a *.so
