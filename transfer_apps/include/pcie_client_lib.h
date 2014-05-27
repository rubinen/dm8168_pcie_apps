#ifndef __PCIE_CLIENT_LIB__
#define __PCIE_CLIENT_LIB__

#include <pcie_common.h>

int pcie_client_init(void);
int pcie_client_open(const char *name, int flags);
int pcie_client_read(int fd, void *buf, int size);
int pcie_client_write(int fd, void *buf, int size);
int pcie_client_close(int fd);

#endif /* __PCIE_CLIENT_LIB__ */
