#ifndef __PCIE_CLIENT_LIB__
#define __PCIE_CLIENT_LIB__

#include <pcie_common.h>

int pcie_cli_init(void);
int pcie_cli_open(const char *name, int flags);
int pcie_cli_read(int fd, void *buf, int size);
int pcie_cli_write(int fd, void *buf, int size);
int pcie_cli_close(int fd);

#endif /* __PCIE_CLIENT_LIB__ */
