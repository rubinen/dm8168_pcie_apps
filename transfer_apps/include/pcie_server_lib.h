#ifndef __PCIE_SERVER_LIB__
#define __PCIE_SERVER_LIB__

#include <pcie_common.h>

int pcie_server_init(void);
/*****************************************\
  Wait for incoming requests:
  ie.
  - open file ()
  - close file
\*****************************************/
int pcie_server_listen(void);

/*****************************************\
  Respond to incoming requests:
  ie.
  - open file status
  - close file status
\*****************************************/
int pcie_server_respond(void);

/*****************************************\
  Receive data from client
\*****************************************/
int pcie_server_receive(void *buf, int size);

/*****************************************\
  Send data to client
\*****************************************/
int pcie_server_send(void *buf, int size);

#endif /* __PCIE_SERVER_LIB__ */
