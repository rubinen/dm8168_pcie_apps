#ifndef __PCIE_SERVER_LIB__
#define __PCIE_SERVER_LIB__

#include <pcie_common.h>

int pcie_srv_init(void);
/*****************************************\
  Wait for command:
  ie.
  - open file ()
  - close file
\*****************************************/
int pcie_srv_cmd_wait(pcie_cmd_t *cmd);

/*****************************************\
  Send response to command:
  ie.
  - open file status
  - close file status
  - number of bytes written
\*****************************************/
int pcie_srv_cmd_response(pcie_response_t *rsp);

/*****************************************\
  Receive data from client
\*****************************************/
int pcie_server_receive(int handler, void *buf, int size);

/*****************************************\
  Send data to client
\*****************************************/
int pcie_server_send(int handler, void *buf, int size);

#endif /* __PCIE_SERVER_LIB__ */
