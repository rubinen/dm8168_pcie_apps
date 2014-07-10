#ifndef __PCIE_SERVER_LIB__
#define __PCIE_SERVER_LIB__

#include <pcie_common.h>

int PCIE_SRV_Init(void);
/*****************************************\
  Wait for command:
  ie.
  - open file ()
  - close file
\*****************************************/
int PCIE_SRV_CmdWait(pcie_cmd_t *cmd);

/*****************************************\
  Send response to command:
  ie.
  - open file status
  - close file status
  - number of bytes written
\*****************************************/
int PCIE_SRV_CmdResponse(void *ep_ptr, pcie_response_t *rsp);

/*****************************************\
  Receive data from client
\*****************************************/
int PCIE_SRV_Receive(void* ep_ptr, void *buf, int size);

/*****************************************\
  Send data to client
\*****************************************/
int PCIE_SRV_Send(void *ep_ptr, void *buf, int size);

#endif /* __PCIE_SERVER_LIB__ */
