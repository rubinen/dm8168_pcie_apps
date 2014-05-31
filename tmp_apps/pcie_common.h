#ifndef __PCIE_COMMON__
#define __PCIE_COMMON__

#define PCIE_CMD_DATA_LEN 128

enum {
  PCIE_CMD_OPEN = 0,
  PCIE_CMD_CLOSE,
  PCIE_CMD_READ,
  PCIE_CMD_WRITE,
  PCIE_CMD_LSEEK,
  PCIE_CMD_MAX
};

enum {
  PCIE_MODE_READ = 0,
  PCIE_MODE_WRITE,
  PCIE_MODE_APPEND
};

typedef struct pcie_cmd_open_s {
  int   mode;       // mode of the opened file
  char  name[128];  // name of the file to be opened
} pcie_cmd_open_t;

typedef struct pcie_cmd_close_s {
  int   handler;    // handler of the file to be closed
} pcie_cmd_close_t;

typedef struct pcie_cmd_lseek_s {
  int   handler;    // handler of the file
  int   offset;     // offset to set
} pcie_cmd_lseek_t;

typedef struct pcie_cmd_read_s {
  int   handler;    // handler of the file to read
  int   rd_len;        // number of bytes to read
  // char *buf;        // pointer to the data buffer
} pcie_cmd_read_t;

typedef struct pcie_cmd_write_s {
  int   handler;    // handle of the file to write
  int   wr_len;        // number of bytes to write
  // char *buf;        // pointer to the data buffer
} pcie_cmd_write_t;


typedef struct pcie_cmd_s {
  int   command;    // command type
  int   len;        // length of data, size of the command structure
                    //  ie. sizeof(pcie_cmd_write_t)
  char  data[PCIE_CMD_DATA_LEN];       // pointer to specific command structure,
                    //  ie. (pcie_cmd_write_t*)
} pcie_cmd_t;


typedef struct pcie_response_s {
  int   command;    // to which command response is dedicated
  int   handler;    // handler of the file opened
  int   result;     // command result:
                    //     negative - error,
                    //     0 - ok,
                    //     positive - number if written or read bytes, offset set
} pcie_response_t;



#endif /* __PCIE_COMMON__ */
