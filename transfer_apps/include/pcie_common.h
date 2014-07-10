#ifndef __PCIE_COMMON__
#define __PCIE_COMMON__


#define PAGE_SIZE         4096
#define ALIGNED(size, alignment) (!(size & (alignment-1)))

#define CMD_LEN_MAX       128
#define CMD_OPEN_LEN_MAX  120

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
  char  name[CMD_OPEN_LEN_MAX];  // name of the file to be opened
} pcie_cmd_open_t;

typedef struct pcie_cmd_close_s {
  int   pcie_chan_handler;    // pcie channel handler
} pcie_cmd_close_t;

typedef struct pcie_cmd_lseek_s {
  int   pcie_chan_handler;    // pcie channel handler
  int   offset;               // offset to set
} pcie_cmd_lseek_t;

// typedef struct pcie_cmd_read_s {
//   int   pcie_chan_handler;        // pcie channel handler
//   int   rd_len;                   // number of bytes to read
//   // char *buf;                   // pointer to the data buffer
// } pcie_cmd_read_t;

typedef struct pcie_cmd_rdwr_s {
  int   pcie_chan_handler;        // pcie channel handler
  int   len;                      // number of bytes to read or write
  // char *buf;                   // pointer to the data buffer
} pcie_cmd_rdwr_t;

typedef struct pcie_cmd_s {
  void* ep_ptr;     // filled by RC according to EP channel used
  int   command;    // command type
  int   len;        // length of data, size of the command structure
                    //  ie. sizeof(pcie_cmd_write_t)
  char  data[CMD_LEN_MAX];       // buffer with specific command structure,
                    //  ie. (pcie_cmd_write_t*)
} pcie_cmd_t;

typedef struct pcie_response_s {
  int   pcie_chan_handler;    // pcie channel handler
  int   command;              // to which command response is dedicated
  int   result;               // command result:
                              //     negative - error,
                              //     0 - ok,
                              //     positive - number if written or read bytes, offset set
} pcie_response_t;



#endif /* __PCIE_COMMON__ */
