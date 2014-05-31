
#include <stdio.h>
#include <pthread.h>
#include <pcie_server_lib.h>


#define APP_NAME     "pcie_sever"

#define dbg(level, debug, fmt, arg...)                \
  do {                                                \
    if (debug >= (level))                             \
      printf(APP_NAME": " fmt , ## arg); \
  } while (0)


typedef struct pcie_srv_proc_s {
  int             handler;    // handle of the file to write
  int             mode;       //
  char           *filename;   // filename
  pid_t           pid;        // pid of the child process
  pthread_mutex_t mutex;      // mutex
} pcie_cmd_write_t;


static int debug = 0;



int main(int argc, char *argv[])
{
  int rv = 0;
  dbg(0, debug, "welcome!\n");

  rv = pcie_server_init();
  if (rv != 0)
  {
    dbg(0, debug, "pcie_server_init returned error rv:%d!\n", rv);
    exit(1);
  }

  while (1)
  {
    pcie_cmd_t cmd;
    rv = pcie_srv_cmd_wait();
    if (rv == -1) {
        dbg(0, debug, "pcie_server_wait returned error rv:%d!\n", rv);
        continue;
    }
    pcie_server_respond();

    inet_ntop(their_addr.ss_family,
        get_in_addr((struct sockaddr *)&their_addr),
        s, sizeof s);
    printf("server: got connection from %s\n", s);


  }
  else
  {
  }


  dbg(0, debug, "bye!\n");
}

