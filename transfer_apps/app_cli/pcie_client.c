#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <pcie_client_lib.h>

extern int debug_test;
extern int bar_chosen;
int ep_id = 22;
extern int sleep_dur;
char name[CMD_OPEN_LEN_MAX] = {0,};

static int parse_opts(int argc, char **argv)
{
  int index;
  int c;

  opterr = 0;

  while ((c = getopt (argc, argv, "ve:b:n:s:")) != -1)
  {
    switch (c)
    {
      case 'v':
        debug_test = 1;
        break;
      case 'e':
        ep_id = atoi(optarg);
        break;
      case 'b':
        bar_chosen = atoi(optarg);
        break;
      case 'n':
        strncpy(name, optarg, strlen(optarg));
        break;
      case 's':
        sleep_dur = atoi(optarg);
        break;
      case '?':
        if (optopt == 'c')
        {
          fprintf (stderr, "Option -%c requires an argument.\n", optopt);
        }
        else if (isprint (optopt))
        {
          fprintf (stderr, "Unknown option `-%c'.\n", optopt);
        }
        else
        {
          fprintf (stderr,
          "Unknown option character `\\x%x'.\n",
          optopt);
        }
        return 1;
      default:
        abort ();
    }
  }

  for (index = optind; index < argc; index++)
  {
    printf ("Non-option argument %s\n", argv[index]);
  }
  return 0;
}

int main(int argc, char *argv[])
{
  int rv = 0;
  parse_opts(argc, argv);

  printf ("EP id:%d\n", ep_id);
  printf ("file name: %s\n", name);
  rv = pcie_cli_init(ep_id);
  if (rv == 0)
  {
    printf ("sleep %d\n", sleep_dur);
    sleep(sleep_dur);
    int fd = pcie_cli_open(name, O_RDWR);
    printf ("file open rv: %d\n", fd);

    rv = pcie_cli_close(fd);
    printf ("file close rv: %d\n", rv);
  }

// int pcie_cli_init(int ep_id);
// int pcie_cli_open(const char *name, int flags);
// int pcie_cli_read(int fd, void *buf, int size);
// int pcie_cli_write(int fd, void *buf, int size);
// int pcie_cli_close(int fd);

  return 0;
}
