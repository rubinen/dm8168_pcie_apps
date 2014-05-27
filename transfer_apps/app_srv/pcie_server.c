
#include <stdio.h>
#include <pthread.h>
#include <pcie_server_lib.h>


#define APP_NAME     "pcie_sever"

#define dbg(level, debug, fmt, arg...)                \
  do {                                                \
    if (debug >= (level))                             \
      printf(APP_NAME": " fmt , ## arg); \
  } while (0)

static int debug = 0;


int main(int argc, char *argv[])
{
  int rv = 0;
  dbg(0, debug, "welcome!\n");

  rv = pcie_server_init();
  if (rv == 0)
  {
#if 0
    pthread_t newThread;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

    while (1)
    {
      /* wait for*/
      int newsockfd = accept(sockfd, (struct sockaddr*)&cli_addr, &cilen);
      noOfChilds++;

      struct fileWriteArgs args;
      args.sockfd = newsockfd;
      args.childNumber = noOfChilds;

      if(newsockfd < 0)
          errorOnAccept();

      pthread_create(&newThread, &attr, &fileWrite, &args);
    }
#endif
  }
  else
  {
    dbg(0, debug, "pcie_server_init returned error rv:%d!\n", rv);
  }


  dbg(0, debug, "bye!\n");
}

