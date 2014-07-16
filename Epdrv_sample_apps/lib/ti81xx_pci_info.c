#include "ti81xx_pci_info.h"
#include "ti81xx_mgmt_lib.h"
#include "ti81xx_trans.h"

/**
 * add_node_in_list(): Add information node in global list.
 * @pci_info: node to be added
 * @start: point to start pointer of list.
 */

int add_node_in_list(struct pci_sys_info *pci_info, struct pci_sys_info **start)
{
  if (start == NULL) {
    *start = pci_info;
  } else {
    pci_info->next = *start;
    *start = pci_info;
  }
  return 0;
}

/**
 * free_list()-- free memory of each node in list
 */
int free_list(struct pci_sys_info *start)
{
  for ( ; start != NULL; start = start->next)
    free(start);
  return 0;
}


/**
 * add_resource_in_list() : create a node , fill it with bars info and add it
 * to list using add_node_in_list
 * @fr: file pointer to resource file ( contain bar info)
 * @start: point to start pointer of list.
 */

int add_resource_in_list(FILE *fr, struct pci_sys_info **start)
{
  char resource_string[6][100];
  char *p;
  int i = 0;
  struct pci_sys_info *pci_info = malloc(sizeof(struct pci_sys_info));
  if (pci_info == NULL) {
    err_print("memory allocation for resource"
          " information failed\n");
    return -1;
  }
  for (i = 0; i <= 5; i++) {
    if (fgets(resource_string[i], 100, fr) == NULL) {
      err_print("not enough info in sysfs\n");
      free(pci_info);
      return -1;
    }
    resource_string[i][37] = 0;
    p = strchr(resource_string[i], 32);
    p[0] = 0;
    pci_info->res_value[i + 1][0] =
        strtoul(resource_string[i], NULL, 16);
    pci_info->res_value[i + 1][1] =
        strtoul(p + 1, NULL, 16) -
        pci_info->res_value[i + 1][0] + 1;
    if (strtoul(resource_string[i], NULL, 16) == 0 &&
        strtoul(p + 1, NULL, 16) == 0) {/*bars unused*/
      pci_info->res_value[i + 1][1] = 0;
    }
  }
  pci_info->next = NULL;
  add_node_in_list(pci_info, start);
  return 0;
}


/**
 * fetch_my_unique_id(): fetch my unique id from management area of EP.
 * @mgmt_area: Mmaped return pointer to BAR2(default inbound) of destined EP.
 * @node: Node in list, that represents this EP mgmt area's info.
 */

int fetch_my_unique_id(unsigned int *mgmt_area, struct pci_sys_info *node)
{
  debug_print("mgmt_area [0]:%d [1]:%d [2]:%d\n",
      mgmt_area[0], mgmt_area[1], mgmt_area[2]);

  node->res_value[0][0] = mgmt_area[1];
  node->res_value[0][1] = 2; /*by default BAR 2 mapping is assumed.*/
  return 0;
}


/**
 * print_list() : print entire list of nodes containing EP bars info.
 * @start: start of list
 */

int print_list(struct pci_sys_info *start)
{
  struct pci_sys_info *tmp = start;
  int i;
  for (tmp = start; tmp != NULL; tmp = tmp->next) {
    debug_print("unique id is --%u  BAR mapping is %u\n",
        tmp->res_value[0][0], tmp->res_value[0][1]);
    for (i = 1; i <= 6; i++)
      debug_print("bar address is %x bar size is %x\n",
        tmp->res_value[i][0], tmp->res_value[i][1]);
  }
  return 0;
}


/**
 * dump_info_on_ep(): put all ep's info on destined EP.
 * @start: start of list
 * @mgmt_area: mmaped return address of destined EP's bar 2.
 * @eps: no of eps in system returned by get_devices().
 */

int dump_info_on_ep(struct pci_sys_info *start, unsigned int *mgmt_area,
        unsigned int eps, unsigned int startaddr)
{
  char *temp = (char *)mgmt_area;
  void *index;
  unsigned int size_mgmt = 0;

  debug_print("mgmt_area:%08x\n", mgmt_area);

ACCESS_MGMT:
  if (access_mgmt_area(mgmt_area, 1) == 0) {
    /*1 will be unique id of RC always.*/
    size_mgmt = mgmt_area[4];
    debug_print("size of management area on rmt EP is %u\n",
                size_mgmt);
    *((unsigned int *) (temp + size_mgmt + sizeof(unsigned int))) =
                startaddr;
    *((unsigned int *) (temp + size_mgmt)) =
            eps;/*no of ep in system*/
    size_mgmt += 2 * sizeof(unsigned int);
    index = temp + size_mgmt;
    for (; start != NULL; start = start->next) {
      memcpy(index, temp,
        sizeof(struct pci_sys_info) - sizeof(void *));
      index += sizeof(struct pci_sys_info) - sizeof(void *);
      size_mgmt +=
        sizeof(struct pci_sys_info) - sizeof(void *);
    }
    mgmt_area[4] = size_mgmt ;/*update new size on EP.*/
    debug_print("size of management area is updated to %u\n",
                mgmt_area[4]);
    release_mgmt_area(mgmt_area);
  } else
    goto ACCESS_MGMT;
  return 0;
}


/**
 * get_devices(): get all devices of particular vendor and device id in system
 */


int get_devices(struct pci_sys_info **start)
{
  char  file_vendor[50] = {0};
  char  file_device[50] = {0};
  char  file_resource[50] = {0};
  char  file_class[50] = {0};
  char  list_devices[25];
  char  cmd[50] = "ls /sys/bus/pci/devices > ";
  char  file_med[14];
  char  top_dir[25] = "/sys/bus/pci/devices/";
  char  vendor_id[7];
  char  device_id[7];
  char  class[11];
  int   eps = 0;
  unsigned int class_code;
  unsigned short vendor;
  int random;
  unsigned short device;
  FILE *fp, *fv, *fd, *fr, *fc;

  random = rand();
  sprintf(list_devices, "%x%c%c%s", random, '0', 'X', ".pcidevlist");
  strcat(cmd, list_devices);
  system(cmd);
  fp = fopen(list_devices, "r");
  if (fp == NULL)
    return -1;

  while (1) {
    if (fgets(file_med, 14, fp) == NULL) {
      debug_print("no more pci devices in sysfs\n");
      fclose(fp);
      return eps;
    }
    file_med[12] = 0;
    strcpy(file_vendor, top_dir);
    strcpy(file_device, top_dir);
    strcpy(file_resource, top_dir);
    strcpy(file_class, top_dir);
    strcat(file_vendor, file_med);
    strcat(file_vendor, VENDOR);
    debug_print("vendor file name is %s\n", file_vendor);
    strcat(file_device, file_med);
    strcat(file_device, DEVICE);
    debug_print("device file name is %s\n", file_device);
    strcat(file_resource, file_med);
    strcat(file_resource, RESOURCE);
    debug_print("resource file name is %s\n", file_resource);
    strcat(file_class, file_med);
    strcat(file_class, CLASS);
    debug_print("class file name is %s\n", file_class);
    fv = fopen(file_vendor, "r");
    if (fv == NULL) {
      debug_print("vendor file %s open fail\n", file_vendor);
      fclose(fp);
      return -1;
    }
    fd = fopen(file_device, "r");
    if (fd == NULL) {
      debug_print("device file %s open fail\n", file_device);
      fclose(fv);
      fclose(fp);
      return -1;
    }
    fr = fopen(file_resource, "r");
    if (fr == NULL) {
      debug_print("resource file %s open fail\n",
                file_resource);
      fclose(fd);
      fclose(fv);
      fclose(fp);
      return -1;
    }

    fc = fopen(file_class, "r");
    if (fr == NULL) {
      debug_print("resource file %s open fail\n",
                file_class);
      fclose(fd);
      fclose(fv);
      fclose(fp);
      fclose(fr);
      return -1;
    }

    fgets(vendor_id, 7, fv);
    fgets(device_id, 7, fd);
    fgets(class, 11, fc);
    vendor = strtoul(vendor_id, NULL, 16);
    device = strtoul(device_id, NULL, 16);
    class_code = strtoul(class, NULL, 16);
    if ((vendor == VENDOR_ID) && ((device >= DEVICE_ID))) {
      if ((class_code  >> 8) == 0x0604)
        debug_print("Skipping RC: file %s\n", file_med);
      else {
        eps++;
        debug_print("\n EP has been found here vendor=0x%X "
            "device=0x%X\n", vendor, device);
        if (add_resource_in_list(fr, start) == -1) {
          debug_print("resource fetch fail\n");
          goto out;
        }
      }
    } else {
      debug_print("this file is not related to EP : "
              "file %s\n", file_med);
    }
    fclose(fd);
    fclose(fr);
    fclose(fv);
    fclose(fc);
  }

out:
  fclose(fd);
  fclose(fv);
  fclose(fp);
  fclose(fr);
  fclose(fc);
  return -1;
}

/**
 * propagate_system_info(): propaget PCI subsytem mapping in RC
 * address space to every EP inlist.
 * @start: start of list
 * @fd: file descriptor returned from RC module device file.
 * @eps: no of eps as returned by get_device()
 */

int propagate_system_info(struct pci_sys_info *start, int fd,
          int eps, unsigned int startaddr)
{
  unsigned int bar2_addr = 0;
  unsigned int bar2_size = 0;
  unsigned int *mgmt_area;
  struct pci_sys_info *temp;
  /*fetch unique id of every node and fill them in structure*/
  for (temp = start; temp != NULL; temp = temp->next) {

    bar2_addr = temp->res_value[3][0];
    bar2_size = temp->res_value[3][1];

    mgmt_area = mmap(0, bar2_size, PROT_READ|PROT_WRITE,
          MAP_SHARED, fd, (off_t)bar2_addr);
    debug_print("bar2_addr:%08x (%08x) mapped:%08x\n", bar2_addr, bar2_size, mgmt_area);

    if ((void *)-1 == (void *) mgmt_area) {
      debug_print("mmap bar2 of EP having ID %u "
          "failed\n", temp->res_value[0][0]);
      close(fd);
      return -1;
    }
    fetch_my_unique_id(mgmt_area, temp);
    munmap(mgmt_area, bar2_size);
  }

  for (temp = start; temp != NULL ; temp = temp->next) {

    bar2_addr = temp->res_value[3][0];
    bar2_size = temp->res_value[3][1];

    mgmt_area = mmap(0, bar2_size, PROT_READ | PROT_WRITE,
          MAP_SHARED, fd, (off_t) bar2_addr);
    debug_print("bar2_addr:%08x (%08x) mapped:%08x\n", bar2_addr, bar2_size, mgmt_area);
    if ((void *)-1 == (void *) mgmt_area) {
      debug_print("mmap bar2 of EP having Id %u "
          "failed\n", temp->res_value[0][0]);
      close(fd);
      return -1;
    }
    dump_info_on_ep(start, mgmt_area , eps, startaddr);
    munmap(mgmt_area, bar2_size);
  }
  return 0;
}
