#include <iostream>
#include <unistd.h>
#include "osl/file_ipc.h"
#include "osl/transform.h"
#include "bitgrid_RMC.h"

int main(int argc,const char *args[]) {
  if (argc<=1) { printf("Usage: dump_grid  foo.grid\n"); return 1; }
  
  file_ipc_link<bitgrid> grid_link(args[1]);
  bitgrid grid;
  while (true) {
	  if (grid_link.subscribe(grid)) {
		  //printf("\033[0;0f"); // seek to start of screen
		  printf("\033[2J"); // seek to (0,0) and clear screen
  		  grid.print();
	  }
	  usleep(10*1000);
  }
  
  return 0;
}


