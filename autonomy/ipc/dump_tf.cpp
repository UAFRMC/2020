#include <iostream>
#include <unistd.h>
#include "osl/file_ipc.h"
#include "osl/transform.h"

int main(int argc,const char *args[]) {
  if (argc<=1) { printf("Usage: dump_tf  /tmp/ipc/foo.tf\n"); return 1; }
  
  file_ipc_link<osl::transform> tf_link(args[1]);
  osl::transform tf;
  while (true) {
	  if (tf_link.subscribe(tf)) {
		  //printf("\033[0;0f"); // seek to start of screen
		  printf("\033[2J"); // seek to (0,0) and clear screen
		  vec3_print("origin: ", tf.origin);
		  tf.basis.print();
	  }
	  usleep(10*1000);
  }
  
  return 0;
}


