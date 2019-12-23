#include "aurora/beacon_commands.h"
#include <stdio.h>

int main(int argc,char *argv[]) {
  if (argc<3) { printf("Usage: beacon <letter> <angle>\n"); return 1; }
  char letter=argv[1][0];
  int angle=atoi(argv[2]);
  std::vector<unsigned char> data;
  send_aurora_beacon_command(letter,data,angle);
  for (size_t i=0;i<data.size();i++) printf("%02x ",data[i]);
  return 0;
}
