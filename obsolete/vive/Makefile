all : vive_robot

OPTS=-g -O3 -flto 
CC:=gcc


INCLUDES:=-I../../autonomy/include -Ilibsurvive/include/libsurvive -Ilibsurvive -Ilibsurvive/include -Ilibsurvive/redist 
DEFINES:=-DUSE_DOUBLE

CFLAGS:=$(INCLUDES) $(OPTS) -std=gnu99 $(DEFINES) 
LDFLAGS:=-L/usr/local/lib  -llapacke  -lcblas -lpthread -lusb-1.0 -lz -lX11 -ldl -lm -lpcap -flto -g


#If you want to use HIDAPI on Linux.
#CFLAGS:=$(CFLAGS) -DHIDAPI
#REDISTS:=$(REDISTS) redist/hid-linux.o
#LDFLAGS:=$(LDFLAGS) -ludev

#Useful Preprocessor Directives:
# -DUSE_DOUBLE = use double instead of float for most operations.
# -DNOZLIB = use puff.c
# -DTCC = various things needed for TCC.
# -DWINDOWS -DWIN32 = Building for Windows
# -DHIDAPI = Build vive driver to use USBHID instead of interrupt/control messages.
# -DRUNTIME_SYMNUM = Don't assume __attribute__((constructor)) works.  Instead comb for anything starting with REGISTER.

# unused: redist/crc32.c

lib: ./libsurvive/src/*.c ./libsurvive/redist/*.c ./libsurvive/redist/mpfit/*.c ./libsurvive/src/epnp/*.c 
	gcc $(CFLAGS) -c $^ $(LDFLAGS) 

vive_robot :  main.cpp lib
	g++ -Wall -std=c++11 -Iinclude $(INCLUDES) -o $@ main.cpp *.o $(LDFLAGS) $(DEFINES) $(OPTS)

clean :
	rm -rf *.o vive_robot




