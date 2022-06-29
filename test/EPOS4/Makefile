#
# 'make depend' uses makedepend to automatically generate dependencies 
#               (dependencies are added to end of Makefile)
# 'make'        build executable file 'mycc'
# 'make clean'  removes all .o and executable files
#

# define the CPP compiler to use
CC = g++

# define any compile-time flags

#SOEM EtherCAT
CFLAGS += -I../../soem 
CFLAGS += -I../../osal
CFLAGS += -I../../oshw
CFLAGS += -O3 -Wall -g -Wno-narrowing


# define any libraries to link into executable:
#   if I want to link in libraries (libx.so or libx.a) I use the -llibname 
#   option, something like (this will link in libmylib.so and libm.so:

#SOEM EtherCAT
LDLIBS += -L../../lib
LDLIBS +=-lsoem
LDLIBS +=-losal
LDLIBS +=-loshw
LDLIBS +=-lrt
LDLIBS +=-lpthread 


SOURCE_DIR = src
INCLUDE_DIR = include

OBJECTS = $(patsubst %.cpp, %.o, $(wildcard $(SOURCE_DIR)/*.cpp))
HEADERS = $(wildcard $(INCLUDE_DIR)/*.h)


# define the executable file 
MAIN = main


all: $(MAIN)

$(MAIN): $(OBJECTS)
	$(CC) $(CFLAGS) -o $(MAIN) $(OBJECTS) $(LDLIBS)

%.o: %.cpp $(HEADERS)
	$(CC) $(CFLAGS) $(LDLIBS) -c $< -o $@ -I$(INCLUDE_DIR)

clean:
	$(RM) $(SOURCE_DIR)/*.o