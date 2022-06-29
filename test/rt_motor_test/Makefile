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
CFLAGS += -O3 -Wall -g


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
# define the CPP source files
SRCS = rt_motor_test.cpp

# define the CPP object files 
#
# This uses Suffix Replacement within a macro:
#   $(name:string1=string2)
#         For each word in 'name' replace 'string1' with 'string2'
# Below we are replacing the suffix .cpp of all words in the macro SRCS
# with the .o suffix
#
OBJS = $(SRCS:.cpp=.o)
 
# define the executable file 
MAIN = rt_motor_test
#
# The following part of the makefile is generic; it can be used to 
# build any executable just by changing the definitions above and by
# deleting dependencies appended to the file from 'make depend'
#

.PHONY: depend clean

all:    $(MAIN)

$(MAIN): $(OBJS)
	$(CC) $(CFLAGS) -o $(MAIN) $(OBJS) $(LDLIBS)

# this is a suffix replacement rule for building .o's from .cpp's
# it uses automatic variables $<: the name of the prerequisite of
# the rule(a .cpp file) and $@: the name of the target of the rule (a .o file) 
# (see the gnu make manual section about automatic variables)
.cpp.o:
	$(CC) $(CFLAGS) $(LDLIBS) -c $<  -o $@

clean:
	$(RM) *.o


# DO NOT DELETE THIS LINE -- make depend needs it