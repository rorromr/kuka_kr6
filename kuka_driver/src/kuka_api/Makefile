PROJECTROOT=..

CC = gcc
AR= ar rc 
RM = rm

CFLAGS += -g -Wall -O3 -Wno-unused
LDLIBS += -lnsl -lkuka
RPCGENFLAGS =

INCLUDE_PATH=$(PROJECTROOT)/include
LIB_PATH=$(PROJECTROOT)/lib

all : libkuka.a

kuka_clnt.c kuka_xdr.c kuka.h : kuka.x 
	rpcgen $(RPCGENFLAGS) $<

%.o : %.c kuka.h
	$(CC) $(CFLAGS) -c $< -I$(INCLUDE_PATH) 

libkuka.a : kuka_api.o kuka_clnt.o kuka_xdr.o
	$(AR) $@ $^
	cp -f $@ $(PROJECTROOT)/lib/$@
	cp -f kuka_api.h $(PROJECTROOT)/include/kuka_api.h 

clean:
	$(RM) -f *~ *.*~ *.o *.a
	$(RM) -f kuka_clnt.c kuka_svc.c kuka_xdr.c kuka.h
	$(RM) -f $(PROJECTROOT)/lib/libkuka.a
	$(RM) -f $(PROJECTROOT)/include/kuka_api.h
