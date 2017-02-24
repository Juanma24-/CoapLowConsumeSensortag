DEFINES+=PROJECT_CONF_H=\"project-conf.h\"
CONTIKI_PROJECT = very-sleepy-demo

TARGET=srf06-cc26xx
BOARD=sensortag/cc2650
CPU_FAMILY=cc26xx

all: $(CONTIKI_PROJECT)

CONTIKI_WITH_IPV6 = 1

APPS += er-coap
APPS += rest-engine

CONTIKI = ../../..
MODULES += core/net/ipv6/multicast

include $(CONTIKI)/Makefile.include
