DEFINES+=PROJECT_CONF_H=\"project-conf.h\"
CONTIKI_PROJECT = very-sleepy-demo

all: $(CONTIKI_PROJECT)

CONTIKI_WITH_IPV6 = 1

PROJECTDIR?=.

APPDIRS += $(PROJECTDIR)/apps

APPS += er-coap
APPS += rest-engine
APPS += ota

OTA=1

CONTIKI = ../../../..
MODULES += core/net/ipv6/multicast


include $(CONTIKI)/Makefile.include
