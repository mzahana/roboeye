# Makefile

UNAME="vio"
U_UID=1000
U_GID=1000
.PHONY: openvins-rpi4

all: openvins-rpi4


openvins-rpi4:
	docker build --build-arg USER_GID=$(U_GID) --build-arg USER_UID=$(U_UID) --build-arg USERNAME=$(UNAME) --build-arg BASE_IMAGE="ros:humble-perception" -t mzahana/openvins_rpi:rpi4 . -f Dockerfile.openvins.rpi4
