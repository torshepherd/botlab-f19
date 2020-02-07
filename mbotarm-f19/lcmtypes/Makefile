LCM = lcm-gen
LCMTYPES_NAMES := $(shell ls *.lcm)

# Construct the pathnames to the C types
LCMTYPES_PY := $(LCMTYPES_NAMES:%.lcm=%.py)
ALL = $(LCMTYPES_PY)

all: $(ALL)

%.py: %.lcm
	$(LCM) -p --package-prefix lcmtypes $<

clean:
	rm -rf lcmtypes/
	rm -f *.c *.h *.o *.cpp *.hpp *~
