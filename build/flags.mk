CXX_FEATURES = -fno-exceptions 
C_FEATURES =

CPPFLAGS := $(INCLUDES) $(TARGET_CPPFLAGS)
CXXFLAGS := $(OPTIMIZE) $(PROFILE) $(CXX_FEATURES)
CFLAGS := $(OPTIMIZE) $(PROFILE) $(C_FEATURES)


CPPFLAGS        += -DFLARM_AVERAGE -DFIXED_MATH
