#
# Setup the flags based on debug versus
#
ifndef CONFIG
$(error missing the CONFIG variable on the make command line, e.g. make CONFIG=debug)
else
ifeq ($(CONFIG), debug)
CXXFLAGS += -g
else
ifeq ($(CONFIG), release)
CXXFLAGS += -O3
else
$(error CONFIG must be set to either 'debug' or 'release')
endif
endif
endif

#
# The compiler to use to build the robot code
#
ifeq ($(OS),Windows_NT)
CXX = ../../external/frc/bin/arm-frc-linux-gnueabi-g++
AR = ../../external/frc/bin/arm-frc-linux-gnueabi-ar
else
CXX = arm-frc-linux-gnueabi-g++
CXX = arm-frc-linux-gnueabi-ar
endif


#
# The flags to apply to the C++ compilation
#
CXXFLAGS = -std=c++14 -Wall

#
# The preprocessor flags to apply
#
CPPFLAGS =

#
# The build directory based on the CONFIG
#
TARGETDIR=../builddir/$(CONFIG)

#
# The set of directories to make in the target
#
TARGETMKDIRS=$(addprefix $(OBJDIR)/,$(MKDIRS))

#
# Target name
#
TARGETNAME=$(basename $(TARGET))

#
# The directory for the object files
#
OBJDIR=$(TARGETDIR)/obj/$(TARGETNAME)

#
# Generate the list of object files desired
#
OBJS=$(addprefix $(OBJDIR)/,$(patsubst %.cpp,%.o,$(SRC)))

#
# Check to see if we need the NAVX libraries
#
ifdef NAVX
ifeq ($(NAVX), true)
CXXFLAGS += -I../../external/navx/include
ADDLIBS += -L../../external/navx/lib -lnavx_frc_cpp
endif
endif

#
# Check to see if we need the CTRE libraries
#
ifdef CTRE
ifeq ($(CTRE), true)
CXXFLAGS += -I../../external/CTRE_FRCLibs/cpp/include
ADDLIBS += -L../../external/CTRE_FRCLibs/cpp/lib -lCTRE_Phoenix -lCTRE_PhoenixCCI
endif
endif

#
# Check to see if we need the WPILIB
#
ifdef WPILIB
ifeq ($(WPILIB), true)
CXXFLAGS += -I../../external/wpilib/cpp/current/include
ADDLIBS += -L../../external/wpilib/cpp/current/reflib/linux/athena/shared
ADDLIBS += -L../../external/wpilib/common/current/lib/linux/athena/shared
ADDLIBS += -lwpi -lpthread
endif
endif

#
# Check to see if we need the boost libraries
#
ifdef BOOST
ifeq ($(BOOST), true)
CXXFLAGS += -I../../external/boost/include
ADDLIBS += ../../external/boost/lib/libboost_system.a
endif
endif
