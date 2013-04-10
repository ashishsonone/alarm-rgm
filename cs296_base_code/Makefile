.SUFFIXES: .cpp .hpp

# Programs
SHELL 	= bash
CC     	= g++
LD	    = ld
RM  	= rm
ECHO	= /bin/echo
CAT	    = cat
PRINTF	= printf
SED	    = sed
DOXYGEN = doxygen
GRAPH	= gnuplot
TMPDATA	= awk

# Project Name (Executable Name)
TARGET = cs296_exe_13

# Project Paths
PROJECT_ROOT=.
EXTERNAL_ROOT=$(PROJECT_ROOT)/external
SRCDIR = $(PROJECT_ROOT)/src
OBJDIR = $(PROJECT_ROOT)/obj
BINDIR = $(PROJECT_ROOT)/bin
DOCDIR = $(PROJECT_ROOT)/doc
SCRIPTDIR = $(PROJECT_ROOT)/scripts
DATADIR		= $(PROJECT_ROOT)/data
PLOTDIR		= $(PROJECT_ROOT)/plots

# Library Paths
BOX2D_ROOT=$(EXTERNAL_ROOT)
GLUI_ROOT=/usr
GL_ROOT=/usr

#Libraries
LIBS = -lBox2D -lglui -lglut -lGLU -lGL

# Compiler and Linker flags
CPPFLAGS =-g -O3 -Wall 
CPPFLAGS+=-I $(BOX2D_ROOT)/include -I $(GLUI_ROOT)/include
LDFLAGS+=-L $(BOX2D_ROOT)/lib -L $(GLUI_ROOT)/lib

SRCS := $(wildcard $(SRCDIR)/*.cpp)
INCS := $(wildcard $(SRCDIR)/*.hpp)
OBJS := $(SRCS:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)

######################################

.PHONY: setup box2d gui nogui fixup exe doc clean distclean dist

all: exe

setup:
	@$(ECHO) "Setting up compilation..."
	@mkdir -p obj
	@mkdir -p bin

# Extract and Compile the Box2D Library
box2d:
	@cd $(EXTERNAL_ROOT)/src \
	 && tar xvf Box2D.tgz >/dev/null 
	@cd $(EXTERNAL_ROOT)/src/Box2D \
	 && mkdir -p build296 \
	 && cd build296 \
	 && cmake ../ >/dev/null \
	 && make >/dev/null \
	 && make install >/dev/null

fixup:
	@rm -f $(SRCDIR)/main.cpp

gui: fixup
	@cd $(SRCDIR); cp main.gui main.cpp
    
nogui: fixup
	@cp $(SRCDIR)/main.nogui $(SRCDIR)/main.cpp

exe: setup gui $(BINDIR)/$(TARGET) fixup

noguiexe: setup nogui $(BINDIR)/$(TARGET) fixup

    
$(BINDIR)/$(TARGET): $(OBJS)
	@$(CC) -o $@ $(LDFLAGS) $(OBJS) $(LIBS) 

-include $(OBJS:.o=.d)

$(OBJS): $(OBJDIR)/%.o : $(SRCDIR)/%.cpp
	@$(CC) $(CPPFLAGS) -c $< -o $@ -MD

doc:
	@$(ECHO) -n "Generating Doxygen Documentation... "
	@$(RM) -rf doc/html
	@$(DOXYGEN) $(DOCDIR)/Doxyfile 2>/dev/null
	@$(ECHO) "Done!"

clean:
	@$(ECHO) -n "Cleaning up... "
	@$(RM) -rf $(OBJDIR) $(PLOTDIR) $(DATADIR) *~  $(SRCDIR)/*~ $(OBJDIR)
	@$(ECHO) "Done!"

#distclean:also remove bin directory,html in doc,external/src/Box2D , external/include , external/lib (i.e ones created by setting box2d)
distclean: clean 
	@$(RM) -rf $(EXTERNAL_ROOT)/src/Box2D $(EXTERNAL_ROOT)/lib $(EXTERNAL_ROOT)/include #remove box2d installed traces
	@$(RM) -rf $(BINDIR) $(DOCDIR)/html	#remove html , bin directory

dist: distclean
	@cd ..; tar cfz cs296_g13_project.tar.gz cs296_base_code
	
#the extra stuff
############PLOTTING
direct:
	@scripts/gen_data_csv.sh

temp:
	@for i in 1 2 3 4 5 6 ; do \
		$(TMPDATA) -F "\"*,\"*" -f $(SCRIPTDIR)/helper0$$i.awk \
			$(DATADIR)/lab05_g13_data.csv > $(DATADIR)/helper0$$i ; \
	done

drawplots:
	@mkdir -p $(PLOTDIR)
	@for i in 1 2 3 4 5 6 ; do \
		$(GRAPH) $(SCRIPTDIR)/g13_plot0$$i.gpt ; \
		rm -f $(DATADIR)/helper0$$i* ; \
	done

####################################################################################################

plot: direct temp drawplots
