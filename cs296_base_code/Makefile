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
WD		= cd
PROFILE	= gprof

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
CPPFLAGS+=-I $(BOX2D_ROOT)/include -I $(GLUI_ROOT)/include
LDFLAGS+=-L $(BOX2D_ROOT)/lib -L $(GLUI_ROOT)/lib

SRCS := $(wildcard $(SRCDIR)/*.cpp)
INCS := $(wildcard $(SRCDIR)/*.hpp)
OBJS := $(SRCS:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)

#the default ones
DEBUG = Release

FLAGS=-Wall -g -pg -fno-omit-frame-pointer -O2 -DNDEBUG \
				-fno-inline-functions -fno-inline-functions-called-once \
				-fno-optimize-sibling-calls

######################################

.PHONY: setup box2d gui nogui fixup exe doc clean distclean dist report

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
	 && cmake  -DCMAKE_BUILD_TYPE=$(DEBUG) ../ >/dev/null  \
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
	@$(CC) $(FLAGS) -o $@ $(LDFLAGS) $(OBJS) $(LIBS) 

-include $(OBJS:.o=.d)

$(OBJS): $(OBJDIR)/%.o : $(SRCDIR)/%.cpp
	@$(CC) $(FLAGS) $(CPPFLAGS) -c $< -o $@ -MD

doc:
	@$(ECHO) -n "Generating Doxygen Documentation... "
	@$(RM) -rf doc/html
	@$(DOXYGEN) $(DOCDIR)/Doxyfile 2>/dev/null
	@$(ECHO) "Done!"

clean:
	@$(ECHO) -n "Cleaning up... "
	@$(RM) -rf $(OBJDIR) $(PLOTDIR) $(DATADIR) *~  $(SRCDIR)/*~ $(OBJDIR) *.out
	@$(RM) -f $(PROJECT_ROOT)/perf.data.old
	@$(ECHO) "Done!"

#distclean:also remove bin directory,html in doc,external/src/Box2D , external/include , external/lib (i.e ones created by setting box2d)
distclean: clean 
	@$(RM) -rf $(EXTERNAL_ROOT)/src/Box2D $(EXTERNAL_ROOT)/lib $(EXTERNAL_ROOT)/include #remove box2d installed traces
	@$(RM) -rf $(BINDIR) $(DOCDIR)/html	#remove html , bin directory

dist: distclean
	@cd ..; tar cfz cs296_g13_project.tar.gz cs296_base_code
	
TEXFILE=g13_report
	
report:
	@pdflatex $(DOCDIR)/$(TEXFILE).tex >/dev/null
	@mv $(TEXFILE).pdf $(DOCDIR)/$(TEXFILE).pdf
	@rm -f *.dvi *.ps *.log *.aux *.synctex.gz \
		$(DOCDIR)/*.dvi $(DOCDIR)/*.ps $(DOCDIR)/*.log \
		$(DOCDIR)/*.aux $(DOCDIR)/*.synctex.gz
	

############   PLOTTING   		################################

#using python matplotlib
plot:
	@mkdir -p $(DATADIR)
	@mkdir -p $(PLOTDIR)
	@./$(SCRIPTDIR)/g13_gen_csv.py
	@./$(SCRIPTDIR)/g13_gen_plots.py

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

plotgnu: direct temp drawplots

############   PROFILING   		##################################
debug_prof: FLAGS=-Wall -g -pg -fno-omit-frame-pointer
debug_prof: DEBUG=Debug
debug_prof: DEBUG2=debug

release_prof: FLAGS=-Wall -g -pg -fno-omit-frame-pointer -O2 -DNDEBUG \
				-fno-inline-functions -fno-inline-functions-called-once \
				-fno-optimize-sibling-calls
release_prof: DEBUG=Release
release_prof: DEBUG2=release


	
debug_prof: box2d noguiexe profile

release_prof: box2d noguiexe profile

ITRS=10000

# Note: Since we couldn't easily analyze the perf-generated report (which was quite long), we
# store the report generated by gprof instead. Nonetheless, that the call-graph is still made
# with the statistics generated by perf. @perf report > $(DATADIR)/g13_$(DEBUG2)_prof.dat

profile:
	@mkdir -p $(DATADIR)
	@rm -f $(DATADIR)/g13_$(DEBUG2)_prof.dat $(DATADIR)/$(DEBUG2).png
	@perf record -g $(BINDIR)/$(TARGET) $(ITRS) >/dev/null 2>/dev/null
	@$(PROFILE) $(BINDIR)/$(TARGET) gmon.out > $(DATADIR)/g13_$(DEBUG2)_prof.dat
	@perf script 2>/dev/null | $(SCRIPTDIR)/gprof2dot.py -s -f perf | dot -Tpng -o $(DATADIR)/$(DEBUG2).png
	@rm -f perf.data gmon.out


