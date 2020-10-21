CXX               := g++
CXX_FLAGS         := -Wall -Wextra -std=c++17 -ggdb

BIN               := bin
SRC               := src
INCLUDE           := inc
LIB               := lib

BOOST_ROOT        := /home/eicossa/repo/boost_install
BOOST_INCL        := ${BOOST_ROOT}/include/
BOOST_LIBS        := ${BOOST_ROOT}/libs/ 

PYTHON_INCL       := /usr/include/python3.8m/
PYTHON_LIBS       := /usr/lib/python3.8/

LIBFLAGS          := -lboost_system  -lboost_python37 -lpython3.7m #changes made here
EXECUTABLE        := hello


all: $(BIN)/$(EXECUTABLE)

run: clean all
	clear
	./$(BIN)/$(EXECUTABLE)

$(BIN)/$(EXECUTABLE): $(SRC)/*.cpp 
	$(CXX) $(CXX_FLAGS) -I$(PYTHON_INCL) -I$(INCLUDE) -I$(BOOST_INCL) -L$(BOOST_LIBS) -L$(PYTHON_LIBS) -L$(LIB$

clean:
	-rm $(BIN)/*
