CXX = g++
CXXFLAGS = -Iinclude -Wall -Wextra
SOURCES = src/main.cpp src/Graph.cpp src/GraphGenerator.cpp
OBJECTS = $(SOURCES:.cpp=.o)
TARGET = my_program

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CXX) -o $@ $^

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $<

clean:
	rm -f $(OBJECTS) $(TARGET)
