all: simulator

simulator: simple.cpp
	g++ -o simulator `pkg-config --cflags playerc` simple.cpp `pkg-config --libs playerc++`
