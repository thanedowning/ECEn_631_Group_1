all: serial


basic : serial.cpp
	g++ serial.cpp -o $@ -std=c++14 `pkg-config --cflags --libs opencv`

clean :
	-rm serial
