all:
	g++ *.c -o april `pkg-config --cflags --libs opencv`
