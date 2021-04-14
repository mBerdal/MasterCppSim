
CC = g++
CFLAGS = -Wall -g -std=c++0x
OBJECT_DEST = object_files/

proj: $(OBJECT_DEST)$ main.o $(OBJECT_DEST)$ sim.o $(OBJECT_DEST)$ env.o $(OBJECT_DEST)$ range_ray.o
	$(CC) $(CFLAGS) -o proj $(OBJECT_DEST)$ main.o $(OBJECT_DEST)$ sim.o $(OBJECT_DEST)$ env.o $(OBJECT_DEST)$ range_ray.o


$(OBJECT_DEST)$ main.o: main.cpp sim.h
	$(CC) $(CFLAGS) -c main.cpp -o $(OBJECT_DEST)$ main.o

$(OBJECT_DEST)$ sim.o: sim.cpp helper.h env.h
	$(CC) $(CFLAGS) -c sim.cpp -o $(OBJECT_DEST)$ sim.o

$(OBJECT_DEST)$ env.o: env.cpp
	$(CC) $(CFLAGS) -c env.cpp -o $(OBJECT_DEST)$ env.o

$(OBJECT_DEST)$ range_ray.o: range_ray.cpp helper.h env.h
	$(CC) $(CFLAGS) -c range_ray.cpp -o $(OBJECT_DEST)$ range_ray.o

clean:
	rm -f proj object_files/*.o