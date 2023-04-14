CC = g++
CFLAGS =
LIBS = 
SIMLIBS = -lsfml-graphics -lsfml-window -lsfml-system

prm: prm.cpp agent.cpp graph.cpp
	$(CC) $(CFLAGS) $(LIBS) graph.cpp agent.cpp prm.cpp -o prm

sim: prm.cpp agent.cpp graph.cpp sim.cpp
	$(CC) $(CFLAGS) $(LIBS) $(SIMLIBS) -DSIM graph.cpp agent.cpp prm.cpp sim.cpp -o prm

debug: prm.cpp agent.cpp graph.cpp
	$(CC) $(CFLAGS) $(LIBS) -g graph.cpp agent.cpp prm.cpp -o prm

clean: 
	rm -f prm
