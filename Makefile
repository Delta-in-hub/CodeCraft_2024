.PHONY: bin clean test replay

default: bin

bin: main.cpp
	g++ -o build/main *.cpp -std=c++17 -O2 -Wall -Wextra -Werror -DNDEBUG

clean:
	rm -rf build/main

test: bin
	LinuxReleasev1.1/PreliminaryJudge build/main -m maps/map1.txt

replay:
	xdg-open LinuxReleasev1.1/replayer/