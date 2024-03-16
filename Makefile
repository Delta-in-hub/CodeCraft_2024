.PHONY: bin clean test replay default debug run

default: bin

bin: main.cpp
	g++ -o build/main *.cpp -std=c++17 -O2 -Wall -Wextra -Werror -DNDEBUG

debug: main.cpp
	g++ -o build/main *.cpp -std=c++17 -g -Wall -Wextra -Werror

run: bin
	LinuxReleasev1.1/PreliminaryJudge build/main -m maps/map1.txt

clean:
	rm -rf build/main

test: debug
	LinuxReleasev1.1/PreliminaryJudge build/main -m maps/map1.txt

replay:
	xdg-open LinuxReleasev1.1/replayer/