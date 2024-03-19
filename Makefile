.PHONY: bin clean test replay default debug run

default: bin

bin: main.cpp
	g++ -o build/main *.cpp -std=c++17 -O2 -Wall -Wextra -Werror -DNDEBUG

debug: main.cpp
	g++ -o build/main *.cpp -std=c++17 -g -Og -Wall -Wextra -Werror

ifeq ($(1),)
  ARG1 = "maps/map1.txt"
else
  ARG1 = $(1)
endif

run: bin
	LinuxReleasev1.2/PreliminaryJudge build/main -m $(ARG1)

clean:
	rm -rf build/main *.zip

test: debug
	LinuxReleasev1.2/PreliminaryJudge build/main -m $(ARG1)

replay:
	xdg-open LinuxReleasev1.2/replayer/

zip:
	zip -rq `date +%F-%H-%M-%S`.zip *.cpp *.h CMakeLists.txt