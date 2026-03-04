.PHONY: build run clean setup

setup:
	git clone https://github.com/raysan5/raylib.git ext/raylib
	cd ext/raylib/src && mingw32-make PLATFORM=PLATFORM_DESKTOP

build:
	g++ src/main.cpp -o build/game.exe -I ext/raylib/src -L ext/raylib/src -lraylib -lopengl32 -lgdi32 -lwinmm

run: build
	.\build\game.exe

clean:
	del build\game.exe