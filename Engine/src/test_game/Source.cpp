#define SDL_MAIN_HANDLED
#include "Game.h"

int main()
{
	Game game;
	if (!game.Init())
		return 1;
	game.Start();
	return 0;
}