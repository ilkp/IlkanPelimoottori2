#include <stdlib.h>
#include <time.h>
#include "Game.h"
#include "PrimitiveMesh.h"
#include "Physics.h"
#include <chrono>
#include <iostream>

bool Game::Init()
{
	srand(time(NULL));
	if (_SDLrenderer.Init(600, 400) != 0)
	{
		return false;
	}
	_cameraSystem.Reserve(_camera);
	_cameraSystem.Identity(_camera);
	_meshSystem.SetMesh(_cubeMesh, idop::primitives::Cube());

	for (int i = 0; i < 1000; ++i)
	{
		_cubes.push_back(idop::Entity(_entitySystem.Next()));
	}

	std::vector<uint32_t> meshMVP;
	float randomPos = 40.0f;
	for (int i = 0; i < _cubes.size(); ++i)
	{
		const idop::Entity& entity = _cubes[i];
		_transformSystem.Reserve(entity);
		_transformSystem.Identity(entity);
		_transformSystem.SetPosition(entity._entityId, -randomPos + (float)(rand()) / (float(RAND_MAX/(randomPos * 2.0f))), -randomPos + (float)(rand()) / (float(RAND_MAX / (randomPos * 2.0f))), -randomPos + (float)(rand()) / (float(RAND_MAX / (randomPos * 2.0f))));
		meshMVP.push_back(entity._entityId);
	}

	//_rigidBodySystem.Reserve(_cubes[0]);
	//_rigidBodySystem.Identity(_cubes[0]);
	//_rigidBodySystem._componentData[_cubes[0]._entityId & _rigidBodySystem.INDEX_BITS_SEQ]._useGravity[_cubes[0]._entityId & _rigidBodySystem.INDEX_BITS_COMP] = true;

	_renderingData._meshMVP.insert({ _cubeMesh._entityId, meshMVP });

	return true;
}

void Game::Start()
{
	_running = true;
	Execute();
}

void Game::Execute()
{
	auto lastFrameStart = std::chrono::system_clock::now();
	auto now = std::chrono::system_clock::now();
	std::chrono::duration<float> elapsedTime;
	while (_running)
	{
		HandleEvents();
		now = std::chrono::system_clock::now();
		elapsedTime = now - lastFrameStart;
		_timeSinceStart += elapsedTime.count();
		lastFrameStart = std::chrono::system_clock::now();
		Update((float)elapsedTime.count());
	}
}

void Game::Update(float deltaTime)
{
	float camPosScale = 70.0f;
	glm::vec3 camPos = glm::vec3(camPosScale, -camPosScale, 2.0f);
	_cameraSystem.LookAt(_camera, camPos, glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
	for (int i = 0; i < _cubes.size(); ++i)
	{
		_transformSystem.Translate(_cubes[i]._entityId, glm::cos(_timeSinceStart) * 5.0f * deltaTime, 0.0f, glm::sin(_timeSinceStart) * 5.0f * deltaTime);
		_transformSystem.Rotate(_cubes[i]._entityId, glm::vec3(10.0f * deltaTime, 0.0f, 0.0f));
	}
	idop::phys::ProcessRbs(&_rigidBodySystem, &_transformSystem, deltaTime, _gravity);
	_transformSystem.CalculateMVP();
	_SDLrenderer.Render(_camera._entityId, _renderingData);

	//renderer.Render(camManager._entityData[0]);
	//timer += dt;

	//if (timer > 2.0f)
	//{
	//	timer = 0.0f;
	//	for (int i = 0; i < nEntities; ++i)
	//	{
	//		idop::SetPosition(*entities[i].transformData, glm::cos(i) * 10.0f, 0.0f, glm::sin(i) * 10.0f);
	//		entities[i].rbData->_velocity = -idop::Position(*entities[i].transformData) / glm::length(idop::Position(*entities[i].transformData));
	//		entities[i].rbData->_velocity *= 0.05f;
	//	}
	//}
}

void Game::HandleEvents()
{
	SDL_Event e;
	while (SDL_PollEvent(&e))
	{
		switch (e.type)
		{
		case SDL_QUIT:
			_running = false;
			break;
		}
	}
}
