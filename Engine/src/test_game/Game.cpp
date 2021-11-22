#include <stdlib.h>
#include <time.h>
#include <chrono>
#include <iostream>
#include <string>
#include "Game.h"
#include "PrimitiveMesh.h"
#include "Physics.h"

bool Game::Init()
{
	srand(time(NULL));
	if (_SDLrenderer.Init(600, 400) != 0)
		return false;

	float randomPos = 40.0f;
	int nItems = 1000;

	for (int i = 0; i < nItems; ++i)
	{
		_cubes.push_back(idop::Entity(_entitySystem.Next()));
		_transformSystem.Reserve(_cubes[i]._entityId);
		_transformSystem.Identity(_cubes[i]._entityId);
		_transformSystem.SetPosition(_cubes[i]._entityId, -randomPos + (float)(rand()) / (float(RAND_MAX / (randomPos * 2.0f))), -randomPos + (float)(rand()) / (float(RAND_MAX / (randomPos * 2.0f))), -randomPos + (float)(rand()) / (float(RAND_MAX / (randomPos * 2.0f))));
		_meshMVP.push_back(_cubes[i]._entityId);
	}

	_cameraSystem.Reserve(_camera);
	_cameraSystem.Identity(_camera);
	_meshSystem.SetMesh(_cubeMesh, std::move(idop::primitives::Cube()));

	//_rigidBodySystem.Reserve(_cubes[0]);
	//_rigidBodySystem.Identity(_cubes[0]);
	//_rigidBodySystem._componentData[_cubes[0]._entityId & _rigidBodySystem.INDEX_BITS_SEQ]._useGravity[_cubes[0]._entityId & _rigidBodySystem.INDEX_BITS_COMP] = true;

	_renderingData._meshMVP.insert(std::make_pair(_cubeMesh._entityId, _meshMVP));

	return true;
}

void Game::Start()
{
	_running = true;
	auto lastFrameStart = std::chrono::system_clock::now();
	auto now = std::chrono::system_clock::now();
	std::chrono::duration<float> elapsedTime;
	const float FPS_MAX_TIME = 0.5f;
	float fpsTimer = 0;
	while (_running)
	{
		HandleEvents();
		now = std::chrono::system_clock::now();
		elapsedTime = now - lastFrameStart;
		_timeSinceStart += elapsedTime.count();
		lastFrameStart = std::chrono::system_clock::now();
		fpsTimer += elapsedTime.count();
		if (fpsTimer > FPS_MAX_TIME)
		{
			_SDLrenderer.SetTitle(("test_game | fps: " + std::to_string(1.0f / (float)elapsedTime.count())).c_str());
			fpsTimer = 0.0f;
		}
		Update((float)elapsedTime.count());
	}
}

void Game::UpdateCubes(float deltaTime)
{
	for (int i = 0; i < _cubes.size(); ++i)
	{
		_transformSystem.Translate(_cubes[i]._entityId, glm::cos(_timeSinceStart) * 5.0f * deltaTime, 0.0f, glm::sin(_timeSinceStart) * 5.0f * deltaTime);
		_transformSystem.Rotate(_cubes[i]._entityId, glm::vec3(10.0f * deltaTime, 0.0f, 0.0f));
	}
}

void Game::UpdateCubesParallel(float deltaTime)
{
	std::vector<std::future<void>> results(_threadPool.size());
	const unsigned int workSize = std::ceil(_cubes.size() / _threadPool.size());
	for (int i = 0; i < _threadPool.size(); ++i)
	{
		results[i] = _threadPool.push([this, i, workSize, deltaTime](int id)
			{
				for (int j = i * workSize; j < i * workSize + workSize && j < _cubes.size(); ++j)
				{
					_transformSystem.Translate(_cubes[j]._entityId, glm::cos(_timeSinceStart) * 5.0f * deltaTime, 0.0f, glm::sin(_timeSinceStart) * 5.0f * deltaTime);
					_transformSystem.Rotate(_cubes[j]._entityId, glm::vec3(10.0f * deltaTime, 0.0f, 0.0f));
				}
			});
	}
	for (int i = 0; i < results.size(); ++i)
		results[i].get();
}

void Game::Update(float deltaTime)
{
	float camPosScale = 70.0f;
	glm::vec3 camPos = glm::vec3(camPosScale, -camPosScale, 2.0f);
	_cameraSystem.LookAt(_camera, camPos, glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
	UpdateCubes(deltaTime);
	idop::phys::ProcessRigidbodies(&_rigidBodySystem, &_transformSystem, deltaTime, _gravity);
	_transformSystem.CalculateMVP();
	_SDLrenderer.Render(_camera._entityId, _renderingData);
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
