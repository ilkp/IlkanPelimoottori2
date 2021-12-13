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

	_transformSystem.Reserve(_camera);
	_transformSystem.Identity(_camera);
	_cameraSystem.Reserve(_camera);
	_cameraSystem.Identity(_camera);

	_transformSystem.SetPosition(_camera._entityId, 20.0f, 20.0f, 0.0f);
	_transformSystem.SetRotation(_camera._entityId,
		glm::lookAt(
			_transformSystem.GetPosition(_camera._entityId),
			glm::vec3(0.0f, 0.0f, 0.0f),
			glm::vec3(0.0f, -1.0f, 0.0f)));
	_meshSystem.SetMesh(_cubeMesh, idop::primitives::Cube());

	float randomPos = 40.0f;
	int nItems = 1;

	//for (int i = 0; i < nItems; ++i)
	//{
	//	_cubes.push_back(idop::Entity(_entitySystem.Next()));
	//	_transformSystem.Reserve(_cubes[i]._entityId);
	//	_transformSystem.Identity(_cubes[i]._entityId);
	//	//_transformSystem.SetPosition(_cubes[i]._entityId, -randomPos + (float)(rand()) / (float(RAND_MAX / (randomPos * 2.0f))), -randomPos + (float)(rand()) / (float(RAND_MAX / (randomPos * 2.0f))), -randomPos + (float)(rand()) / (float(RAND_MAX / (randomPos * 2.0f))));
	//	_rigidBodySystem.Reserve(_cubes[i]._entityId);
	//	_rigidBodySystem.Identity(_cubes[i]._entityId);
	//	_colliderSystem.Reserve(_cubes[i]._entityId);
	//	_colliderSystem.Identity(_cubes[i]._entityId);
	//	_colliderSystem._componentData[_cubes[i]._entityId & _colliderSystem.INDEX_BITS_SEQ]._meshId[_cubes[i]._entityId & _colliderSystem.INDEX_BITS_COMP] = _cubeMesh._entityId;
	//	_meshMVP.push_back(_cubes[i]._entityId);
	//}


	_transformSystem.Reserve(_cube._entityId);
	_transformSystem.Identity(_cube._entityId);
	_rigidBodySystem.Reserve(_cube._entityId);
	_rigidBodySystem.Identity(_cube._entityId);
	_colliderSystem.Reserve(_cube._entityId);
	_colliderSystem.Identity(_cube._entityId);
	_colliderSystem._componentData[_cube._entityId & _colliderSystem.INDEX_BITS_SEQ]._meshId[_cube._entityId & _colliderSystem.INDEX_BITS_COMP] = _cubeMesh._entityId;
	_meshMVP.push_back(_cube._entityId);
	_transformSystem.SetPosition(_cube._entityId, -5.0f, 8.0f, 0.0f);
	_rigidBodySystem.SetVelocity(_cube._entityId, 0.0f, 0.01f, 0.0f);
	_rigidBodySystem.SetAngularVelocity(_cube._entityId, 0.04f, 0.04f, -0.04f);

	//_transformSystem.SetPosition(_cubes[0]._entityId, -2.0f, 8.0f, 0.0f);
	//_rigidBodySystem.SetVelocity(_cubes[0]._entityId, 0.02f, 0.0f, 0.0f);
	//_rigidBodySystem.SetAngularVelocity(_cubes[0]._entityId, 0.0f, 0.0f, -0.02f);
	//_transformSystem.SetPosition(_cubes[1]._entityId, 0.0f, 10.0f, -5.0f);
	//_rigidBodySystem.SetVelocity(_cubes[1]._entityId, 0.0f, 0.0f, 0.05f);
	//_rigidBodySystem.SetAngularVelocity(_cubes[1]._entityId, 0.0f, -0.01f, 0.0f);

	_rigidBodySystem.SetMass(_cube._entityId, 1.0f);

	_transformSystem.Reserve(_floor._entityId);
	_transformSystem.Identity(_floor._entityId);
	_transformSystem.SetScale(_floor._entityId, 20.0f, 4.0f, 20.0f);
	_transformSystem.SetPosition(_floor._entityId, 0.0f, 0.0f, 0.0f);
	//_transformSystem._componentData.at(_floor._entityId & _transformSystem.INDEX_BITS_SEQ)._isStatic[_floor._entityId & _transformSystem.INDEX_BITS_COMP] = true;
	_colliderSystem.Reserve(_floor._entityId);
	_colliderSystem.Identity(_floor._entityId);
	_colliderSystem._componentData[_floor._entityId & _colliderSystem.INDEX_BITS_SEQ]._meshId[_floor._entityId & _colliderSystem.INDEX_BITS_COMP] = _cubeMesh._entityId;
	_meshMVP.push_back(_floor._entityId);

	_transformSystem.Reserve(_walle._entityId);
	_transformSystem.Identity(_walle._entityId);
	_transformSystem.SetScale(_walle._entityId, 2.0f, 5.0f, 20.0f);
	_transformSystem.SetPosition(_walle._entityId, 10.0f, 4.0f, 0.0f);
	_transformSystem._componentData.at(_walle._entityId & _transformSystem.INDEX_BITS_SEQ)._isStatic[_walle._entityId & _transformSystem.INDEX_BITS_COMP] = true;;
	_colliderSystem.Reserve(_walle._entityId);
	_colliderSystem.Identity(_walle._entityId);
	_colliderSystem._componentData[_walle._entityId & _colliderSystem.INDEX_BITS_SEQ]._meshId[_walle._entityId & _colliderSystem.INDEX_BITS_COMP] = _cubeMesh._entityId;
	_meshMVP.push_back(_walle._entityId);

	_transformSystem.Reserve(_wallw._entityId);
	_transformSystem.Identity(_wallw._entityId);
	_transformSystem.SetScale(_wallw._entityId, 2.0f, 5.0f, 20.0f);
	_transformSystem.SetPosition(_wallw._entityId, -10.0f, 4.0f, 0.0f);
	_transformSystem._componentData.at(_wallw._entityId & _transformSystem.INDEX_BITS_SEQ)._isStatic[_wallw._entityId & _transformSystem.INDEX_BITS_COMP] = true;;
	_colliderSystem.Reserve(_wallw._entityId);
	_colliderSystem.Identity(_wallw._entityId);
	_colliderSystem._componentData[_wallw._entityId & _colliderSystem.INDEX_BITS_SEQ]._meshId[_wallw._entityId & _colliderSystem.INDEX_BITS_COMP] = _cubeMesh._entityId;
	_meshMVP.push_back(_wallw._entityId);

	_transformSystem.Reserve(_walln._entityId);
	_transformSystem.Identity(_walln._entityId);
	_transformSystem.SetScale(_walln._entityId, 20.0f, 5.0f, 2.0f);
	_transformSystem.SetPosition(_walln._entityId, 0.0f, 4.0f, 10.0f);
	_transformSystem._componentData.at(_walln._entityId & _transformSystem.INDEX_BITS_SEQ)._isStatic[_walln._entityId & _transformSystem.INDEX_BITS_COMP] = true;;
	_colliderSystem.Reserve(_walln._entityId);
	_colliderSystem.Identity(_walln._entityId);
	_colliderSystem._componentData[_walln._entityId & _colliderSystem.INDEX_BITS_SEQ]._meshId[_walln._entityId & _colliderSystem.INDEX_BITS_COMP] = _cubeMesh._entityId;
	_meshMVP.push_back(_walln._entityId);

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
		lastFrameStart = std::chrono::system_clock::now();
		_timeSinceStart += elapsedTime.count();
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
	//UpdateCubes(deltaTime);
	_colliderSystem.CalculateWorldBounds(_transformSystem, _meshSystem);
	idop::phys::ProcessRigidbodies(_transformSystem, _rigidBodySystem, _colliderSystem, _meshSystem, deltaTime, _gravity);
	_transformSystem.CalculateMVP();
	_cameraSystem.LookAt(_camera._entityId, glm::vec3(5.0f, 10.0f, -20.0f), glm::vec3(0.0f, 2.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f));
	//_cameraSystem.ApplyTransforms(_camera._entityId, _transformSystem.GetPosition(_camera._entityId), _transformSystem.GetRotation(_camera._entityId));
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
