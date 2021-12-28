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
	if (idop::render::Init(_renderer, 800, 600) != 0)
		return false;

	_transformSystem.Reserve(_camera);
	_transformSystem.Identity(_camera);
	_cameraSystem.Reserve(_camera);
	_cameraSystem.Identity(_camera);
	_transformSystem.SetPosition(_camera._entityId, 5.0f, 10.0f, -20.0f);
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
	_transformSystem.SetPosition(_cube._entityId, 0.0f, 4.0f, 0.0f);
	_rigidBodySystem.SetVelocity(_cube._entityId, 3.0f, 0.0f, 0.0f);
	_rigidBodySystem.SetAngularVelocity(_cube._entityId, 0.0f, 5.0f, 0.0f);

	//_transformSystem.SetPosition(_cubes[0]._entityId, -2.0f, 8.0f, 0.0f);
	//_rigidBodySystem.SetVelocity(_cubes[0]._entityId, 0.02f, 0.0f, 0.0f);
	//_rigidBodySystem.SetAngularVelocity(_cubes[0]._entityId, 0.0f, 0.0f, -0.02f);
	//_transformSystem.SetPosition(_cubes[1]._entityId, 0.0f, 10.0f, -5.0f);
	//_rigidBodySystem.SetVelocity(_cubes[1]._entityId, 0.0f, 0.0f, 0.05f);
	//_rigidBodySystem.SetAngularVelocity(_cubes[1]._entityId, 0.0f, -0.01f, 0.0f);

	_rigidBodySystem.SetMass(_cube._entityId, 1.0f);

	_transformSystem.Reserve(_floor._entityId);
	_transformSystem.Identity(_floor._entityId);
	_transformSystem.SetStatic(_floor._entityId, true);
	_transformSystem.SetScale(_floor._entityId, 20.0f, 4.0f, 20.0f);
	_transformSystem.SetPosition(_floor._entityId, 0.0f, 0.0f, 0.0f);
	_colliderSystem.Reserve(_floor._entityId);
	_colliderSystem.Identity(_floor._entityId);
	_colliderSystem._componentData[_floor._entityId & _colliderSystem.INDEX_BITS_SEQ]._meshId[_floor._entityId & _colliderSystem.INDEX_BITS_COMP] = _cubeMesh._entityId;
	_meshMVP.push_back(_floor._entityId);

	_transformSystem.Reserve(_walle._entityId);
	_transformSystem.Identity(_walle._entityId);
	_transformSystem.SetScale(_walle._entityId, 2.0f, 5.0f, 20.0f);
	_transformSystem.SetPosition(_walle._entityId, 10.0f, 4.0f, 0.0f);
	_transformSystem.SetStatic(_walle._entityId, true);
	_colliderSystem.Reserve(_walle._entityId);
	_colliderSystem.Identity(_walle._entityId);
	_colliderSystem._componentData[_walle._entityId & _colliderSystem.INDEX_BITS_SEQ]._meshId[_walle._entityId & _colliderSystem.INDEX_BITS_COMP] = _cubeMesh._entityId;
	_meshMVP.push_back(_walle._entityId);

	_transformSystem.Reserve(_wallw._entityId);
	_transformSystem.Identity(_wallw._entityId);
	_transformSystem.SetScale(_wallw._entityId, 2.0f, 5.0f, 20.0f);
	_transformSystem.SetPosition(_wallw._entityId, -10.0f, 4.0f, 0.0f);
	_transformSystem.SetStatic(_wallw._entityId, true);
	_colliderSystem.Reserve(_wallw._entityId);
	_colliderSystem.Identity(_wallw._entityId);
	_colliderSystem._componentData[_wallw._entityId & _colliderSystem.INDEX_BITS_SEQ]._meshId[_wallw._entityId & _colliderSystem.INDEX_BITS_COMP] = _cubeMesh._entityId;
	_meshMVP.push_back(_wallw._entityId);

	_transformSystem.Reserve(_walln._entityId);
	_transformSystem.Identity(_walln._entityId);
	_transformSystem.SetScale(_walln._entityId, 20.0f, 5.0f, 2.0f);
	_transformSystem.SetPosition(_walln._entityId, 0.0f, 4.0f, 10.0f);
	_transformSystem.SetStatic(_walln._entityId, true);
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
			SDL_SetWindowTitle(_renderer._window, ("test_game | fps: " + std::to_string(1.0f / (float)elapsedTime.count())).c_str());
			fpsTimer = 0.0f;
		}
		Update((float)elapsedTime.count() * 0.25);
	}
}

void Game::Update(float deltaTime)
{
	MoveCamera(deltaTime, movex, movey);
	_cameraSystem.ApplyTransforms(_camera._entityId, _transformSystem.GetPosition(_camera._entityId), _transformSystem.GetRotation(_camera._entityId));
	idop::render::Clear(_renderer._renderer, glm::vec4(200, 200, 200, 255));
	DrawAxises();
	_colliderSystem.CalculateWorldBounds(_transformSystem, _meshSystem);
	std::vector<idop::CollisionInfo> cols = idop::phys::ProcessRigidbodies(_transformSystem, _rigidBodySystem, _colliderSystem, _meshSystem, deltaTime, _gravity);
	DrawCollisionPoints(cols);
	_transformSystem.CalculateMVP();
	idop::render::DrawMeshes(
		_renderer,
		_cameraSystem.GetCamera(_camera._entityId),
		_transformSystem,
		_meshSystem,
		_renderingData
	);
	SDL_RenderPresent(_renderer._renderer);
}

void Game::DrawAxises()
{
	glm::vec3 zero(0);
	constexpr float length = 3.0f;
	idop::Camera camera = _cameraSystem.GetCamera(_camera._entityId);
	idop::render::DrawVector(_renderer, camera, zero, glm::vec3(length, 0.0f, 0.0f), glm::vec4(255, 0, 0, 255));
	idop::render::DrawVector(_renderer, camera, zero, glm::vec3(0.0f, length, 0.0f), glm::vec4(0, 255, 0, 255));
	idop::render::DrawVector(_renderer, camera, zero, glm::vec3(0.0f, 0.0f, length), glm::vec4(0, 0, 255, 255));
}

void Game::DrawCollisionPoints(std::vector<idop::CollisionInfo> cols)
{
	float normalLength = 1.5f;
	glm::vec4 normalColor(255, 0, 0, 255);
	glm::vec4 tangentColor(0, 0, 255, 255);
	idop::Camera camera = _cameraSystem.GetCamera(_camera._entityId);
	for (const idop::CollisionInfo& col : cols)
	{
		for (const idop::ContactPoint& point : col._contactPoints)
		{
			idop::render::DrawVector(_renderer, camera, point._worldPoint, point._normal * normalLength, normalColor);
			idop::render::DrawVector(_renderer, camera, point._worldPoint, point._tangent * normalLength, tangentColor);
		}
	}
}

void Game::DrawRbVars(uint32_t entityId)
{
	float length = 1.5f;
	glm::vec4 velColor(255, 0, 255, 255);
	idop::Camera camera = _cameraSystem.GetCamera(_camera._entityId);
	idop::render::DrawVector(_renderer, camera, _transformSystem.GetPosition(entityId), _rigidBodySystem.GetVelocity(entityId), velColor);
}

void Game::HandleEvents()
{
	SDL_Event e;
	movex = 0.0f;
	movey = 0.0f;
	while (SDL_PollEvent(&e))
	{
		switch (e.type)
		{
		case SDL_QUIT:
			_running = false;
			break;
		case SDL_KEYDOWN:
			if (e.key.keysym.sym == SDLK_LEFT)
				movex = -1.0f;
			if (e.key.keysym.sym == SDLK_RIGHT)
				movex = 1.0f;
			if (e.key.keysym.sym == SDLK_UP)
				movey = 1.0f;
			if (e.key.keysym.sym == SDLK_DOWN)
				movey = -1.0f;
			break;
		}
	}
}

void Game::MoveCamera(float deltaTime, float x, float y)
{
	const float speed = 50.0f;
	_transformSystem.Translate(_camera._entityId, x * speed * deltaTime, 0.0f, y * speed * deltaTime);
}