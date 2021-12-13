#pragma once
#include <thread>
#include "SDL2Renderer.h"
#include "EntitySystem.h"
#include "Entity.h"
#include "ctpl_stl.h"

class Game
{
public:
	bool Init();
	void Start();
	bool IsRunning() { return _running; }

private:
	bool _running = true;
	float _timeSinceStart = 0.0f;
	ctpl::thread_pool _threadPool = ctpl::thread_pool(std::thread::hardware_concurrency());
	glm::vec3 _gravity = glm::vec3(0.0f, -0.1f, 0.0f);
	std::vector<idop::Entity> _cubes;
	std::vector<uint32_t> _meshMVP;

	idop::EntitySystem _entitySystem;
	idop::MeshSystem _meshSystem = idop::MeshSystem(32 * 1000);
	idop::TransformSystem _transformSystem = idop::TransformSystem(32 * 1000);
	idop::CameraSystem _cameraSystem = idop::CameraSystem(32 * 1000);
	idop::ColliderSystem _colliderSystem = idop::ColliderSystem(32 * 1000, { &_transformSystem });
	idop::RigidbodySystem _rigidBodySystem = idop::RigidbodySystem(32 * 1000, { &_transformSystem });

	idop::SDL2Renderer _SDLrenderer = idop::SDL2Renderer(&_transformSystem, &_meshSystem, &_cameraSystem);
	idop::RenderingData _renderingData;

	idop::Entity _cube = idop::Entity(_entitySystem.Next());
	idop::Entity _camera = idop::Entity(_entitySystem.Next());
	idop::Entity _cubeMesh = idop::Entity(_entitySystem.Next());
	idop::Entity _collider = idop::Entity(_entitySystem.Next());
	idop::Entity _floor = idop::Entity(_entitySystem.Next());
	idop::Entity _walls = idop::Entity(_entitySystem.Next());
	idop::Entity _walle = idop::Entity(_entitySystem.Next());
	idop::Entity _walln = idop::Entity(_entitySystem.Next());
	idop::Entity _wallw = idop::Entity(_entitySystem.Next());

	void UpdateCubes(float deltaTime);
	void UpdateCubesParallel(float deltaTime);
	void Update(float deltaTime);
	void HandleEvents();
};
