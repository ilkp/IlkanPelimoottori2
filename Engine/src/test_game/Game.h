#pragma once
#include "SDL2Renderer.h"
#include "EntitySystem.h"
#include "Entity.h"

class Game
{
public:
	bool Init();
	void Start();
	bool IsRunning() { return _running; }

private:
	bool _running = true;
	float _timeSinceStart = 0.0f;

	idop::EntitySystem _entitySystem;
	idop::MeshSystem _meshSystem;
	idop::TransformSystem _transformSystem;
	idop::CameraSystem _cameraSystem;
	idop::ColliderSystem _colliderSystem;
	idop::RigidbodySystem _rigidBodySystem;

	idop::SDL2Renderer _SDLrenderer = idop::SDL2Renderer(&_transformSystem, &_meshSystem, &_cameraSystem);
	idop::RenderingData _renderingData;

	idop::Entity _cubeMesh = idop::Entity(_entitySystem.Next());
	idop::Entity _camera = idop::Entity(_entitySystem.Next());
	idop::Entity _collider = idop::Entity(_entitySystem.Next());

	std::vector<idop::Entity> _cubes;

	glm::vec3 _gravity = glm::vec3(0.0f, -0.1f, 0.0f);

	void Execute();
	void Update(float deltaTime);
	void HandleEvents();
};

