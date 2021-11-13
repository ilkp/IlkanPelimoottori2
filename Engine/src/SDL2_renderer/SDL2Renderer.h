#pragma once
#include <SDL.h>
#include "ComponentSystem.h"
#include "ComponentData.h"

namespace idop
{
	class SDL2Renderer
	{
	public:
		SDL2Renderer(TransformSystem* transformSystem, MeshSystem* meshSystem, CameraSystem* cameraSystem) :
			_transformSystem(transformSystem), _meshSystem(meshSystem), _cameraSystem(cameraSystem) {}
		int Init(int width, int height);
		void Clean();
		void Render(uint32_t cameraEntityId, const RenderingData& renderingData);

	private:
		TransformSystem* _transformSystem = nullptr;;
		MeshSystem* _meshSystem = nullptr;
		CameraSystem* _cameraSystem = nullptr;
		SDL_Window* _window = nullptr;
		SDL_Renderer* _renderer = nullptr;
		glm::vec4 _viewport;
	};
}