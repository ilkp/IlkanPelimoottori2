#include "SDL2Renderer.h"
#include <iostream>

int idop::SDL2Renderer::Init(int width, int height)
{
	if (SDL_Init(SDL_INIT_VIDEO) != 0)
	{
		fprintf(stderr, "Failed to initialize SDL2.0: %s\n", SDL_GetError());
		return EXIT_FAILURE;
	}

	_window = SDL_CreateWindow("Game", 400, 200, width, height, SDL_WINDOW_SHOWN);
	if (_window == nullptr)
	{
		fprintf(stderr, "Failed to create SDL2.0 window: %s\n", SDL_GetError());
		return EXIT_FAILURE;
	}

	_renderer = SDL_CreateRenderer(_window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
	if (_renderer == nullptr)
	{
		SDL_DestroyWindow(_window);
		fprintf(stderr, "Failed to create SDL2.0 renderer: %s\n", SDL_GetError());
		return EXIT_FAILURE;
	}
	_viewport = glm::vec4(0.0f, 0.0f, width, height);
	return 0;
}

void idop::SDL2Renderer::Clean()
{
	SDL_DestroyRenderer(_renderer);
	SDL_DestroyWindow(_window);
}

void idop::SDL2Renderer::Render(uint32_t cameraEntityId, const RenderingData& renderingData)
{
	const MeshData* meshData;
	const CameraData& cameraData = _cameraSystem->_componentData.at(cameraEntityId & _cameraSystem->INDEX_BITS_SEQ);
	const int* triangles;
	SDL_SetRenderDrawColor(_renderer, 200, 200, 200, 255);
	SDL_RenderClear(_renderer);
	SDL_SetRenderDrawColor(_renderer, 0, 0, 0, 255);
	std::vector<glm::vec3> projectedPoints;

	for (auto const& [meshId, mvpVec] : renderingData._meshMVP)
	{
		meshData = &_meshSystem->_componentData.at(meshId & _meshSystem->INDEX_BITS_SEQ);
		if (!meshData->_reserved[meshId & _meshSystem->INDEX_BITS_COMP])
			continue;
		projectedPoints.clear();
		projectedPoints.resize(meshData->_verticesLength[meshId & _meshSystem->INDEX_BITS_COMP]);

		for (auto const& mvpId : mvpVec)
		{
			for (int i = 0; i < projectedPoints.size(); ++i)
			{
				projectedPoints[i] = glm::project(
					meshData->_vertices[meshId & _meshSystem->INDEX_BITS_COMP][i],
					cameraData._viewMatrix[cameraEntityId & _cameraSystem->INDEX_BITS_COMP] * _transformSystem->_componentData.at(mvpId & _transformSystem->INDEX_BITS_SEQ)._modelViewProjection[mvpId & _transformSystem->INDEX_BITS_COMP],
					cameraData._projectionMatrix[cameraEntityId & _cameraSystem->INDEX_BITS_COMP],
					_viewport);
			}

			triangles = meshData->_triangles[meshId & _meshSystem->INDEX_BITS_COMP];
			for (int i = 0; i < meshData->_trianglesLength[meshId & _meshSystem->INDEX_BITS_COMP]; i += 3)
			{
				SDL_RenderDrawLine(_renderer,
					projectedPoints[triangles[i]].x,
					projectedPoints[triangles[i]].y,
					projectedPoints[triangles[i + 1]].x,
					projectedPoints[triangles[i + 1]].y);
				SDL_RenderDrawLine(_renderer,
					projectedPoints[triangles[i + 1]].x,
					projectedPoints[triangles[i + 1]].y,
					projectedPoints[triangles[i + 2]].x,
					projectedPoints[triangles[i + 2]].y);
				SDL_RenderDrawLine(_renderer,
					projectedPoints[triangles[i + 2]].x,
					projectedPoints[triangles[i + 2]].y,
					projectedPoints[triangles[i]].x,
					projectedPoints[triangles[i]].y);
			}
		}
	}
	RenderAxises(cameraEntityId);
	SDL_RenderPresent(_renderer);
}

void idop::SDL2Renderer::RenderAxises(uint32_t cameraEntityId)
{
	//const CameraData* cameraData = &_cameraSystem->_componentData.at(cameraEntityId & _cameraSystem->INDEX_BITS_SEQ);
}