#pragma once
#include <SDL.h>
#include "ComponentSystem.h"
#include "ComponentData.h"

namespace idop::render
{
	struct Renderer
	{
		SDL_Window* _window = nullptr;
		SDL_Renderer* _renderer = nullptr;
		glm::vec4 _viewport;
	};

	inline int Init(Renderer& renderer, int width, int height)
	{
		if (SDL_Init(SDL_INIT_VIDEO) != 0)
		{
			fprintf(stderr, "Failed to initialize SDL2.0: %s\n", SDL_GetError());
			return EXIT_FAILURE;
		}

		renderer._window = SDL_CreateWindow("Game", 400, 200, width, height, SDL_WINDOW_SHOWN);
		if (renderer._window == nullptr)
		{
			fprintf(stderr, "Failed to create SDL2.0 window: %s\n", SDL_GetError());
			return EXIT_FAILURE;
		}

		renderer._renderer = SDL_CreateRenderer(renderer._window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
		if (renderer._renderer == nullptr)
		{
			SDL_DestroyWindow(renderer._window);
			fprintf(stderr, "Failed to create SDL2.0 renderer: %s\n", SDL_GetError());
			return EXIT_FAILURE;
		}
		renderer._viewport = glm::vec4(0.0f, 0.0f, width, height);
		return 0;
	}

	inline void Clear(SDL_Renderer* renderer, const glm::vec4& color)
	{
		SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
		SDL_RenderClear(renderer);
	}

	inline void DrawVector(
		Renderer& renderer,
		const Camera& camData,
		const glm::vec3& start,
		const glm::vec3& vector,
		const glm::vec4& color)
	{
		Uint8 r, g, b, a;
		glm::vec3 pStart = glm::project(start, camData._viewMatrix, camData._projectionMatrix, renderer._viewport);
		glm::vec3 pEnd = glm::project(start + vector, camData._viewMatrix, camData._projectionMatrix, renderer._viewport);
		SDL_GetRenderDrawColor(renderer._renderer, &r, &g, &b, &a);
		SDL_SetRenderDrawColor(renderer._renderer, color.r, color.g, color.b, color.a);
		SDL_RenderDrawLine(renderer._renderer, pStart.x, pStart.y, pEnd.x, pEnd.y);
		SDL_SetRenderDrawColor(renderer._renderer, r, g, b, a);
	}

	inline void DrawMeshes(
		Renderer& renderer,
		const Camera& camera,
		const TransformSystem& tSystem,
		const MeshSystem& mSystem,
		const RenderingData& renderingData)
	{
		const MeshData* meshData;
		const int* triangles;
		SDL_SetRenderDrawColor(renderer._renderer, 0, 0, 0, 255);
		std::vector<glm::vec3> projectedPoints;

		for (auto const& [meshId, mvpVec] : renderingData._meshMVP)
		{
			meshData = &mSystem._componentData.at(meshId & mSystem.INDEX_BITS_SEQ);
			if (!meshData->_reserved[meshId & mSystem.INDEX_BITS_COMP])
				continue;
			projectedPoints.clear();
			projectedPoints.resize(meshData->_verticesLength[meshId & mSystem.INDEX_BITS_COMP]);

			for (auto const& mvpId : mvpVec)
			{
				for (int i = 0; i < projectedPoints.size(); ++i)
				{
					projectedPoints[i] = glm::project(
						meshData->_vertices[meshId & mSystem.INDEX_BITS_COMP][i],
						camera._viewMatrix * tSystem._componentData.at(mvpId & tSystem.INDEX_BITS_SEQ)._modelViewProjection[mvpId & tSystem.INDEX_BITS_COMP],
						camera._projectionMatrix,
						renderer._viewport);
				}

				triangles = meshData->_triangles[meshId & mSystem.INDEX_BITS_COMP];
				for (int i = 0; i < meshData->_trianglesLength[meshId & mSystem.INDEX_BITS_COMP]; i += 3)
				{
					SDL_RenderDrawLine(renderer._renderer,
						projectedPoints[triangles[i]].x,
						projectedPoints[triangles[i]].y,
						projectedPoints[triangles[i + 1]].x,
						projectedPoints[triangles[i + 1]].y);
					SDL_RenderDrawLine(renderer._renderer,
						projectedPoints[triangles[i + 1]].x,
						projectedPoints[triangles[i + 1]].y,
						projectedPoints[triangles[i + 2]].x,
						projectedPoints[triangles[i + 2]].y);
					SDL_RenderDrawLine(renderer._renderer,
						projectedPoints[triangles[i + 2]].x,
						projectedPoints[triangles[i + 2]].y,
						projectedPoints[triangles[i]].x,
						projectedPoints[triangles[i]].y);
				}
			}
		}
	}
}