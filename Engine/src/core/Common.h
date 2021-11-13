#pragma once
#include "glm/glm.hpp"

namespace idop
{
	struct Bounds3D
	{
		glm::vec3 _min;
		glm::vec3 _max;
	};

	struct Bounds2D
	{
		glm::vec2 _min;
		glm::vec2 _max;
	};

	struct Mesh
	{
		int _verticesLength;
		int _trianglesLength;
		int _uvsLength;
		glm::vec3* _vertices;
		int* _triangles;
		glm::vec2* _uvs;
		Bounds3D _bounds;
	};

	struct Camera
	{
		glm::mat4 _projectionMatrix;
		glm::mat4 _viewMatrix;
	};
}