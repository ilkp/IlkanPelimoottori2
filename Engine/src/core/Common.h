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
}