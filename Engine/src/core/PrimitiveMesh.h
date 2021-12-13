#pragma once
#include "Common.h"

namespace idop::primitives
{
	static Mesh Cube()
	{
		Mesh mesh;
		const float pointDist = PrimitiveShapeVertexDefaultDistance;
		mesh._verticesLength = 8;
		mesh._trianglesLength = 12 * 3;
		mesh._uvsLength = 8;
		mesh._vertices = new glm::vec3[]
		{
			glm::vec3(-pointDist, -pointDist, -pointDist),
			glm::vec3(pointDist, -pointDist, -pointDist),
			glm::vec3(pointDist, -pointDist, pointDist),
			glm::vec3(-pointDist, -pointDist, pointDist),

			glm::vec3(-pointDist, pointDist, -pointDist),
			glm::vec3(pointDist, pointDist, -pointDist),
			glm::vec3(pointDist, pointDist, pointDist),
			glm::vec3(-pointDist, pointDist, pointDist),
		};
		mesh._triangles = new int[]
		{
			0, 4, 5,
				0, 5, 1,
				1, 5, 6,
				1, 6, 2,
				2, 6, 7,
				2, 7, 3,
				3, 7, 4,
				3, 4, 0,
				4, 7, 6,
				4, 6, 5,
				3, 0, 1,
				3, 1, 2
		};
		mesh._uvs = new glm::vec2[]
		{
			glm::vec2(0.0f, 0.0f),
			glm::vec2(0.0f, 0.0f),
			glm::vec2(0.0f, 0.0f),
			glm::vec2(0.0f, 0.0f),
			glm::vec2(0.0f, 0.0f),
			glm::vec2(0.0f, 0.0f),
			glm::vec2(0.0f, 0.0f),
			glm::vec2(0.0f, 0.0f)
		};
		mesh._bounds._min.x = -pointDist;
		mesh._bounds._min.y = -pointDist;
		mesh._bounds._min.z = -pointDist;
		mesh._bounds._max.x = pointDist;
		mesh._bounds._max.y = pointDist;
		mesh._bounds._max.z = pointDist;
		return mesh;
	}
}