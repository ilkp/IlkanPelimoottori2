#pragma once
#include "glm/glm.hpp"
#include <vector>

namespace idop
{
	extern float PrimitiveShapeVertexDefaultDistance;

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

	enum class ColliderType
	{
		Box,
		Sphere,
		Mesh
	};

	struct ContactPoint
	{
		glm::vec3 _worldPoint;
		glm::vec3 _normal;
		glm::vec3 _tangent;
		float _penetrationDepth;
	};

	struct CollisionInfo
	{
		std::vector<ContactPoint> _contactPoints;
		uint32_t _entityIdA;
		uint32_t _entityIdB;
	};

	struct RbConstraints
	{
		bool _freezeRotationX;
		bool _freezeRotationY;
		bool _freezeRotationZ;
	};
}