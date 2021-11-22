#include "ComponentData.h"

namespace idop
{
	void TransformData::Allocate(uint32_t length)
	{
		_reserved = new bool[length];
		_isStatic = new bool[length];
		_scale = new glm::mat4[length];
		_translation = new glm::mat4[length];
		_quaternion = new glm::quat[length];
		_modelViewProjection = new glm::mat4[length];
	}
	void TransformData::DeAllocate()
	{
		delete[](_reserved);
		delete[](_isStatic);
		delete[](_scale);
		delete[](_translation);
		delete[](_quaternion);
		delete[](_modelViewProjection);
	}
	void TransformData::Identity(uint32_t index)
	{
		_isStatic[index] = false;
		_scale[index] = glm::mat4(1.0f);
		_translation[index] = glm::mat4(1.0f);
		_quaternion[index] = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
		_modelViewProjection[index] = glm::mat4(1.0f);
	}

	void CameraData::Allocate(uint32_t length)
	{
		_reserved = new bool[length];
		_projectionMatrix = new glm::mat4[length];
		_viewMatrix = new glm::mat4[length];
	}
	void CameraData::DeAllocate()
	{
		delete[](_reserved);
		delete[](_projectionMatrix);
		delete[](_viewMatrix);
	}
	void CameraData::Identity(uint32_t index)
	{
		_projectionMatrix[index] = glm::perspective(
			glm::radians(45.0f),
			4.0f / 3.0f,
			0.1f,
			100.0f);
		_viewMatrix[index] = glm::lookAt(
			glm::vec3(20.0f, 20.0f, 0.0f),
			glm::vec3(0.0f, 0.0f, 0.0f),
			glm::vec3(0.0f, 1.0f, 0.0f));
	}

	void RigidbodyData::Allocate(uint32_t length)
	{
		_reserved = new bool[length];
		_useGravity = new bool[length];
		_mass = new float[length];
		_momentOfInertia = new float[length];
		_velocity = new glm::vec3[length];
		_acceleration = new glm::vec3[length];
		_angularVelocity = new glm::vec3[length];
	}
	void RigidbodyData::DeAllocate()
	{
		delete[](_reserved);
		delete[](_useGravity);
		delete[](_mass);
		delete[](_momentOfInertia);
		delete[](_velocity);
		delete[](_acceleration);
		delete[](_angularVelocity);
	}
	void RigidbodyData::Identity(uint32_t index)
	{
		_mass[index] = 1.0f;
		_momentOfInertia[index] = 0.1f;
		_velocity[index].x = 0.0f;
		_velocity[index].y = 0.0f;
		_velocity[index].z = 0.0f;
		_acceleration[index].x = 0.0f;
		_acceleration[index].y = 0.0f;
		_acceleration[index].z = 0.0f;
		_angularVelocity[index].x = 0.0f;
		_angularVelocity[index].y = 0.0f;
		_angularVelocity[index].z = 0.0f;
	}

	void MeshData::Allocate(uint32_t length)
	{
		_length = length;
		_reserved = new bool[length];
		_verticesLength = new int[length];
		_trianglesLength = new int[length];
		_uvsLength = new int[length];
		_vertices = new glm::vec3 * [length] { nullptr };
		_triangles = new int* [length] { nullptr };
		_uvs = new glm::vec2 * [length] { nullptr };
		_bounds = new Bounds3D[length];
	}
	void MeshData::DeAllocate()
	{
		for (int i = 0; i < _length; ++i)
		{
			delete[](_vertices[i]);
			delete[](_triangles[i]);
			delete[](_uvs[i]);
		}
		delete[](_reserved);
		delete[](_verticesLength);
		delete[](_trianglesLength);
		delete[](_uvsLength);
		delete[](_bounds);
	}
	void MeshData::Identity(uint32_t index)
	{
		if (_vertices[index])
		{
			delete[](_vertices[index]);
			delete[](_triangles[index]);
			delete[](_uvs[index]);
		}
		_vertices[index] = nullptr;
		_triangles[index] = nullptr;
		_uvs[index] = nullptr;
		_verticesLength[index] = 0;
		_trianglesLength[index] = 0;
		_uvsLength[index] = 0;
		_bounds[index]._min.x = 0.0f;
		_bounds[index]._min.y = 0.0f;
		_bounds[index]._min.z = 0.0f;
		_bounds[index]._max.x = 0.0f;
		_bounds[index]._max.y = 0.0f;
		_bounds[index]._max.z = 0.0f;
	}

	void ColliderData::Allocate(uint32_t length)
	{
		_reserved = new bool[length];
		_colliderType = new ColliderType[length];
		_colliderGroupId = new uint32_t[length];
		_transformId = new uint32_t[length];
		_meshId = new uint32_t[length];
		_worldBounds = new Bounds3D[length];
	}
	void ColliderData::DeAllocate()
	{
		delete[](_reserved);
		delete[](_colliderType);
		delete[](_colliderGroupId);
		delete[](_transformId);
		delete[](_meshId);
		delete[](_worldBounds);
	}
	void ColliderData::Identity(uint32_t index)
	{
		_colliderType[index] = ColliderType::Box;
		_colliderGroupId[index] = 0;
		_transformId[index] = 0;
		_worldBounds[index]._min.x = 0.0f;
		_worldBounds[index]._min.y = 0.0f;
		_worldBounds[index]._min.z = 0.0f;
		_worldBounds[index]._max.x = 0.0f;
		_worldBounds[index]._max.y = 0.0f;
		_worldBounds[index]._max.z = 0.0f;
	}
}