#pragma once
#include <cstdint>
#include "Common.h"
#include "glm/glm.hpp"
#include "glm/gtc/quaternion.hpp"

namespace idop
{
	struct IComponentData
	{
		virtual void Allocate(uint32_t length) = 0;
		virtual void Deallocate() = 0;
	};

	struct TransformData : public IComponentData
	{
		static const size_t MAX_COMP_SIZE = sizeof(glm::mat4);
		bool* _reserved;
		bool* _isStatic;
		glm::mat4* _scale;
		glm::mat4* _translation;
		glm::quat* _quaternion;
		glm::mat4* _modelViewProjection;

		void Allocate(uint32_t length) override
		{
			_reserved = new bool[length] { false };
			_isStatic = new bool[length] { false };
			_scale = new glm::mat4[length] { glm::mat4(1.0f) };
			_translation = new glm::mat4[length] { glm::mat4(1.0f) };
			_quaternion = new glm::quat[length] { glm::quat(1.0f, 0.0, 0.0f, 0.0f) };
			_modelViewProjection = new glm::mat4[length] { glm::mat4(1.0f) };
		}

		void Deallocate() override
		{
			delete[](_reserved);
			delete[](_isStatic);
			delete[](_scale);
			delete[](_translation);
			delete[](_quaternion);
			delete[](_modelViewProjection);
		}
	};

	struct CameraData : public IComponentData
	{
		static const size_t MAX_COMP_SIZE = sizeof(glm::mat4);
		bool* _reserved;
		glm::mat4* _projectionMatrix;
		glm::mat4* _viewMatrix;

		void Allocate(uint32_t length) override
		{
			_reserved = new bool[length] {false};
			_projectionMatrix = new glm::mat4[length] { glm::mat4(1.0f) };
			_viewMatrix = new glm::mat4[length] { glm::mat4(1.0f) };
		}

		void Deallocate() override
		{
			delete[](_reserved);
			delete[](_projectionMatrix);
			delete[](_viewMatrix);
		}
	};

	struct RigidbodyData : public IComponentData
	{
		static const size_t MAX_COMP_SIZE = sizeof(glm::vec3);
		bool* _reserved;
		bool* _useGravity;
		float* _mass;
		float* _momentOfInertia;
		glm::vec3* _velocity;
		glm::vec3* _acceleration;
		glm::vec3* _angularVelocity;

		void Allocate(uint32_t length) override
		{
			_reserved = new bool[length] {false};
			_useGravity = new bool[length] {false};
			_mass = new float[length] {0.0f};
			_momentOfInertia = new float[length] {0.0f};
			_velocity = new glm::vec3[length]{glm::vec3(0.0f, 0.0f, 0.0f)};
			_acceleration = new glm::vec3[length]{glm::vec3(0.0f, 0.0f, 0.0f)};
			_angularVelocity = new glm::vec3[length]{glm::vec3(0.0f, 0.0f, 0.0f)};
		}

		void Deallocate() override
		{
			delete[](_reserved);
			delete[](_useGravity);
			delete[](_mass);
			delete[](_momentOfInertia);
			delete[](_velocity);
			delete[](_acceleration);
			delete[](_angularVelocity);
		}
	};

	struct MeshData : public IComponentData
	{
		static const size_t MAX_COMP_SIZE = sizeof(Bounds3D);
		int _length;
		bool* _reserved = nullptr;
		int* _verticesLength = nullptr;
		int* _trianglesLength = nullptr;
		int* _uvsLength = nullptr;
		glm::vec3** _vertices = nullptr;
		int** _triangles = nullptr;
		glm::vec2** _uvs = nullptr;
		Bounds3D* _bounds = nullptr;

		void Allocate(uint32_t length) override
		{
			_length = length;
			_reserved = new bool[length] {false};
			_verticesLength = new int[length];
			_trianglesLength = new int[length];
			_uvsLength = new int[length];
			_vertices = new glm::vec3 * [length] {nullptr};
			_triangles = new int* [length] {nullptr};
			_uvs = new glm::vec2 * [length] {nullptr};
			_bounds = new Bounds3D[length];
		}

		void Deallocate() override
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
	};

	struct ColliderData : public IComponentData
	{
		static const size_t MAX_COMP_SIZE = sizeof(Bounds3D);
		bool* _reserved;
		uint32_t* _colliderGroupId;
		uint32_t* _transformId;
		uint32_t* _meshId;
		Bounds3D* _worldBounds;

		void Allocate(uint32_t length) override
		{
			_reserved = new bool[length] { false };
			_colliderGroupId = new uint32_t[length] { 0 };
			_transformId = new uint32_t[length] { 0 };
			_meshId = new uint32_t[length] { 0 };
			_worldBounds = new Bounds3D[length];
		}

		void Deallocate() override
		{

			delete[](_reserved);
			delete[](_colliderGroupId);
			delete[](_transformId);
			delete[](_meshId);
			delete[](_worldBounds);
		}
	};

	struct RenderingData
	{
		std::unordered_map<uint32_t, std::vector<uint32_t>> _meshMVP;
	};
}