#pragma once
#include <cstdint>
#include <iostream>
#include "Common.h"
#include "glm/glm.hpp"
#include "glm/gtc/quaternion.hpp"
#include <unordered_map>

namespace idop
{
	struct IComponentData
	{
		virtual void Allocate(uint32_t length) = 0;
		virtual void DeAllocate() = 0;
		virtual void Identity(uint32_t index) = 0;
	};

	struct TransformData : public IComponentData
	{
		static const size_t MAX_COMP_SIZE = sizeof(glm::mat4);
		bool* _reserved = nullptr;
		bool* _isStatic = nullptr;
		glm::mat4* _scale = nullptr;
		glm::mat4* _translation = nullptr;
		glm::quat* _quaternion = nullptr;
		glm::mat4* _modelViewProjection = nullptr;

		void Allocate(uint32_t length) override;
		void DeAllocate() override;
		void Identity(uint32_t index) override;
	};

	struct CameraData : public IComponentData
	{
		static const size_t MAX_COMP_SIZE = sizeof(glm::mat4);
		bool* _reserved = nullptr;
		glm::mat4* _projectionMatrix = nullptr;
		glm::mat4* _viewMatrix = nullptr;

		void Allocate(uint32_t length) override;
		void DeAllocate() override;
		void Identity(uint32_t index) override;
	};

	struct RigidbodyData : public IComponentData
	{
		static const size_t MAX_COMP_SIZE = sizeof(glm::vec3);
		bool* _reserved = nullptr;
		bool* _useGravity = nullptr;
		RbConstraints* _constraints = nullptr;
		float* _mass = nullptr;
		float* _momentOfInertia = nullptr;
		float* _dragCoefficient = nullptr;
		float* _coefficientOfRestitution = nullptr;
		float* _staticFriction = nullptr;
		float* _dynamicFriction = nullptr;
		glm::vec3* _velocity = nullptr;
		glm::vec3* _acceleration = nullptr;
		glm::vec3* _angularVelocity = nullptr;

		void Allocate(uint32_t length) override;
		void DeAllocate() override;
		void Identity(uint32_t index) override;
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

		void Allocate(uint32_t length) override;
		void DeAllocate() override;
		void Identity(uint32_t index) override;
	};

	struct ColliderData : public IComponentData
	{
		static const size_t MAX_COMP_SIZE = sizeof(Bounds3D);
		bool* _reserved = nullptr;
		ColliderType* _colliderType = nullptr;
		glm::mat4* _scale = nullptr;
		glm::mat4* _offset = nullptr;
		uint32_t* _colliderGroupId = nullptr;
		uint32_t* _meshId = nullptr;
		Bounds3D* _worldBounds = nullptr;

		void Allocate(uint32_t length) override;
		void DeAllocate() override;
		void Identity(uint32_t index) override;
	};

	struct RenderingData
	{
		std::unordered_map<uint32_t, std::vector<uint32_t>> _meshMVP;
	};
}