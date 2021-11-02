#pragma once
#include <unordered_map>
#include <type_traits>
#include <glm/glm.hpp>
#include <glm/ext/matrix_clip_space.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <math.h>
#include <iostream>
#include <bitset>

#include "Entity.h"

namespace idop
{
	template<class T>
	class ComponentSystem
	{
	public:
		virtual void Reserve(uint32_t id) = 0;
		virtual void Reserve(EntityBlock entityBlock) = 0;
		virtual void Release(uint32_t id) = 0;
		virtual void Identity(uint32_t id) = 0;
		virtual size_t Size() const = 0;

		void Print()
		{
			std::cout << seqLength << std::endl;
			std::cout << seqIndexLength << std::endl;
			std::bitset<32> a(seqIndexBits);
			std::cout << seqIndexBits << "\t" << a << std::endl;
			std::bitset<32> b(componentIndexBits);
			std::cout << componentIndexBits << "\t\t" << b << std::endl;
		}

	protected:
		uint32_t seqBytes = 256 * 1000;
		uint32_t seqLength = seqBytes / sizeof(int);
		uint32_t seqIndexLength = (uint32_t)std::ceil(std::log2((double)seqLength) + 1.0);
		uint32_t componentIndexBits = ~(uint32_t(0)) >> seqIndexLength;
		uint32_t seqIndexBits = ~componentIndexBits;
		std::unordered_map<uint32_t, T> _componentData;
	};

	struct ComponentData
	{
		bool* _reserved;
		virtual void Allocate(uint32_t length) = 0;
		virtual void DeAllocate() = 0;
	};

	struct TransformData : public ComponentData
	{
		bool* _isStatic;
		glm::vec3* _scale;
		glm::vec3* _position;
		glm::quat* _quaternion;

		void Allocate(uint32_t length)
		{
			_reserved = new bool[length];
			_isStatic = new bool[length];
			_scale = new glm::vec3[length];
			_position = new glm::vec3[length];
			_quaternion = new glm::quat[length];
		}

		void DeAllocate()
		{

			delete[](_reserved);
			delete[](_isStatic);
			delete[](_scale);
			delete[](_position);
			delete[](_quaternion);
		}
	};

	struct CameraData : public ComponentData
	{
		glm::mat4* _projectionMatrix;
		glm::mat4* _viewMatrix;

		void Allocate(uint32_t length)
		{
			_projectionMatrix = new glm::mat4[length];
			_viewMatrix = new glm::mat4[length];
		}

		void Release()
		{
			delete[](_projectionMatrix);
			delete[](_viewMatrix);
		}

		//void Identity(uint32_t index)
		//{
		//	_projectionMatrix[index] = glm::perspective(
		//		glm::radians(45.0f),
		//		4.0f / 3.0f,
		//		0.1f,
		//		100.0f);
		//	_viewMatrix[index] = glm::lookAt(
		//		glm::vec3(0.0f, 0.0f, 20.0f),
		//		glm::vec3(0.0f, 0.0f, 0.0f),
		//		glm::vec3(0.0f, 1.0f, 0.0f));
		//}
	};

	struct RigidbodyData : public ComponentData
	{
		float* _mass;
		float* _momentOfInertia;
		glm::vec3* _velocity;
		glm::vec3* _acceleration;
		glm::vec3* _angularVelocity;

		void Allocate(uint32_t length)
		{
			_reserved = new bool[length];
			_mass = new float[length];
			_momentOfInertia = new float[length];
			_velocity = new glm::vec3[length];
			_acceleration = new glm::vec3[length];
			_angularVelocity = new glm::vec3[length];
		}

		void Release()
		{
			delete[](_reserved);
			delete[](_mass);
			delete[](_momentOfInertia);
			delete[](_velocity);
			delete[](_acceleration);
			delete[](_angularVelocity);
		}

		//void Identity(uint32_t index)
		//{
		//	_mass[index] = 1.0f;
		//	_momentOfInertia[index] = 0.1f;
		//	_velocity[index].x = 0.0f;
		//	_velocity[index].y = 0.0f;
		//	_velocity[index].z = 0.0f;
		//	_acceleration[index].x = 0.0f;
		//	_acceleration[index].y = 0.0f;
		//	_acceleration[index].z = 0.0f;
		//	_angularVelocity[index].x = 0.0f;
		//	_angularVelocity[index].y = 0.0f;
		//	_angularVelocity[index].z = 0.0f;
		//}
	};

	struct MeshData : public ComponentData
	{
		int _length;
		int* _verticesLength;
		int* _trianglesLength;
		int* _uvsLength;
		glm::vec3** _vertices;
		int** _triangles;
		int** _uvs;

		void Allocate(uint32_t length)
		{
			_length = length;
			_reserved = new bool[length];
			_verticesLength = new int[length];
			_trianglesLength = new int[length];
			_uvsLength = new int[length];
			_vertices = new glm::vec3*[length];
			_triangles = new int*[length];
			_uvs = new int*[length];
		}

		void Release()
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
		}

		//void Identity(uint32_t index)
		//{
		//	delete[](_vertices[index]);
		//	delete[](_triangles[index]);
		//	delete[](_uvs[index]);
		//	_verticesLength[index] = 0;
		//	_trianglesLength[index] = 0;
		//	_uvsLength[index] = 0;
		//}
	};

	struct ModelViewData : public ComponentData
	{
		glm::mat4* _modelMatrix;

		void Allocate(uint32_t length)
		{
			_reserved = new bool[length];
			_modelMatrix = new glm::mat4[length];
		}

		void Release()
		{
			delete[](_reserved);
			delete[](_modelMatrix);
		}
	};

	struct ColliderData : public ComponentData
	{
		//TransformData* m_transform = nullptr;
		//RigidbodyData
	};

	class TransformSystem : public ComponentSystem<TransformData>
	{
	public:
		TransformSystem();

		void Reserve(uint32_t id)
		{
			auto it = _componentData.find(id & seqIndexBits);
			if (it == _componentData.end())
			{
				it = _componentData.insert({ id & seqIndexBits, TransformData() }).first;
				(*it).second.Allocate(seqLength);
			}
			(*it).second._reserved[id & componentIndexBits] = true;
		}

		void Reserve(EntityBlock entityBlock)
		{
		}

		void Release(uint32_t id)
		{
			std::unordered_map<uint32_t, TransformData>::iterator it = _componentData.find(id & seqIndexBits);
			if (it != _componentData.end())
				(*it).second._reserved[id & componentIndexBits] = false;
		}

		std::unordered_map<uint32_t, TransformData>::iterator NCReserve(uint32_t id)
		{
			std::unordered_map<uint32_t, TransformData>::iterator it = _componentData.insert({ id & seqIndexBits, TransformData() }).first;
			(*it).second.Allocate(seqLength);
			(*it).second._reserved[id & componentIndexBits] = true;
			return it;
		}

		void Identity(uint32_t id)
		{
			uint32_t componentIndex = id & componentIndexBits;
			std::unordered_map<uint32_t, TransformData>::iterator it = _componentData.find(id & seqIndexBits);
			if (it == _componentData.end())
				it = NCReserve(id);
			TransformData& cData = (*it).second;
			cData._isStatic[componentIndex] = false;
			cData._scale[componentIndex].x = 1.0f;
			cData._scale[componentIndex].y = 1.0f;
			cData._scale[componentIndex].z = 1.0f;
			cData._position[componentIndex].x = 0.0f;
			cData._position[componentIndex].y = 0.0f;
			cData._position[componentIndex].z = 0.0f;
			cData._quaternion[componentIndex] = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
		}

		size_t Size() const { return sizeof(glm::quat); }

		glm::vec3 Position(TransformData& entity);
		void SetScale(TransformData& entity, const glm::vec3& vector);
		void SetScale(TransformData& entity, float x, float y, float z);
		void SetScale(TransformData& entity, float scale);
		void SetPosition(TransformData& entity, const glm::vec3& vector);
		void SetPosition(TransformData& entity, float x, float y, float z);
		void Translate(TransformData& entity, const glm::vec3& vector);
		void Translate(TransformData& entity, float x, float y, float z);
		void Rotate(TransformData& entity, const glm::vec3& eulerAngles);
		void Rotate(TransformData& entity, float x, float y, float z);
	};
}