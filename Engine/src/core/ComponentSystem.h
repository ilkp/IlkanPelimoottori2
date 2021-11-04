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
#include "Common.h"


namespace idop
{
	struct ComponentData
	{
		bool* _reserved;
		virtual void Allocate(uint32_t length) = 0;
		virtual void DeAllocate() = 0;
	};

	template<class T>
	class ComponentSystem
	{
		static_assert(std::is_base_of<ComponentData, T>::value, "T must derive from ComponentData");
	public:
		virtual void Reserve(uint32_t id)
		{
			typename std::unordered_map<uint32_t, T>::iterator it = _componentData.find(id & seqIndexBits);
			if (it == _componentData.end())
			{
				it = _componentData.insert({ id & seqIndexBits, T() }).first;
				(*it).second.Allocate(seqLength);
			}
			(*it).second._reserved[id & componentIndexBits] = true;
		}
		virtual void Release(uint32_t id)
		{
			typename std::unordered_map<uint32_t, T>::iterator it = _componentData.find(id & seqIndexBits);
			if (it != _componentData.end())
				(*it).second._reserved[id & componentIndexBits] = false;
		}
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

	struct TransformData : public ComponentData
	{
		bool* _isStatic;
		glm::mat4* _scale;
		glm::mat4* _translation;
		glm::quat* _quaternion;

		void Allocate(uint32_t length)
		{
			_reserved = new bool[length];
			_isStatic = new bool[length];
			_scale = new glm::mat4[length];
			_translation = new glm::mat4[length];
			_quaternion = new glm::quat[length];
		}

		void DeAllocate()
		{
			delete[](_reserved);
			delete[](_isStatic);
			delete[](_scale);
			delete[](_translation);
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

		void DeAllocate()
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

		void DeAllocate()
		{
			delete[](_reserved);
			delete[](_mass);
			delete[](_momentOfInertia);
			delete[](_velocity);
			delete[](_acceleration);
			delete[](_angularVelocity);
		}
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
		Bounds3D* _bounds;

		void Allocate(uint32_t length)
		{
			_length = length;
			_reserved = new bool[length];
			_verticesLength = new int[length];
			_trianglesLength = new int[length];
			_uvsLength = new int[length];
			_vertices = new glm::vec3*[length];
			_triangles = new int* [length];
			_uvs = new int* [length];
			_bounds = new Bounds3D[length];
		}

		void DeAllocate()
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

	struct ColliderData : public ComponentData
	{
		uint32_t* _colliderGroupId;
		std::pair<TransformData*, uint32_t>* _transform;
	};

	class TransformSystem : public ComponentSystem<TransformData>
	{
	public:
		TransformSystem();
		void Reserve(uint32_t id);
		std::unordered_map<uint32_t, TransformData>::iterator NCReserve(uint32_t id)
		{
			std::unordered_map<uint32_t, TransformData>::iterator it = _componentData.insert({ id & seqIndexBits, TransformData() }).first;
			(*it).second.Allocate(seqLength);
			(*it).second._reserved[id & componentIndexBits] = true;
			return it;
		}
		void Identity(uint32_t id);
		size_t Size() const { return sizeof(glm::mat4); }

		void CalculateMVP();
		void SetScale(uint32_t entityId, const glm::vec3& vector);
		void SetScale(uint32_t entityId, float x, float y, float z);
		void SetPosition(uint32_t entityId, const glm::vec3& vector);
		void SetPosition(uint32_t entityId, float x, float y, float z);
		void Translate(uint32_t entityId, const glm::vec3& vector);
		void Translate(uint32_t entityId, float x, float y, float z);
		void Rotate(uint32_t entityId, const glm::vec3& eulerAngles);
		void Rotate(uint32_t entityId, float x, float y, float z);

		glm::vec3 Position(uint32_t entityId) const;
		glm::vec3 Scale(uint32_t entityId) const;

	private:
		std::unordered_map<uint32_t, glm::mat4*> _MVP;
	};

	class RigidbodySystem : public ComponentSystem<RigidbodyData>
	{
	public:
		RigidbodySystem();
		void Identity(uint32_t id);
		size_t Size() { return sizeof(glm::vec3); }

		void SetMass(uint32_t entityId, float mass);
		void SetMomentOfInertia(uint32_t entityId, float inertia);
		void SetVelocity(uint32_t entityId, const glm::vec3& velocity);
		void SetVelocity(uint32_t entityId, float x, float y, float z);
		void SetAcceleration(uint32_t entityId, const glm::vec3& acceleration);
		void SetAcceleration(uint32_t entityId, float x, float y, float z);
		void SetAngularVelocity(uint32_t entityId, const glm::vec3& angularVelocity);
		void SetAngularVelocity(uint32_t entityId, float x, float y, float z);
	};
}