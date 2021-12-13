#pragma once
#include <unordered_map>
#include <glm/glm.hpp>
#include <glm/ext/matrix_clip_space.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include "ctpl_stl.h"

#include "Entity.h"
#include "Common.h"
#include "ComponentData.h"


namespace idop
{
	static uint32_t ComponentIndexBits(uint32_t seqBytes, uint32_t maxCompSize)
	{
		return (1 << (uint32_t)(std::log2((double)(seqBytes / maxCompSize)) + 1.0)) - 1;
	}

	class IComponentSystem
	{
	public:
		// Reserve will allocate data, if one for the entityId doesn't exist
		// Reserve will do nothing, if entityId is already marked as in use
		virtual void Reserve(uint32_t entityId) = 0;

		// Release will only mark data correspoding to entityId as not in use
		virtual void Release(uint32_t entityId) = 0;

		// Will reset data of entity.
		// Function will allocate data, if it wasn't allocated
		virtual void Identity(uint32_t entityId) = 0;
	};

	template <class T>
	class ComponentSystem : public IComponentSystem
	{
	public:
		std::unordered_map<uint32_t, T> _componentData;
		const uint32_t SEQ_BYTES = 32 * 1000;
		const uint32_t INDEX_BITS_COMP = ComponentIndexBits(SEQ_BYTES, T::MAX_COMP_SIZE);
		const uint32_t INDEX_BITS_SEQ = ~INDEX_BITS_COMP;

		ComponentSystem(uint32_t seqBytes) : SEQ_BYTES(seqBytes) {}
		ComponentSystem(uint32_t seqBytes, std::initializer_list<IComponentSystem*> dependencies) : SEQ_BYTES(seqBytes), _dependencies{dependencies} {}

		void Reserve(uint32_t entityId) override
		{
			typename std::unordered_map<uint32_t, T>::iterator it = _componentData.find(entityId & INDEX_BITS_SEQ);
			if (it == _componentData.end())
			{
				it = _componentData.insert(std::make_pair(entityId & INDEX_BITS_SEQ, T())).first;
				it->second.Allocate(INDEX_BITS_COMP + 1);
			}
			it->second._reserved[entityId & INDEX_BITS_COMP] = true;
			for (IComponentSystem* system : _dependencies)
				system->Reserve(entityId);
		}
		void Release(uint32_t entityId) override
		{
			typename std::unordered_map<uint32_t, T>::iterator it = _componentData.find(entityId & INDEX_BITS_SEQ);
			if (it != _componentData.end())
				it->second._reserved[entityId & INDEX_BITS_COMP] = false;
		}
		void Identity(uint32_t entityId) override
		{
			typename std::unordered_map<uint32_t, T>::iterator it = _componentData.find(entityId & INDEX_BITS_SEQ);
			if (it == _componentData.end())
				it = NoCheckReserve(entityId);
			it->second.Identity(entityId & INDEX_BITS_COMP);
		}
		void Reserve(Entity entity) { Reserve(entity._entityId); }
		void Release(Entity entity) { Release(entity._entityId); }
		void Identity(Entity entity) { Identity(entity._entityId); }

	protected:
		std::vector<IComponentSystem*> _dependencies;
		typename std::unordered_map<uint32_t, T>::iterator NoCheckReserve(uint32_t entityId)
		{
			typename std::unordered_map<uint32_t, T>::iterator it = _componentData.insert(std::make_pair(entityId & INDEX_BITS_SEQ, T())).first;
			it->second.Allocate(INDEX_BITS_COMP + 1);
			it->second._reserved[entityId & INDEX_BITS_COMP] = true;
			for (IComponentSystem* system : _dependencies)
				system->Reserve(entityId);
			return it;
		}
	};

	class TransformSystem : public ComponentSystem<TransformData>
	{
	public:
		using ComponentSystem::ComponentSystem;

		bool IsReserved(uint32_t entityId) const;
		bool IsReserved(Entity entity) const { return IsReserved(entity._entityId); };
		bool IsStatic(uint32_t entityId) const;
		bool IsStatic(Entity entity) const { return IsStatic(entity._entityId); };
		
		void CalculateMVP();
		void Translate(uint32_t entityId, const glm::vec3& vector);
		void Translate(uint32_t entityId, float x, float y, float z);
		void SetScale(uint32_t entityId, const glm::vec3& vector);
		void SetScale(uint32_t entityId, float x, float y, float z);
		void SetPosition(uint32_t entityId, const glm::vec3& vector);
		void SetPosition(uint32_t entityId, float x, float y, float z);
		void SetRotation(uint32_t entityId, const glm::quat& rotation);
		void SetRotation(uint32_t entityId, const glm::mat4& rotation);
		void Rotate(uint32_t entityId, const glm::vec3& eulerAngles);
		void Rotate(uint32_t entityId, float x, float y, float z);

		glm::vec3 GetPosition(uint32_t entityId) const;
		glm::mat4 GetPositionMat(uint32_t entityId) const;
		glm::vec3 GetScaleVector(uint32_t entityId) const;
		glm::quat GetRotation(uint32_t entityId) const;
		bool GetIsStatic(uint32_t entityId) const;

	private:
		ctpl::thread_pool _threadPool = ctpl::thread_pool(std::thread::hardware_concurrency());
		void CalculateMVPSeq(TransformData* tData, uint32_t start, uint32_t end);
	};

	class RigidbodySystem : public ComponentSystem<RigidbodyData>
	{
	public:
		using ComponentSystem::ComponentSystem;

		void SetMass(uint32_t entityId, float mass);
		void SetMomentOfInertia(uint32_t entityId, float inertia);
		void SetVelocity(uint32_t entityId, const glm::vec3& velocity);
		void SetVelocity(uint32_t entityId, float x, float y, float z);
		void SetAcceleration(uint32_t entityId, const glm::vec3& acceleration);
		void SetAcceleration(uint32_t entityId, float x, float y, float z);
		void SetAngularVelocity(uint32_t entityId, const glm::vec3& angularVelocity);
		void SetAngularVelocity(uint32_t entityId, float x, float y, float z);
		void SetUseGravity(uint32_t entityId, bool v) { _componentData.at(entityId & INDEX_BITS_SEQ)._useGravity[entityId & INDEX_BITS_COMP] = v; }

		bool GetReserved(uint32_t entityId) const { return _componentData.at(entityId & INDEX_BITS_SEQ)._reserved[entityId & INDEX_BITS_COMP]; }
		float GetCoef(uint32_t entityId) const {return _componentData.at(entityId & INDEX_BITS_SEQ)._coefficientOfRestitution[entityId & INDEX_BITS_COMP]; }
		float GetMass(uint32_t entityId) const { return _componentData.at(entityId & INDEX_BITS_SEQ)._mass[entityId & INDEX_BITS_COMP]; }
		float GetMomentOfInertia(uint32_t entityId) const { return _componentData.at(entityId & INDEX_BITS_SEQ)._momentOfInertia[entityId & INDEX_BITS_COMP]; }
		float GetStaticFriction(uint32_t entityId) const { return _componentData.at(entityId & INDEX_BITS_SEQ)._staticFriction[entityId & INDEX_BITS_COMP]; }
		float GetDynamicFriction(uint32_t entityId) const { return _componentData.at(entityId & INDEX_BITS_SEQ)._dynamicFriction[entityId & INDEX_BITS_COMP]; }
		glm::vec3 GetVelocity(uint32_t entityId) const { return _componentData.at(entityId & INDEX_BITS_SEQ)._velocity[entityId & INDEX_BITS_COMP]; }
		glm::vec3 GetAcceleration(uint32_t entityId) const { return _componentData.at(entityId & INDEX_BITS_SEQ)._acceleration[entityId & INDEX_BITS_COMP]; }
		glm::vec3 GetAngularVelocity(uint32_t entityId) const { return _componentData.at(entityId & INDEX_BITS_SEQ)._angularVelocity[entityId & INDEX_BITS_COMP]; }
	};

	class MeshSystem : public ComponentSystem<MeshData>
	{
	public:
		using ComponentSystem::ComponentSystem;

		void SetMesh(uint32_t entityId, const Mesh& mesh);
		void SetMesh(Entity entity, const Mesh& mesh) { SetMesh(entity._entityId, mesh); }
	};

	class CameraSystem : public ComponentSystem<CameraData>
	{
	public:
		using ComponentSystem::ComponentSystem;

		Camera GetCamera(uint32_t entityId) const;
		void ApplyTransforms(uint32_t entityId, const glm::vec3& position, const glm::quat& rotation);
		void LookAt(uint32_t entityId, const glm::vec3& position, const glm::vec3& center, const glm::vec3& up);
		void LookAt(Entity entity, const glm::vec3& position, const glm::vec3& center, const glm::vec3& up) { LookAt(entity._entityId, position, center, up); }
	};

	class ColliderSystem : public ComponentSystem<ColliderData>
	{
	public:
		using ComponentSystem::ComponentSystem;

		void CalculateWorldBounds(const TransformSystem& tSystem, const MeshSystem& mSystem);
		void SetMeshId(uint32_t entityId, uint32_t meshId);
		void SetMeshId(Entity entity, uint32_t meshId) { SetMeshId(entity._entityId, meshId); }

	private:
		void BoxPoints(glm::vec4* points);
		void CalculateWorldBoundsBox(const TransformData& tData, uint32_t tIndex, const ColliderData& cData, uint32_t cIndex);
	};
}