#pragma once
#include <unordered_map>
#include <type_traits>
#include <glm/glm.hpp>
#include <glm/ext/matrix_clip_space.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include "Entity.h"
#include "Common.h"
#include "ComponentData.h"
#include "ctpl_stl.h"


namespace idop
{
	class IComponentSystem
	{
	public:
		// Reserve will allocate data, if one for the entityId doesn't exist
		// Reserve will do nothing, if entityId is already marked as in use
		virtual void Reserve(uint32_t entityId) = 0;

		// Release will only mark data correspoding to entityId as not in use
		virtual void Release(uint32_t entityId) = 0;

		// Identity will return data correspoding to entityId to default state and deletes pointer members
		// If data corresponding to entityId doesn't exist, it will be reserved
		virtual void Identity(uint32_t entityId) = 0;
	};

	class TransformSystem : public IComponentSystem
	{
	public:
		const uint32_t SEQ_BYTES = 256 * 1000;
		const uint32_t INDEX_BITS_COMP = (1 << (uint32_t)(std::log2((double)(SEQ_BYTES / TransformData::MAX_COMP_SIZE)) + 1.0)) - 1;
		const uint32_t INDEX_BITS_SEQ = ~INDEX_BITS_COMP;
		std::unordered_map<uint32_t, TransformData> _componentData;

		void Reserve(uint32_t entityId) override;
		void Release(uint32_t entityId) override;
		void Identity(uint32_t entityId) override;
		void Reserve(Entity entity) { Reserve(entity._entityId); }
		void Release(Entity entity) { Release(entity._entityId); }
		void Identity(Entity entity) { Identity(entity._entityId); }
		bool IsReserved(uint32_t entityId) const;
		bool IsReserved(Entity entity) const { return IsReserved(entity._entityId); };
		bool IsStatic(uint32_t entityId) const;
		bool IsStatic(Entity entity) const { return IsStatic(entity._entityId); };

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
		void CalculateMVPSeq(TransformData* tData, uint32_t start, uint32_t end);

	private:
		ctpl::thread_pool _threadPool = ctpl::thread_pool(std::thread::hardware_concurrency());
		std::unordered_map<uint32_t, TransformData>::iterator NCReserve(uint32_t entityId);
	};

	class RigidbodySystem : public IComponentSystem
	{
	public:
		std::unordered_map<uint32_t, RigidbodyData> _componentData;
		const uint32_t SEQ_BYTES = 256 * 1000;
		const uint32_t INDEX_BITS_COMP = ~(uint32_t(0)) >> (uint32_t)std::ceil(std::log2((double)SEQ_BYTES / RigidbodyData::MAX_COMP_SIZE) + 1.0);
		const uint32_t INDEX_BITS_SEQ = ~INDEX_BITS_COMP;

		void Reserve(uint32_t entityId) override;
		void Release(uint32_t entityId) override;
		void Identity(uint32_t entityId) override;
		void Reserve(Entity entity) { Reserve(entity._entityId); }
		void Release(Entity entity) { Release(entity._entityId); }
		void Identity(Entity entity) { Identity(entity._entityId); }

		void SetMass(uint32_t entityId, float mass);
		void SetMomentOfInertia(uint32_t entityId, float inertia);
		void SetVelocity(uint32_t entityId, const glm::vec3& velocity);
		void SetVelocity(uint32_t entityId, float x, float y, float z);
		void SetAcceleration(uint32_t entityId, const glm::vec3& acceleration);
		void SetAcceleration(uint32_t entityId, float x, float y, float z);
		void SetAngularVelocity(uint32_t entityId, const glm::vec3& angularVelocity);
		void SetAngularVelocity(uint32_t entityId, float x, float y, float z);

	private:
		std::unordered_map<uint32_t, RigidbodyData>::iterator NCReserve(uint32_t entityId);
	};

	class MeshSystem : public IComponentSystem
	{
	public:
		const uint32_t SEQ_BYTES = 256 * 1000;
		const uint32_t INDEX_BITS_COMP = ~(uint32_t(0)) >> (uint32_t)std::ceil(std::log2((double)SEQ_BYTES / MeshData::MAX_COMP_SIZE) + 1.0);
		const uint32_t INDEX_BITS_SEQ = ~INDEX_BITS_COMP;
		std::unordered_map<uint32_t, MeshData> _componentData;

		void Reserve(uint32_t entityId) override;
		void Release(uint32_t entityId) override;
		void Identity(uint32_t entityId) override;
		void Reserve(Entity entity) { Reserve(entity._entityId); }
		void Release(Entity entity) { Release(entity._entityId); }
		void Identity(Entity entity) { Identity(entity._entityId); }

		void SetMesh(uint32_t entityId, const Mesh& mesh);
		void SetMesh(Entity entity, const Mesh& mesh) { SetMesh(entity._entityId, mesh); }

	private:
		std::unordered_map<uint32_t, MeshData>::iterator NCReserve(uint32_t entityId);
	};

	class CameraSystem : public IComponentSystem
	{
	public:
		std::unordered_map<uint32_t, CameraData> _componentData;
		const uint32_t SEQ_BYTES = 256 * 1000;
		const uint32_t INDEX_BITS_COMP = ~(uint32_t(0)) >> (uint32_t)std::ceil(std::log2((double)SEQ_BYTES / CameraData::MAX_COMP_SIZE) + 1.0);
		const uint32_t INDEX_BITS_SEQ = ~INDEX_BITS_COMP;

		void Reserve(uint32_t entityId) override;
		void Release(uint32_t entityId) override;
		void Identity(uint32_t entityId) override;
		void Reserve(Entity entity) { Reserve(entity._entityId); };
		void Release(Entity entity) { Release(entity._entityId); };
		void Identity(Entity entity) { Identity(entity._entityId); };

		Camera GetCamera(uint32_t entityId) const;
		void LookAt(uint32_t entityId, glm::vec3 position, glm::vec3 center, glm::vec3 up);
		void LookAt(Entity entity, glm::vec3 position, glm::vec3 center, glm::vec3 up) { LookAt(entity._entityId, position, center, up); }

	private:
		std::unordered_map<uint32_t, CameraData>::iterator NCReserve(uint32_t entityId);

	};

	class ColliderSystem : public IComponentSystem
	{
	public:
		const uint32_t SEQ_BYTES = 256 * 1000;
		const uint32_t INDEX_BITS_COMP = ~(uint32_t(0)) >> (uint32_t)std::ceil(std::log2((double)SEQ_BYTES / ColliderData::MAX_COMP_SIZE) + 1.0);
		const uint32_t INDEX_BITS_SEQ = ~INDEX_BITS_COMP;
		std::unordered_map<uint32_t, ColliderData> _componentData;

		void Reserve(uint32_t entityId) override;
		void Release(uint32_t entityId) override;
		void Identity(uint32_t entityId) override;
		void Reserve(Entity entity) { Reserve(entity._entityId); }
		void Release(Entity entity) { Release(entity._entityId); }
		void Identity(Entity entity) { Identity(entity._entityId); }

		void CalculateWorldBounds(const TransformSystem& tSystem, const MeshSystem& mSystem);
		void SetMeshId(uint32_t entityId, uint32_t meshId);
		void SetMeshId(Entity entity, uint32_t meshId) { SetMeshId(entity._entityId, meshId); }
		void SetTransformId(uint32_t entityId, uint32_t transformId);
		void SetTransformId(Entity entity, uint32_t transformId) { SetTransformId(entity._entityId, transformId); }

	private:
		std::unordered_map<uint32_t, ColliderData>::iterator NCReserve(uint32_t entityId);
	};
}