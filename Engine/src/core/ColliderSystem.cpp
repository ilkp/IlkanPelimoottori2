#include "ComponentSystem.h"
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/quaternion.hpp>

namespace idop
{
	void idop::ColliderSystem::Reserve(uint32_t entityId)
	{
		auto it = _componentData.find(entityId & INDEX_BITS_SEQ);
		if (it == _componentData.end())
		{
			it = _componentData.insert(std::make_pair(entityId & INDEX_BITS_SEQ, std::move(ColliderData()))).first;
			it->second.Allocate(INDEX_BITS_COMP + 1);
		}
		it->second._reserved[entityId & INDEX_BITS_COMP] = true;
	}

	void ColliderSystem::CalculateWorldBounds(const TransformSystem& tSystem, const MeshSystem& mSystem)
	{
		//float minx, miny, maxx, maxy;
		//for (auto& [seqId, colData] : _componentData)
		//{

		//}
	}

	void ColliderSystem::Release(uint32_t entityId)
	{
		auto it = _componentData.find(entityId & INDEX_BITS_SEQ);
		if (it != _componentData.end())
			it->second._reserved[entityId & INDEX_BITS_COMP] = false;
	}

	void ColliderSystem::Identity(uint32_t entityId)
	{
		uint32_t componentIndex = entityId & INDEX_BITS_COMP;
		std::unordered_map<uint32_t, ColliderData>::iterator it = _componentData.find(entityId & INDEX_BITS_SEQ);
		if (it == _componentData.end())
			it = NCReserve(entityId);
		it->second.Identity(componentIndex);
	}

	void ColliderSystem::SetMeshId(uint32_t entityId, uint32_t meshId)
	{
		_componentData[entityId & INDEX_BITS_SEQ]._meshId[entityId & INDEX_BITS_COMP] = meshId;
	}

	void ColliderSystem::SetTransformId(uint32_t entityId, uint32_t tranformId)
	{
		_componentData[entityId & INDEX_BITS_SEQ]._transformId[entityId & INDEX_BITS_COMP] = tranformId;
	}

	std::unordered_map<uint32_t, ColliderData>::iterator ColliderSystem::NCReserve(uint32_t entityId)
	{
		std::unordered_map<uint32_t, ColliderData>::iterator it = _componentData.insert(std::make_pair(entityId & INDEX_BITS_SEQ, std::move(ColliderData()))).first;
		it->second.Allocate(INDEX_BITS_COMP + 1);
		it->second._reserved[entityId & INDEX_BITS_COMP] = true;
		return it;
	}

}