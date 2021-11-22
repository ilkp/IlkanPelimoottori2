#include "ComponentSystem.h"

namespace idop
{
	void CameraSystem::Reserve(uint32_t entityId)
	{
		auto it = _componentData.find(entityId & INDEX_BITS_SEQ);
		if (it == _componentData.end())
		{
			CameraData cData;
			it = _componentData.insert(std::make_pair(entityId & INDEX_BITS_SEQ, std::move(cData))).first;
			it->second.Allocate(INDEX_BITS_COMP + 1);
		}
		it->second._reserved[entityId & INDEX_BITS_COMP] = true;
	}

	void CameraSystem::Release(uint32_t entityId)
	{
		auto it = _componentData.find(entityId & INDEX_BITS_SEQ);
		if (it != _componentData.end())
			it->second._reserved[entityId & INDEX_BITS_COMP] = false;
	}

	void CameraSystem::Identity(uint32_t entityId)
	{
		uint32_t componentIndex = entityId & INDEX_BITS_COMP;
		std::unordered_map<uint32_t, CameraData>::iterator it = _componentData.find(entityId & INDEX_BITS_SEQ);
		if (it == _componentData.end())
			it = NCReserve(entityId);
		it->second.Identity(componentIndex);
	}

	Camera CameraSystem::GetCamera(uint32_t entityId) const
	{
		Camera camera;
		camera._projectionMatrix = _componentData.at(entityId & INDEX_BITS_SEQ)._projectionMatrix[entityId & INDEX_BITS_COMP];
		camera._viewMatrix = _componentData.at(entityId & INDEX_BITS_SEQ)._viewMatrix[entityId & INDEX_BITS_COMP];
		return camera;
	}

	void CameraSystem::LookAt(uint32_t entityId, glm::vec3 position, glm::vec3 center, glm::vec3 up)
	{
		_componentData[entityId & INDEX_BITS_SEQ]._viewMatrix[entityId & INDEX_BITS_COMP] = glm::lookAt(position, center, up);
	}

	std::unordered_map<uint32_t, CameraData>::iterator CameraSystem::NCReserve(uint32_t entityId)
	{
		std::unordered_map<uint32_t, CameraData>::iterator it = _componentData.insert(std::make_pair(entityId & INDEX_BITS_SEQ, std::move(CameraData()))).first;
		it->second.Allocate(INDEX_BITS_COMP + 1);
		it->second._reserved[entityId & INDEX_BITS_COMP] = true;
		return it;
	}
}