#include "ComponentSystem.h"

namespace idop
{
	void CameraSystem::Reserve(uint32_t entityId)
	{
		std::unordered_map<uint32_t, CameraData>::iterator it = _componentData.find(entityId & INDEX_BITS_SEQ);
		if (it == _componentData.end())
		{
			it = _componentData.insert({ entityId & INDEX_BITS_SEQ, CameraData() }).first;
			(*it).second.Allocate(INDEX_BITS_COMP);
		}
		(*it).second._reserved[entityId & INDEX_BITS_COMP] = true;
	}

	void CameraSystem::Release(uint32_t entityId)
	{
		std::unordered_map<uint32_t, CameraData>::iterator it = _componentData.find(entityId & INDEX_BITS_SEQ);
		if (it != _componentData.end())
			(*it).second._reserved[entityId & INDEX_BITS_COMP] = false;
	}

	void CameraSystem::Identity(uint32_t entityId)
	{
		uint32_t componentIndex = entityId & INDEX_BITS_COMP;
		std::unordered_map<uint32_t, CameraData>::iterator it = _componentData.find(entityId & INDEX_BITS_SEQ);
		if (it == _componentData.end())
			it = NCReserve(entityId);
		(*it).second._projectionMatrix[componentIndex] = glm::perspective(
			glm::radians(45.0f),
			4.0f / 3.0f,
			0.1f,
			100.0f);
		(*it).second._viewMatrix[componentIndex] = glm::lookAt(
			glm::vec3(20.0f, 20.0f, 0.0f),
			glm::vec3(0.0f, 0.0f, 0.0f),
			glm::vec3(0.0f, 1.0f, 0.0f));
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
		std::unordered_map<uint32_t, CameraData>::iterator it = _componentData.insert({ entityId & INDEX_BITS_SEQ, CameraData() }).first;
		(*it).second.Allocate(INDEX_BITS_COMP);
		(*it).second._reserved[entityId & INDEX_BITS_COMP] = true;
		return it;
	}
}