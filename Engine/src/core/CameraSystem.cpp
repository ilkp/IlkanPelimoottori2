#include "ComponentSystem.h"

namespace idop
{
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
}