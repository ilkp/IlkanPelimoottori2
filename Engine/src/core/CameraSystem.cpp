#include "ComponentSystem.h"
#include <glm/gtx/matrix_decompose.hpp>
#include <glm/gtx/quaternion.hpp>

namespace idop
{
	Camera CameraSystem::GetCamera(uint32_t entityId) const
	{
		Camera camera;
		camera._projectionMatrix = _componentData.at(entityId & INDEX_BITS_SEQ)._projectionMatrix[entityId & INDEX_BITS_COMP];
		camera._viewMatrix = _componentData.at(entityId & INDEX_BITS_SEQ)._viewMatrix[entityId & INDEX_BITS_COMP];
		return camera;
	}

	void CameraSystem::LookAt(uint32_t entityId, const glm::vec3& position, const glm::vec3& center, const glm::vec3& up)
	{
		_componentData[entityId & INDEX_BITS_SEQ]._viewMatrix[entityId & INDEX_BITS_COMP] = glm::lookAt(position, center, up);
	}

    void CameraSystem::ApplyTransforms(uint32_t entityId, const glm::vec3& position, const glm::quat& rotation)
    {
		glm::vec3 direction = glm::toMat4(rotation) * glm::vec4(0.0f, 0.0f, -1.0f, 1.0f);
		_componentData[entityId & INDEX_BITS_SEQ]._viewMatrix[entityId & INDEX_BITS_COMP] = glm::lookAt(
			position,
			glm::vec3(0.0f, 0.0f, 0.0f),
			glm::vec3(0.0f, -1.0f, 0.0f)
		);
    }
}