#include "ComponentSystem.h"

namespace idop
{
	void RigidbodySystem::SetMass(uint32_t entityId, float mass)
	{
		_componentData[entityId & INDEX_BITS_SEQ]._mass[entityId & INDEX_BITS_COMP] = mass;
	}

	void RigidbodySystem::SetMomentOfInertia(uint32_t entityId, float inertia)
	{
		_componentData[entityId & INDEX_BITS_SEQ]._momentOfInertia[entityId & INDEX_BITS_COMP] = inertia;
	}

	void RigidbodySystem::SetVelocity(uint32_t entityId, const glm::vec3& velocity)
	{
		_componentData[entityId & INDEX_BITS_SEQ]._velocity[entityId & INDEX_BITS_COMP].x = velocity.x;
		_componentData[entityId & INDEX_BITS_SEQ]._velocity[entityId & INDEX_BITS_COMP].y = velocity.y;
		_componentData[entityId & INDEX_BITS_SEQ]._velocity[entityId & INDEX_BITS_COMP].z = velocity.z;
	}

	void RigidbodySystem::SetVelocity(uint32_t entityId, float x, float y, float z)
	{
		_componentData[entityId & INDEX_BITS_SEQ]._velocity[entityId & INDEX_BITS_COMP].x = x;
		_componentData[entityId & INDEX_BITS_SEQ]._velocity[entityId & INDEX_BITS_COMP].y = y;
		_componentData[entityId & INDEX_BITS_SEQ]._velocity[entityId & INDEX_BITS_COMP].z = z;
	}

	void RigidbodySystem::SetAcceleration(uint32_t entityId, const glm::vec3& acceleration)
	{
		_componentData[entityId & INDEX_BITS_SEQ]._acceleration[entityId & INDEX_BITS_COMP].x = acceleration.x;
		_componentData[entityId & INDEX_BITS_SEQ]._acceleration[entityId & INDEX_BITS_COMP].y = acceleration.y;
		_componentData[entityId & INDEX_BITS_SEQ]._acceleration[entityId & INDEX_BITS_COMP].z = acceleration.z;
	}

	void RigidbodySystem::SetAcceleration(uint32_t entityId, float x, float y, float z)
	{
		_componentData[entityId & INDEX_BITS_SEQ]._acceleration[entityId & INDEX_BITS_COMP].x = x;
		_componentData[entityId & INDEX_BITS_SEQ]._acceleration[entityId & INDEX_BITS_COMP].y = y;
		_componentData[entityId & INDEX_BITS_SEQ]._acceleration[entityId & INDEX_BITS_COMP].z = z;
	}

	void RigidbodySystem::SetAngularVelocity(uint32_t entityId, const glm::vec3& angularVelocity)
	{
		_componentData[entityId & INDEX_BITS_SEQ]._angularVelocity[entityId & INDEX_BITS_COMP].x = angularVelocity.x;
		_componentData[entityId & INDEX_BITS_SEQ]._angularVelocity[entityId & INDEX_BITS_COMP].y = angularVelocity.y;
		_componentData[entityId & INDEX_BITS_SEQ]._angularVelocity[entityId & INDEX_BITS_COMP].z = angularVelocity.z;
	}

	void RigidbodySystem::SetAngularVelocity(uint32_t entityId, float x, float y, float z)
	{
		_componentData[entityId & INDEX_BITS_SEQ]._angularVelocity[entityId & INDEX_BITS_COMP].x = x;
		_componentData[entityId & INDEX_BITS_SEQ]._angularVelocity[entityId & INDEX_BITS_COMP].y = y;
		_componentData[entityId & INDEX_BITS_SEQ]._angularVelocity[entityId & INDEX_BITS_COMP].z = z;
	}
    glm::vec3 RigidbodySystem::GetAngularVelocityDeg(uint32_t entityId) const
    {
		const glm::vec3& target = _componentData.at(entityId & INDEX_BITS_SEQ)._angularVelocity[entityId & INDEX_BITS_COMP];
		return glm::vec3(glm::degrees(target.x), glm::degrees(target.y), glm::degrees(target.z));
    }
}