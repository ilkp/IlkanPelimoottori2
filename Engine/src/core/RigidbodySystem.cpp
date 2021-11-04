#include "ComponentSystem.h"

namespace idop
{
	idop::RigidbodySystem::RigidbodySystem()
	{
		seqLength = seqBytes / (uint32_t)Size();
		seqIndexLength = (uint32_t)std::ceil(std::log2((double)seqLength) + 1.0);
		componentIndexBits = ~(uint32_t(0)) >> seqIndexLength;
		seqIndexBits = ~componentIndexBits;
	}

	void RigidbodySystem::Identity(uint32_t id)
	{
		const uint32_t seqIndex = id & seqIndexBits;
		const uint32_t compIndex = id & componentIndexBits;
		_componentData[seqIndexBits]._mass[compIndex] = 1.0f;
		_componentData[seqIndexBits]._momentOfInertia[compIndex] = 0.1f;
		_componentData[seqIndexBits]._velocity[compIndex].x = 0.0f;
		_componentData[seqIndexBits]._velocity[compIndex].y = 0.0f;
		_componentData[seqIndexBits]._velocity[compIndex].z = 0.0f;
		_componentData[seqIndexBits]._acceleration[compIndex].x = 0.0f;
		_componentData[seqIndexBits]._acceleration[compIndex].y = 0.0f;
		_componentData[seqIndexBits]._acceleration[compIndex].z = 0.0f;
		_componentData[seqIndexBits]._angularVelocity[compIndex].x = 0.0f;
		_componentData[seqIndexBits]._angularVelocity[compIndex].y = 0.0f;
		_componentData[seqIndexBits]._angularVelocity[compIndex].z = 0.0f;
	}
	void RigidbodySystem::SetMass(uint32_t entityId, float mass)
	{
		_componentData[entityId & seqIndexBits]._mass[entityId & componentIndexBits] = mass;
	}
	void RigidbodySystem::SetMomentOfInertia(uint32_t entityId, float inertia)
	{
		_componentData[entityId & seqIndexBits]._momentOfInertia[entityId & componentIndexBits] = inertia;
	}
	void RigidbodySystem::SetVelocity(uint32_t entityId, const glm::vec3& velocity)
	{
		_componentData[entityId & seqIndexBits]._velocity[entityId & componentIndexBits].x = velocity.x;
		_componentData[entityId & seqIndexBits]._velocity[entityId & componentIndexBits].y = velocity.y;
		_componentData[entityId & seqIndexBits]._velocity[entityId & componentIndexBits].z = velocity.z;
	}
	void RigidbodySystem::SetVelocity(uint32_t entityId, float x, float y, float z)
	{
		_componentData[entityId & seqIndexBits]._velocity[entityId & componentIndexBits].x = x;
		_componentData[entityId & seqIndexBits]._velocity[entityId & componentIndexBits].y = y;
		_componentData[entityId & seqIndexBits]._velocity[entityId & componentIndexBits].z = z;
	}
	void RigidbodySystem::SetAcceleration(uint32_t entityId, const glm::vec3& acceleration)
	{
		_componentData[entityId & seqIndexBits]._acceleration[entityId & componentIndexBits].x = acceleration.x;
		_componentData[entityId & seqIndexBits]._acceleration[entityId & componentIndexBits].y = acceleration.y;
		_componentData[entityId & seqIndexBits]._acceleration[entityId & componentIndexBits].z = acceleration.z;
	}
	void RigidbodySystem::SetAcceleration(uint32_t entityId, float x, float y, float z)
	{
		_componentData[entityId & seqIndexBits]._acceleration[entityId & componentIndexBits].x = x;
		_componentData[entityId & seqIndexBits]._acceleration[entityId & componentIndexBits].y = y;
		_componentData[entityId & seqIndexBits]._acceleration[entityId & componentIndexBits].z = z;
	}
	void RigidbodySystem::SetAngularVelocity(uint32_t entityId, const glm::vec3& angularVelocity)
	{
		_componentData[entityId & seqIndexBits]._angularVelocity[entityId & componentIndexBits].x = angularVelocity.x;
		_componentData[entityId & seqIndexBits]._angularVelocity[entityId & componentIndexBits].y = angularVelocity.y;
		_componentData[entityId & seqIndexBits]._angularVelocity[entityId & componentIndexBits].z = angularVelocity.z;
	}
	void RigidbodySystem::SetAngularVelocity(uint32_t entityId, float x, float y, float z)
	{
		_componentData[entityId & seqIndexBits]._angularVelocity[entityId & componentIndexBits].x = x;
		_componentData[entityId & seqIndexBits]._angularVelocity[entityId & componentIndexBits].y = y;
		_componentData[entityId & seqIndexBits]._angularVelocity[entityId & componentIndexBits].z = z;
	}
}