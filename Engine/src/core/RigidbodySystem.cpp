#include "ComponentSystem.h"

namespace idop
{
	void RigidbodySystem::Reserve(uint32_t entityId)
	{
		std::unordered_map<uint32_t, RigidbodyData>::iterator it = _componentData.find(entityId & INDEX_BITS_SEQ);
		if (it == _componentData.end())
		{
			it = _componentData.insert({ entityId & INDEX_BITS_SEQ, RigidbodyData() }).first;
			(*it).second.Allocate(INDEX_BITS_COMP);
		}
		(*it).second._reserved[entityId & INDEX_BITS_COMP] = true;
	}

    void RigidbodySystem::Release(uint32_t entityId)
    {
		std::unordered_map<uint32_t, RigidbodyData>::iterator it = _componentData.find(entityId & INDEX_BITS_SEQ);
		if (it != _componentData.end())
			(*it).second._reserved[entityId & INDEX_BITS_COMP] = false;
    }

	void RigidbodySystem::Identity(uint32_t entityId)
	{
		uint32_t compIndex = entityId & INDEX_BITS_COMP;
		std::unordered_map<uint32_t, RigidbodyData>::iterator it = _componentData.find(entityId & INDEX_BITS_SEQ);
		if (it == _componentData.end())
			it = NCReserve(entityId);
		(*it).second._mass[compIndex] = 1.0f;
		(*it).second._momentOfInertia[compIndex] = 0.1f;
		(*it).second._velocity[compIndex].x = 0.0f;
		(*it).second._velocity[compIndex].y = 0.0f;
		(*it).second._velocity[compIndex].z = 0.0f;
		(*it).second._acceleration[compIndex].x = 0.0f;
		(*it).second._acceleration[compIndex].y = 0.0f;
		(*it).second._acceleration[compIndex].z = 0.0f;
		(*it).second._angularVelocity[compIndex].x = 0.0f;
		(*it).second._angularVelocity[compIndex].y = 0.0f;
		(*it).second._angularVelocity[compIndex].z = 0.0f;
	}

	void RigidbodySystem::SetMass(uint32_t entityId, float mass)
	{
		_componentData[entityId & INDEX_BITS_SEQ]._mass[entityId & INDEX_BITS_COMP] = mass;
	}

	std::unordered_map<uint32_t, RigidbodyData>::iterator RigidbodySystem::NCReserve(uint32_t entityId)
	{
		std::unordered_map<uint32_t, RigidbodyData>::iterator it = _componentData.insert({ entityId & INDEX_BITS_SEQ, RigidbodyData() }).first;
		(*it).second.Allocate(INDEX_BITS_COMP);
		(*it).second._reserved[entityId & INDEX_BITS_COMP] = true;
		return it;
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
}