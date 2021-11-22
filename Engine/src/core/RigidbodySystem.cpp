#include "ComponentSystem.h"

namespace idop
{
	void RigidbodySystem::Reserve(uint32_t entityId)
	{
		auto it = _componentData.find(entityId & INDEX_BITS_SEQ);
		if (it == _componentData.end())
		{
			it = _componentData.insert(std::make_pair(entityId & INDEX_BITS_SEQ, std::move(RigidbodyData()))).first;
			it->second.Allocate(INDEX_BITS_COMP + 1);
		}
		it->second._reserved[entityId & INDEX_BITS_COMP + 1] = true;
	}

    void RigidbodySystem::Release(uint32_t entityId)
    {
		auto it = _componentData.find(entityId & INDEX_BITS_SEQ);
		if (it != _componentData.end())
			it->second._reserved[entityId & INDEX_BITS_COMP] = false;
    }

	void RigidbodySystem::Identity(uint32_t entityId)
	{
		uint32_t componentIndex = entityId & INDEX_BITS_COMP;
		std::unordered_map<uint32_t, RigidbodyData>::iterator it = _componentData.find(entityId & INDEX_BITS_SEQ);
		if (it == _componentData.end())
			it = NCReserve(entityId);
		it->second.Identity(componentIndex);
	}

	void RigidbodySystem::SetMass(uint32_t entityId, float mass)
	{
		_componentData[entityId & INDEX_BITS_SEQ]._mass[entityId & INDEX_BITS_COMP] = mass;
	}

	std::unordered_map<uint32_t, RigidbodyData>::iterator RigidbodySystem::NCReserve(uint32_t entityId)
	{
		std::unordered_map<uint32_t, RigidbodyData>::iterator it = _componentData.insert(std::make_pair(entityId & INDEX_BITS_SEQ, std::move(RigidbodyData()))).first;
		it->second.Allocate(INDEX_BITS_COMP + 1);
		it->second._reserved[entityId & INDEX_BITS_COMP] = true;
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