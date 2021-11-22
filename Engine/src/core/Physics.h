#pragma once
#include "ComponentSystem.h"
#include "Common.h"

namespace idop::phys
{
	bool BoundsCollide(const Bounds3D& first, const Bounds3D& second)
	{
		if (first._min.x > second._max.x || first._min.y > second._max.y || first._max.x < second._min.x || first._max.y < second._min.y)
			return false;
		return true;
	}

	void ProcessRigidbodies(RigidbodySystem* rbSystem, TransformSystem* tSystem, float deltaTime, const glm::vec3& gravity)
	{
		uint32_t entityId;
		for (auto const& [rbSeqIndex, rbData] : rbSystem->_componentData)
		{
			for (uint32_t i = 0, entityId = rbSeqIndex; i < rbSystem->INDEX_BITS_COMP; ++i, ++entityId)
			{
				if (!rbData._reserved[i] || !tSystem->IsReserved(entityId))
					continue;
				if (tSystem->_componentData[entityId & tSystem->INDEX_BITS_SEQ]._isStatic[entityId & tSystem->INDEX_BITS_COMP])
					continue;
				rbData._velocity[i] += rbData._acceleration[i] * deltaTime;
				if (rbData._useGravity[i])
					rbData._velocity[i] += gravity * deltaTime;
				tSystem->Translate(entityId, rbData._velocity[i]);
			}
		}
	}

	float CollisionImpulse(float e, float massFirst, float massSecond, const glm::vec3& collisionNormal, const glm::vec3& velocityFirst, const glm::vec3& velocitySecond)
	{
		float inumerator = (velocityFirst.x - velocitySecond.x) * collisionNormal.x
			+ (velocityFirst.y - velocitySecond.y) * collisionNormal.y
			+ (velocityFirst.z - velocitySecond.z) * collisionNormal.z;
		float denominator = (1.0f / massFirst) + (1.0f / massSecond);
		return -(1.0f + e) * inumerator / denominator;
	}

	bool CollisionComponentsReserved(uint32_t entityId, const TransformSystem& tSystem, const ColliderSystem& colSystem, const RigidbodySystem& rbSystem)
	{
		if (!colSystem._componentData.at(entityId & colSystem.INDEX_BITS_COMP)._reserved[entityId & colSystem.INDEX_BITS_COMP])
			return false;
		if (!tSystem._componentData.at(entityId & tSystem.INDEX_BITS_SEQ)._reserved[entityId & tSystem.INDEX_BITS_COMP])
			return false;
		if (!rbSystem._componentData.at(entityId & rbSystem.INDEX_BITS_SEQ)._reserved[entityId & rbSystem.INDEX_BITS_COMP])
			return false;
		return true;
	}

	void ProcessCollisions(float deltaTime, const TransformSystem& tSystem, const ColliderSystem& colSystem, RigidbodySystem& rbSystem)
	{
		float impulse;
		glm::vec3 firstToSecond, normal;

		for (auto itA = colSystem._componentData.begin(); itA != colSystem._componentData.end(); ++itA)
		{
			for (int i = 0; i < colSystem.INDEX_BITS_COMP; ++i)
			{
				if (!CollisionComponentsReserved(itA->first + i, tSystem, colSystem, rbSystem))
					continue;

				for (auto itB = itA; itB != colSystem._componentData.end(); ++itB)
				{
					for (int j = itA == itB ? i + 1 : 0; j < colSystem.INDEX_BITS_COMP; ++j)
					{
						if (!CollisionComponentsReserved(itB->first + j, tSystem, colSystem, rbSystem))
							continue;
						if (!BoundsCollide(
							itA->second._worldBounds[(itA->first + i) & colSystem.INDEX_BITS_COMP],
							itB->second._worldBounds[(itB->first + j) & colSystem.INDEX_BITS_COMP]))
							continue;
					}
				}
			}
		}
	}

	//void Collide(float deltaTime, uint32_t entityIdA, uint32_t entityIdB, const TransformSystem& tSystem)
	//{

	//}
}