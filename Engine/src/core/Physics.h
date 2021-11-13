#pragma once
#include "ComponentSystem.h"
#include "Common.h"

namespace idop::phys
{
	bool BoundsCollide(const Bounds3D& first, const Bounds3D second)
	{
		if (first._min.x > second._max.x || first._min.y > second._max.y || first._max.x < second._min.x || first._max.y < second._min.y)
			return false;
		return true;
	}

	void ProcessRbs(RigidbodySystem* rbSystem, TransformSystem* tSystem, float deltaTime, const glm::vec3& gravity)
	{
		uint32_t entityId;
		for (auto const& [rbSeqIndex, rbData] : rbSystem->_componentData)
		{
			for (uint32_t i = 0; i < rbSystem->INDEX_BITS_COMP; ++i)
			{
				entityId = rbSeqIndex + i;
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
}