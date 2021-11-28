#include "ComponentSystem.h"
#include "IdopException.h"
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/quaternion.hpp>

namespace idop
{
	void ColliderSystem::CalculateWorldBounds(const TransformSystem& tSystem, const MeshSystem& mSystem)
	{
		uint32_t tCompIndex;
		uint32_t entityId;
		for (auto& [seqId, colData] : _componentData)
		{
			entityId = seqId;
			for (uint32_t i = 0; i < INDEX_BITS_COMP; ++i, ++entityId)
			{
				tCompIndex = entityId & tSystem.INDEX_BITS_COMP;
				if (!colData._reserved[i])
					continue;
				if (!tSystem._componentData.at(entityId & tSystem.INDEX_BITS_SEQ)._reserved[tCompIndex])
					continue;

				switch (colData._colliderType[i])
				{
				case ColliderType::Box:
				{
					CalculateWorldBoundsBox(tSystem._componentData.at(entityId & tSystem.INDEX_BITS_SEQ), tCompIndex, colData, i);
					break;
				}
				case ColliderType::Sphere:
				{
					throw NotImplementedException("CalculateWorldBounds for sphere collider not implemented.");
					break;
				}
				case ColliderType::Mesh:
				{
					throw NotImplementedException("CalculateWorldBounds for Mesh collider not implemented.");
					break;
				}
				default:
				{
					break;
				}
				}
			}
		}
	}

	void ColliderSystem::CalculateWorldBoundsBox(const TransformData& tData, uint32_t tIndex, const ColliderData& cData, uint32_t cIndex)
	{
		glm::vec4 boxPoints[8];
		BoxPoints(boxPoints);
		for (glm::vec4& point : boxPoints)
		{
			point =
				tData._translation[tIndex]
				* glm::toMat4(tData._quaternion[tIndex])
				* cData._offset[cIndex]
				* tData._scale[tIndex]
				* cData._scale[cIndex]
				* point;
		}
		cData._worldBounds[cIndex]._min.x = boxPoints[0].x;
		cData._worldBounds[cIndex]._max.x = boxPoints[0].x;
		cData._worldBounds[cIndex]._min.y = boxPoints[0].y;
		cData._worldBounds[cIndex]._max.y = boxPoints[0].y;
		cData._worldBounds[cIndex]._min.z = boxPoints[0].z;
		cData._worldBounds[cIndex]._max.z = boxPoints[0].z;
		for (int j = 1; j < 8; ++j)
		{
			if (boxPoints[j].x < cData._worldBounds[cIndex]._min.x)
				cData._worldBounds[cIndex]._min.x = boxPoints[j].x;

			if (boxPoints[j].x > cData._worldBounds[cIndex]._max.x)
				cData._worldBounds[cIndex]._max.x = boxPoints[j].x;

			if (boxPoints[j].y < cData._worldBounds[cIndex]._min.y)
				cData._worldBounds[cIndex]._min.y = boxPoints[j].y;

			if (boxPoints[j].y > cData._worldBounds[cIndex]._max.y)
				cData._worldBounds[cIndex]._max.y = boxPoints[j].y;

			if (boxPoints[j].z < cData._worldBounds[cIndex]._min.z)
				cData._worldBounds[cIndex]._min.z = boxPoints[j].z;

			if (boxPoints[j].z > cData._worldBounds[cIndex]._max.z)
				cData._worldBounds[cIndex]._max.z = boxPoints[j].z;
		}
	}

	void ColliderSystem::BoxPoints(glm::vec4* points)
	{
		const float pointDist = PrimitiveShapeVertexDefaultDistance;
		points[0].x = -pointDist;
		points[0].y = -pointDist;
		points[0].z = pointDist;
		points[0].w = 1.0f;

		points[1].x = pointDist;
		points[1].y = -pointDist;
		points[1].z = pointDist;
		points[1].w = 1.0f;

		points[2].x = pointDist;
		points[2].y = pointDist;
		points[2].z = pointDist;
		points[2].w = 1.0f;

		points[3].x = -pointDist;
		points[3].y = pointDist;
		points[3].z = pointDist;
		points[3].w = 1.0f;

		points[4].x = -pointDist;
		points[4].y = -pointDist;
		points[4].z = -pointDist;
		points[4].w = 1.0f;

		points[5].x = pointDist;
		points[5].y = -pointDist;
		points[5].z = -pointDist;
		points[5].w = 1.0f;

		points[6].x = pointDist;
		points[6].y = pointDist;
		points[6].z = -pointDist;
		points[6].w = 1.0f;

		points[7].x = -pointDist;
		points[7].y = pointDist;
		points[7].z = -pointDist;
		points[7].w = 1.0f;
	}

	void ColliderSystem::SetMeshId(uint32_t entityId, uint32_t meshId)
	{
		_componentData[entityId & INDEX_BITS_SEQ]._meshId[entityId & INDEX_BITS_COMP] = meshId;
	}
}