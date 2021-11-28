#pragma once
#include <glm/gtx/quaternion.hpp>
#include <math.h>
#include <optional>
#include "ComponentSystem.h"
#include "Common.h"

namespace idop::phys
{
#define HalfSpaceTest(planeNormal, planePoint, comparisonPoint) glm::dot(planeNormal, comparisonPoint - planePoint)

	bool BoundsAreColliding(const Bounds3D& first, const Bounds3D& second)
	{
		if (first._min.x > second._max.x ||
			first._max.x < second._min.x ||
			first._min.y > second._max.y ||
			first._max.y < second._min.y ||
			first._min.z > second._max.z ||
			first._max.z < second._min.z
			)
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
				tSystem->Rotate(entityId, rbData._angularVelocity[i]);
			}
		}
	}

	float CollisionImpulse(
		float coefficientOfRestitution,
		const glm::vec3& collisionPoint,
		const glm::vec3& collisionNormalUnit,
		const glm::vec3& aToPoint,
		const glm::vec3& bToPoint,
		const RigidbodySystem& rbSystem,
		const uint32_t entityIdA,
		const uint32_t entityIdB)
	{
		glm::vec3 pointVelocityA = *rbSystem.GetVelocity(entityIdA) + glm::cross(*rbSystem.GetAngularVelocity(entityIdA), aToPoint);
		glm::vec3 pointVelocityB = *rbSystem.GetVelocity(entityIdB) + glm::cross(*rbSystem.GetAngularVelocity(entityIdB), bToPoint);
		glm::vec3 relativeVelocity = pointVelocityA - pointVelocityB;
		float a = -(1.0f + coefficientOfRestitution);
		float b = glm::dot(relativeVelocity, collisionNormalUnit);
		//float b1 = relativeVelocity.x * collisionNormalUnit.x
		float c = 1.0f / rbSystem.GetMass(entityIdA) + 1.0f / rbSystem.GetMass(entityIdB);
		float d = std::pow(glm::length(glm::cross(aToPoint, collisionNormalUnit)), 2.0f) / rbSystem.GetMomentOfInertia(entityIdA);
		float e = std::pow(glm::length(glm::cross(bToPoint, collisionNormalUnit)), 2.0f) / rbSystem.GetMomentOfInertia(entityIdB);

		return -(1.0f + coefficientOfRestitution) * glm::dot(relativeVelocity, collisionNormalUnit)
			/ (
				1.0f / rbSystem.GetMass(entityIdA) + 1.0f / rbSystem.GetMass(entityIdB)
				+ std::pow(glm::length(glm::cross(aToPoint, collisionNormalUnit)), 2.0f) / rbSystem.GetMomentOfInertia(entityIdA)
				+ std::pow(glm::length(glm::cross(bToPoint, collisionNormalUnit)), 2.0f) / rbSystem.GetMomentOfInertia(entityIdB)
				);
	}

	// returns normal of the colliding triangle
	std::optional<glm::vec3> PointCollidesWithConvex(
		const glm::vec3& worldPoint,
		const glm::vec3* worldVertices,
		const int* triangles,
		const uint32_t verticesLength,
		const uint32_t trianglesLength)
	{
		glm::vec3 planeVecA, planeVecB, planePoint, planeNormal;
		float distanceToPlane;
		//float shortestDistToPlane = std::numeric_limits<float>::max();
		glm::vec3 collisionNormal(0.0f, 0.0f, 0.0f);
		for (int i = 0; i < trianglesLength; i += 3)
		{
			planeVecA = worldVertices[triangles[i + 1]] - worldVertices[triangles[i]];
			planeVecB = worldVertices[triangles[i + 2]] - worldVertices[triangles[i + 1]];
			planePoint = worldVertices[triangles[i]];
			planeNormal = glm::normalize(glm::cross(planeVecB, planeVecA));
			if (HalfSpaceTest(planeNormal, planePoint, worldPoint) > 0.0f)
				return {};
			distanceToPlane = glm::dot(planeNormal, worldPoint - planePoint);
			collisionNormal += planeNormal / distanceToPlane;
			//if (distanceToPlane < shortestDistToPlane)
			//{
			//	shortestDistToPlane = distanceToPlane;
			//	collisionNormal = planeNormal;
			//}
		}
		return std::optional<glm::vec3>(glm::normalize(collisionNormal));
	}

	// returns normal of the colliding triangle
	std::optional<glm::vec3> PointCollidesWithConvex(
		const glm::vec3& worldPoint,
		const glm::vec3* localVertices,
		const int* triangles,
		const uint32_t verticesLength,
		const uint32_t trianglesLength,
		const glm::mat4& localToWorldSpace)
	{
		glm::vec3 planeVecA, planeVecB, planePoint, planeNormal;
		float distanceToPlane;
		glm::vec3 collisionNormal(0.0f, 0.0f, 0.0f);
		for (int i = 0; i < trianglesLength; i += 3)
		{
			planeVecA = localToWorldSpace * glm::vec4(localVertices[triangles[i + 1]], 1.0f) - localToWorldSpace * glm::vec4(localVertices[triangles[i]], 1.0f);
			planeVecB = localToWorldSpace * glm::vec4(localVertices[triangles[i + 2]], 1.0f) - localToWorldSpace * glm::vec4(localVertices[triangles[i + 1]], 1.0f);
			planePoint = localToWorldSpace * glm::vec4(localVertices[triangles[i]], 1.0f);
			planeNormal = glm::normalize(glm::cross(planeVecB, planeVecA));
			if (HalfSpaceTest(planeNormal, planePoint, worldPoint) > 0.0f)
				return {};
			distanceToPlane = std::abs(glm::dot(planeNormal, worldPoint - planePoint));
			distanceToPlane = std::clamp(distanceToPlane, 0.001f, 1000.0f);
			collisionNormal += planeNormal / distanceToPlane;
		}
		return std::optional<glm::vec3>(glm::normalize(collisionNormal));
	}

	void Collide(
		const glm::vec3& collisionPoint,
		const glm::vec3& collisionNormalUnit,
		const TransformSystem& tSystem,
		const RigidbodySystem& rbSystem,
		const uint32_t entityIdA,
		const uint32_t entityIdB)
	{
		glm::vec3 aToPoint = collisionPoint - tSystem.GetPositionVector(entityIdA);
		glm::vec3 bToPoint = collisionPoint - tSystem.GetPositionVector(entityIdB);
		float impulse = CollisionImpulse(
			0.1f,
			collisionPoint,
			collisionNormalUnit,
			aToPoint, bToPoint,
			rbSystem,
			entityIdA, entityIdB);

		rbSystem._componentData.at(entityIdA & rbSystem.INDEX_BITS_SEQ)._velocity[entityIdA & rbSystem.INDEX_BITS_COMP]
			+= collisionNormalUnit * impulse / rbSystem.GetMass(entityIdA);

		rbSystem._componentData.at(entityIdB & rbSystem.INDEX_BITS_SEQ)._velocity[entityIdB & rbSystem.INDEX_BITS_COMP]
			-= collisionNormalUnit * impulse / rbSystem.GetMass(entityIdB);

		rbSystem._componentData.at(entityIdA & rbSystem.INDEX_BITS_SEQ)._angularVelocity[entityIdA & rbSystem.INDEX_BITS_COMP]
			+= impulse / rbSystem.GetMomentOfInertia(entityIdA) * glm::cross(aToPoint, collisionNormalUnit);

		rbSystem._componentData.at(entityIdB & rbSystem.INDEX_BITS_SEQ)._angularVelocity[entityIdB & rbSystem.INDEX_BITS_COMP]
			-= impulse / rbSystem.GetMomentOfInertia(entityIdB) * glm::cross(bToPoint, collisionNormalUnit);
	}

	void HandleCollision(
		float deltaTime,
		const TransformSystem& tSystem,
		const ColliderSystem& cSystem,
		const MeshSystem& mSystem,
		RigidbodySystem& rbSystem,
		uint32_t entityIdA,
		uint32_t entityIdB)
	{
		const TransformData& tDataA = tSystem._componentData.at(entityIdA & tSystem.INDEX_BITS_SEQ);
		const TransformData& tDataB = tSystem._componentData.at(entityIdB & tSystem.INDEX_BITS_SEQ);
		const ColliderData& cDataA = cSystem._componentData.at(entityIdA & cSystem.INDEX_BITS_SEQ);
		const ColliderData& cDataB = cSystem._componentData.at(entityIdB & cSystem.INDEX_BITS_SEQ);
		const uint32_t meshIndexA = cDataA._meshId[entityIdA & cSystem.INDEX_BITS_COMP];
		const uint32_t meshIndexB = cDataB._meshId[entityIdB & cSystem.INDEX_BITS_COMP];
		const MeshData& mDataA = mSystem._componentData.at(meshIndexA & mSystem.INDEX_BITS_SEQ);
		const MeshData& mDataB = mSystem._componentData.at(meshIndexB & mSystem.INDEX_BITS_SEQ);
		std::optional<glm::vec3> collisionNormal;
		glm::vec3 worldPoint, EntityAtoPoint;
		glm::mat4 transformationA =
			tDataA._translation[entityIdA & tSystem.INDEX_BITS_COMP]
			* glm::toMat4(tDataA._quaternion[entityIdA & tSystem.INDEX_BITS_COMP])
			* cDataA._offset[entityIdA & cSystem.INDEX_BITS_COMP]
			* tDataA._scale[entityIdA & tSystem.INDEX_BITS_COMP]
			* cDataA._scale[entityIdA & cSystem.INDEX_BITS_COMP];
		glm::mat4 transformationB =
			tDataB._translation[entityIdB & tSystem.INDEX_BITS_COMP]
			* glm::toMat4(tDataB._quaternion[entityIdB & tSystem.INDEX_BITS_COMP])
			* cDataB._offset[entityIdB & cSystem.INDEX_BITS_COMP]
			* tDataB._scale[entityIdB & tSystem.INDEX_BITS_COMP]
			* cDataB._scale[entityIdB & cSystem.INDEX_BITS_COMP];
		for (int i = 0; i < mDataB._verticesLength[meshIndexB & mSystem.INDEX_BITS_COMP]; ++i)
		{
			worldPoint = transformationB * glm::vec4(mDataB._vertices[meshIndexB & mSystem.INDEX_BITS_COMP][i], 1.0f);
			collisionNormal = PointCollidesWithConvex(
				worldPoint,
				mDataA._vertices[meshIndexA & mSystem.INDEX_BITS_COMP],
				mDataA._triangles[meshIndexA & mSystem.INDEX_BITS_COMP],
				mDataA._verticesLength[meshIndexA & mSystem.INDEX_BITS_COMP],
				mDataA._trianglesLength[meshIndexA & mSystem.INDEX_BITS_COMP],
				transformationA);
			std::cout << "hit" << std::endl;
			if (collisionNormal.has_value())
				Collide(
					worldPoint,
					collisionNormal.value(),
					tSystem,
					rbSystem,
					entityIdA,
					entityIdB);
		}
	}

	void ProcessCollisions(float deltaTime, const TransformSystem& tSystem, const ColliderSystem& cSystem, const MeshSystem& mSystem, RigidbodySystem& rbSystem)
	{
		for (auto itA = cSystem._componentData.begin(); itA != cSystem._componentData.end(); ++itA)
		{
			for (int i = 0; i < cSystem.INDEX_BITS_COMP; ++i)
			{
				if (!itA->second._reserved[(itA->first + i) & cSystem.INDEX_BITS_COMP])
					continue;

				for (auto itB(itA); itB != cSystem._componentData.end(); ++itB)
				{
					for (int j = itA == itB ? i + 1 : 0; j < cSystem.INDEX_BITS_COMP; ++j)
					{
						if (!itB->second._reserved[(itB->first + j) & cSystem.INDEX_BITS_COMP])
							continue;
						if (!BoundsAreColliding(
							itA->second._worldBounds[(itA->first + i) & cSystem.INDEX_BITS_COMP],
							itB->second._worldBounds[(itB->first + j) & cSystem.INDEX_BITS_COMP]))
							continue;
						HandleCollision(deltaTime, tSystem, cSystem, mSystem, rbSystem, itA->first + i, itB->first + j);
					}
				}
			}
		}
	}


	//void Collide(float deltaTime, uint32_t entityIdA, uint32_t entityIdB, const TransformSystem& tSystem)
	//{

	//}
}