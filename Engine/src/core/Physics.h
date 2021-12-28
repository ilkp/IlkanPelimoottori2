#pragma once
#include <glm/gtx/quaternion.hpp>
#include <math.h>
#include <optional>
#include "ComponentSystem.h"
#include "Common.h"

#define HalfSpaceTest(planeNormal, planePoint, comparisonPoint) glm::dot(planeNormal, comparisonPoint - planePoint)

namespace idop::phys
{
	bool BoundsAreColliding(const Bounds3D& first, const Bounds3D& second)
	{
		if (first._min.x > second._max.x ||
			first._max.x < second._min.x ||
			first._min.y > second._max.y ||
			first._max.y < second._min.y ||
			first._min.z > second._max.z ||
			first._max.z < second._min.z)
			return false;
		return true;
	}

	std::optional<ContactPoint> GetContactPoint(
		const glm::vec3& worldPoint,
		const glm::vec3& pointVelocityA,
		const glm::vec3& pointVelocityB,
		const glm::vec3* localVertices,
		const int* triangles,
		const uint32_t verticesLength,
		const uint32_t trianglesLength,
		const glm::mat4& localToWorldSpace)
	{
		glm::vec3 planeVecA, planeVecB, planePoint, planeNormal, collisionNormal;
		float distanceToPlane;
		float shortestDistToPlane = std::numeric_limits<float>::max();
		for (int i = 0; i < trianglesLength; i += 3)
		{
			planeVecA = localToWorldSpace * glm::vec4(localVertices[triangles[i + 1]], 1.0f) - localToWorldSpace * glm::vec4(localVertices[triangles[i]], 1.0f);
			planeVecB = localToWorldSpace * glm::vec4(localVertices[triangles[i + 2]], 1.0f) - localToWorldSpace * glm::vec4(localVertices[triangles[i + 1]], 1.0f);
			planePoint = localToWorldSpace * glm::vec4(localVertices[triangles[i]], 1.0f);
			planeNormal = glm::normalize(glm::cross(planeVecA, planeVecB));
			if (HalfSpaceTest(planeNormal, planePoint, worldPoint) > 0.0f)
				return {};
			distanceToPlane = std::abs(glm::dot(planeNormal, planePoint - worldPoint));
			if (distanceToPlane < shortestDistToPlane)
			{
				shortestDistToPlane = distanceToPlane;
				collisionNormal = planeNormal;
			}
		}
		glm::vec3 relativeVel = pointVelocityA - pointVelocityB;
		glm::vec3 tangent = glm::normalize(relativeVel - glm::dot(relativeVel, collisionNormal));
		tangent = glm::length(tangent) != 0.0f ? glm::normalize(tangent) : glm::vec3(0);
		ContactPoint contactPoint;
		contactPoint._normal = collisionNormal;
		contactPoint._worldPoint = worldPoint;
		contactPoint._tangent = tangent;
		contactPoint._penetrationDepth = shortestDistToPlane;
		return contactPoint;
	}

	glm::mat4 InverseInertiaTensor(const glm::vec3& extents, const glm::mat4& rotation, const float mass)
	{
		glm::vec3 size = extents * 2.0f;
		float fraction = 1.0f / 12.0f;
		float xSqr = size.x * size.x;
		float ySqr = size.y * size.y;
		float zSqr = size.z * size.z;
		glm::vec4 i(0.0f);
		i.x = (ySqr + zSqr) * mass * fraction;
		i.y = (xSqr + zSqr) * mass * fraction;
		i.z = (xSqr + ySqr) * mass * fraction;
		i.w = 1.0f;
		glm::mat4 tensor(1.0f);
		tensor[0][0] = i.x;
		tensor[1][1] = i.y;
		tensor[2][2] = i.z;
		tensor[3][3] = i.w;
		return rotation * glm::inverse(tensor) * glm::transpose(rotation);
	}

	float ImpulseMagnitude(
		const glm::vec3& aToPoint,
		const glm::vec3& bToPoint,
		const glm::vec3& normal,
		const glm::vec3& relativeVelocity,
		const float coefA,
		const float coefB,
		const float massA,
		const float massB,
		const glm::mat4& inverseInertiaTA,
		const glm::mat4& inverseInertiaTB)
	{
		float coefficientOfRestitution = std::min(coefA, coefB);
		float inverseMassSum = (massA == 0.0f ? 0.0f : (1.0f / massA)) + (massB == 0.0f ? 0.0f : (1.0f / massB));
		float inumerator = -(1.0f + coefficientOfRestitution) * glm::dot(relativeVelocity, normal);
		auto momentDenomA = glm::cross(glm::vec3(inverseInertiaTA * glm::vec4(glm::cross(aToPoint, normal), 1.0f)), aToPoint);
		auto momentDenomB = glm::cross(glm::vec3(inverseInertiaTB * glm::vec4(glm::cross(bToPoint, normal), 1.0f)), bToPoint);
		float denominator = inverseMassSum + glm::dot(normal, momentDenomA) + glm::dot(normal, momentDenomB);
		if (denominator != 0.0f)
			inumerator /= denominator;
		return inumerator;
	}

	void Collide(
		const unsigned iterations,
		const CollisionInfo& colInfo,
		const TransformSystem& tSystem,
		const RigidbodySystem& rbSystem)
	{
		bool aStatic = tSystem.GetIsStatic(colInfo._entityIdA);
		bool bStatic = tSystem.GetIsStatic(colInfo._entityIdB);
		if (aStatic && bStatic)
			return;
		float jNormal, staticFriction, jTangent;
		glm::vec3 aToPoint, bToPoint, normalImpulse, tangent, tangentImpulse, pointVelocityA, pointVelocityB, relativeVelocity;
		const RigidbodyData* rbDataA = rbSystem.GetReserved(colInfo._entityIdA) ? &rbSystem._componentData.at(colInfo._entityIdA & tSystem.INDEX_BITS_SEQ) : nullptr;
		const RigidbodyData* rbDataB = rbSystem.GetReserved(colInfo._entityIdB) ? &rbSystem._componentData.at(colInfo._entityIdB & tSystem.INDEX_BITS_SEQ) : nullptr;
		float massA = rbDataA ? rbSystem.GetMass(colInfo._entityIdA) : 0.0f;
		float massB = rbDataB ? rbSystem.GetMass(colInfo._entityIdB) : 0.0f;
		float coefA = rbDataA ? rbSystem.GetCoef(colInfo._entityIdA) : 0.1f;
		float coefB = rbDataB ? rbSystem.GetCoef(colInfo._entityIdB) : 0.1f;
		glm::mat4 inerTensA = rbDataA ? InverseInertiaTensor(glm::vec3(0.5f, 0.5f, 0.5f), glm::toMat4(tSystem.GetRotation(colInfo._entityIdA)), massA) : glm::mat4(1.0f);
		glm::mat4 inerTensB = rbDataB ? InverseInertiaTensor(glm::vec3(0.5f, 0.5f, 0.5f), glm::toMat4(tSystem.GetRotation(colInfo._entityIdB)), massB) : glm::mat4(1.0f);
		float coef = -(1.0f + std::min(
			rbDataA ? rbSystem.GetCoef(colInfo._entityIdA) : 0.1f,
			rbDataB ? rbSystem.GetCoef(colInfo._entityIdB) : 0.1f));
		float inverseMass =
			(rbDataA ? 1.0f / rbSystem.GetMass(colInfo._entityIdA) : 0.0f) +
			(rbDataB ? 1.0f / rbSystem.GetMass(colInfo._entityIdB) : 0.0f);

		for (int i = 0; i < iterations; ++i)
		{
			for (int j = 0; j < colInfo._contactPoints.size(); ++j)
			{
				aToPoint = colInfo._contactPoints[j]._worldPoint - tSystem.GetPosition(colInfo._entityIdA);
				bToPoint = colInfo._contactPoints[j]._worldPoint - tSystem.GetPosition(colInfo._entityIdB);
				pointVelocityA = rbDataA ? rbSystem.GetVelocity(colInfo._entityIdA) + glm::cross(rbSystem.GetAngularVelocity(colInfo._entityIdA), aToPoint) : glm::vec3(0.0f, 0.0f, 0.0f);
				pointVelocityB = rbDataB ? rbSystem.GetVelocity(colInfo._entityIdB) + glm::cross(rbSystem.GetAngularVelocity(colInfo._entityIdB), bToPoint) : glm::vec3(0.0f, 0.0f, 0.0f);
				relativeVelocity = pointVelocityA - pointVelocityB;
				tangent = glm::normalize(relativeVelocity - glm::dot(relativeVelocity, colInfo._contactPoints[j]._normal));
				if (glm::dot(relativeVelocity, colInfo._contactPoints[j]._normal) > 0.0f)
					continue;
					// If relative velocity is pointing at the same direction as collision normal,
					// the point is already moving away and we don't want to collide it again

				jNormal = coef * glm::dot(relativeVelocity, colInfo._contactPoints[j]._normal)
					/ (inverseMass + glm::dot(colInfo._contactPoints[j]._normal,
						glm::cross(glm::vec3(inerTensA * glm::vec4(glm::cross(aToPoint, colInfo._contactPoints[j]._normal), 1.0f)), aToPoint)
						+ glm::cross(glm::vec3(inerTensB * glm::vec4(glm::cross(bToPoint, colInfo._contactPoints[j]._normal), 1.0f)), bToPoint)));
				jTangent = coef * glm::dot(relativeVelocity, tangent)
					/ (inverseMass + glm::dot(tangent,
						glm::cross(glm::vec3(inerTensA * glm::vec4(glm::cross(aToPoint, tangent), 1.0f)), aToPoint)
						+ glm::cross(glm::vec3(inerTensB * glm::vec4(glm::cross(bToPoint, tangent), 1.0f)), bToPoint)));
				jNormal /= colInfo._contactPoints.size();
				jTangent /= colInfo._contactPoints.size();
				normalImpulse = jNormal * colInfo._contactPoints[j]._normal;
				staticFriction = std::sqrt((rbDataA ? rbSystem.GetStaticFriction(colInfo._entityIdA) : 0.5f) * (rbDataB ? rbSystem.GetStaticFriction(colInfo._entityIdB) : 0.5f));
				staticFriction = glm::min(staticFriction, 1.0f);
				if (std::abs(jTangent) < (jNormal * staticFriction))
				{
					tangentImpulse = tangent * jTangent;
				}
				else
				{
					float dynamicFriction = std::sqrt((rbDataA ? rbSystem.GetDynamicFriction(colInfo._entityIdA) : 0.5f) * (rbDataB ? rbSystem.GetDynamicFriction(colInfo._entityIdB) : 0.5f));
					tangentImpulse = tangent * -jNormal * dynamicFriction;
				}

				if (rbDataA)
				{
					rbDataA->_velocity[colInfo._entityIdA & rbSystem.INDEX_BITS_COMP]
						+= normalImpulse / massA
						+ tangentImpulse / massA;
					rbDataA->_angularVelocity[colInfo._entityIdA & rbSystem.INDEX_BITS_COMP]
						+= glm::vec3(inerTensA * glm::vec4(glm::cross(aToPoint, normalImpulse), 1.0f))
						+ glm::vec3(inerTensA * glm::vec4(glm::cross(aToPoint, tangentImpulse), 1.0f));
				}
				if (rbDataB)
				{
					rbDataB->_velocity[colInfo._entityIdB & rbSystem.INDEX_BITS_COMP]
						-= normalImpulse / massB
						- tangentImpulse / massB;
					rbDataB->_angularVelocity[colInfo._entityIdB & rbSystem.INDEX_BITS_COMP]
						-= glm::vec3(inerTensB * glm::vec4(glm::cross(bToPoint, normalImpulse), 1.0f))
						- glm::vec3(inerTensB * glm::vec4(glm::cross(bToPoint, tangentImpulse), 1.0f));
				}
			}
		}
	}

	void CorrectPositions(const CollisionInfo& colInfo, TransformSystem& tSystem, const RigidbodySystem& rbSystem)
	{
		if (colInfo._contactPoints.size() == 0)
			return;
		const TransformData& tDataA = tSystem._componentData.at(colInfo._entityIdA & tSystem.INDEX_BITS_SEQ);
		const TransformData& tDataB = tSystem._componentData.at(colInfo._entityIdB & tSystem.INDEX_BITS_SEQ);
		const RigidbodyData* rbDataA = rbSystem.GetReserved(colInfo._entityIdA) ? &rbSystem._componentData.at(colInfo._entityIdA & tSystem.INDEX_BITS_SEQ) : nullptr;
		const RigidbodyData* rbDataB = rbSystem.GetReserved(colInfo._entityIdB) ? &rbSystem._componentData.at(colInfo._entityIdB & tSystem.INDEX_BITS_SEQ) : nullptr;
		glm::vec3 normal = colInfo._contactPoints[0]._normal;
		float depth = colInfo._contactPoints[0]._penetrationDepth;
		for (int i = 1; i < colInfo._contactPoints.size(); ++i)
		{
			if (colInfo._contactPoints[i]._penetrationDepth > depth)
			{
				depth = colInfo._contactPoints[i]._penetrationDepth;
				normal = colInfo._contactPoints[i]._normal;
			}
		}
		float inverseMassSum = 0.0f;
		inverseMassSum += rbDataA ? 1.0f / rbSystem.GetMass(colInfo._entityIdA) : 0.0f;
		inverseMassSum += rbDataB ? 1.0f / rbSystem.GetMass(colInfo._entityIdB) : 0.0f;
		float scalar = 0.8f * depth / inverseMassSum;
		glm::vec3 projection = scalar * normal;
		if (!tSystem.GetIsStatic(colInfo._entityIdA) && rbDataA)
			tSystem.Translate(colInfo._entityIdA, projection / rbDataA->_mass[colInfo._entityIdA & rbSystem.INDEX_BITS_COMP]);
		if (!tSystem.GetIsStatic(colInfo._entityIdB) && rbDataB)
			tSystem.Translate(colInfo._entityIdB, -projection / rbDataB->_mass[colInfo._entityIdB & rbSystem.INDEX_BITS_COMP]);
	}
	
	std::vector<ContactPoint> ContactPoints(
		const uint32_t entityIdA,
		const uint32_t entityIdB,
		const TransformSystem& tSystem,
		const RigidbodySystem& rbSystem,
		const ColliderSystem& cSystem,
		const MeshSystem& mSystem)
	{
		std::vector<ContactPoint> contactPoints;
		const TransformData& tDataA = tSystem._componentData.at(entityIdA & tSystem.INDEX_BITS_SEQ);
		const TransformData& tDataB = tSystem._componentData.at(entityIdB & tSystem.INDEX_BITS_SEQ);
		const ColliderData& cDataA = cSystem._componentData.at(entityIdA & cSystem.INDEX_BITS_SEQ);
		const ColliderData& cDataB = cSystem._componentData.at(entityIdB & cSystem.INDEX_BITS_SEQ);
		const uint32_t meshIndexA = cDataA._meshId[entityIdA & cSystem.INDEX_BITS_COMP];
		const uint32_t meshIndexB = cDataB._meshId[entityIdB & cSystem.INDEX_BITS_COMP];
		const MeshData& mDataA = mSystem._componentData.at(meshIndexA & mSystem.INDEX_BITS_SEQ);
		const MeshData& mDataB = mSystem._componentData.at(meshIndexB & mSystem.INDEX_BITS_SEQ);
		bool aIsRb = rbSystem.GetReserved(entityIdA);
		bool bIsRb = rbSystem.GetReserved(entityIdB);
		glm::vec3 positionA = tSystem.GetPosition(entityIdA);
		glm::vec3 positionB = tSystem.GetPosition(entityIdB);
		glm::vec3 velocityA = aIsRb ? rbSystem.GetVelocity(entityIdA) : glm::vec3(0.0f, 0.0f, 0.0f);
		glm::vec3 velocityB = bIsRb ? rbSystem.GetVelocity(entityIdB) : glm::vec3(0.0f, 0.0f, 0.0f);
		glm::vec3 angularVelocityA = aIsRb ? rbSystem.GetAngularVelocity(entityIdA) : glm::vec3(0.0f, 0.0f, 0.0f);
		glm::vec3 angularVelocityB = bIsRb ? rbSystem.GetAngularVelocity(entityIdB) : glm::vec3(0.0f, 0.0f, 0.0f);
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
		auto helper = [](
			bool invertNormal,
			std::vector<ContactPoint>& contactPoints,
			const MeshData& mDataA,
			const MeshData& mDataB,
			const uint32_t meshIndexA,
			const uint32_t meshIndexB,
			const glm::mat4& transformationA,
			const glm::mat4& transformationB,
			const glm::vec3 velocityA,
			const glm::vec3 velocityB,
			const glm::vec3 angularVelocityA,
			const glm::vec3 angularVelocityB,
			const glm::vec3 positionA,
			const glm::vec3 positionB)
		{
			glm::vec3 worldPoint, pointVelocity;
			std::optional<ContactPoint> contactPoint;
			for (int i = 0; i < mDataA._verticesLength[meshIndexA]; ++i)
			{
				worldPoint = transformationA * glm::vec4(mDataA._vertices[meshIndexA][i], 1.0f);
				contactPoint = GetContactPoint(
					worldPoint,
					velocityA + glm::cross(angularVelocityA, worldPoint - positionA),
					velocityB + glm::cross(angularVelocityB, worldPoint - positionB),
					mDataB._vertices[meshIndexB],
					mDataB._triangles[meshIndexB],
					mDataB._verticesLength[meshIndexB],
					mDataB._trianglesLength[meshIndexB],
					transformationB);
				if (contactPoint.has_value())
				{
					if (invertNormal)
						contactPoint.value()._normal *= -1.0f;
					contactPoints.push_back(std::move(contactPoint.value()));
				}
			}
		};
		helper(
			false,
			contactPoints,
			mDataA, mDataB,
			meshIndexA & mSystem.INDEX_BITS_COMP,
			meshIndexB & mSystem.INDEX_BITS_COMP,
			transformationA, transformationB,
			velocityA, velocityB,
			angularVelocityA, angularVelocityB,
			positionA, positionB);
		helper(
			true,
			contactPoints,
			mDataB, mDataA,
			meshIndexB & mSystem.INDEX_BITS_COMP,
			meshIndexA & mSystem.INDEX_BITS_COMP,
			transformationB, transformationA,
			velocityB, velocityA,
			angularVelocityB, angularVelocityA,
			positionB, positionA);
		return contactPoints;
	}

	std::vector<CollisionInfo> GenerateCollisions(const TransformSystem& tSystem, const RigidbodySystem& rbSystem, const ColliderSystem& cSystem, const MeshSystem& mSystem)
	{
		uint32_t entityIdA, entityIdB;
		std::vector<CollisionInfo> collisions;
		for (auto itA = cSystem._componentData.begin(); itA != cSystem._componentData.end(); ++itA)
		{
			for (int i = 0; i < cSystem.INDEX_BITS_COMP; ++i)
			{
				entityIdA = itA->first + i;
				if (!itA->second._reserved[entityIdA & cSystem.INDEX_BITS_COMP])
					continue;
				CollisionInfo colInfo;
				colInfo._entityIdA = entityIdA;
				for (auto itB(itA); itB != cSystem._componentData.end(); ++itB)
				{
					for (int j = itA == itB ? i + 1 : 0; j < cSystem.INDEX_BITS_COMP; ++j)
					{
						entityIdB = itB->first + j;
						colInfo._entityIdB = entityIdB;
						if (!itB->second._reserved[entityIdB & cSystem.INDEX_BITS_COMP])
							continue;
						if (!BoundsAreColliding(
							itA->second._worldBounds[entityIdA & cSystem.INDEX_BITS_COMP],
							itB->second._worldBounds[entityIdB & cSystem.INDEX_BITS_COMP]))
							continue;
						colInfo._contactPoints = ContactPoints(colInfo._entityIdA, colInfo._entityIdB, tSystem, rbSystem, cSystem, mSystem);
						collisions.push_back(colInfo);
					}
				}
			}
		}
		return collisions;
	}

	std::vector<CollisionInfo> ProcessRigidbodies(
		TransformSystem& tSystem,
		const RigidbodySystem& rbSystem,
		const ColliderSystem& colSystem,
		const MeshSystem& mSystem,
		float deltaTime,
		const glm::vec3& gravity)
	{
		// Apply air resistance and gravity
		for (auto const& [rbSeqIndex, rbData] : rbSystem._componentData)
		{
			for (uint32_t i = 0, entityId = rbSeqIndex; i < rbSystem.INDEX_BITS_COMP; ++i, ++entityId)
			{
				if (!rbData._reserved[i] || !tSystem.IsReserved(entityId))
					continue;
				if (tSystem._componentData[entityId & tSystem.INDEX_BITS_SEQ]._isStatic[entityId & tSystem.INDEX_BITS_COMP])
					continue;
				rbData._acceleration[i] -= rbData._dragCoefficient[i] * std::sqrt(rbData._velocity[i].x * rbData._velocity[i].x
					+ rbData._velocity[i].y * rbData._velocity[i].y
					+ rbData._velocity[i].z * rbData._velocity[i].z) * rbData._velocity[i];
				rbData._velocity[i] += rbData._acceleration[i] * deltaTime;
				if (rbData._useGravity[i])
					rbData._velocity[i] += gravity * deltaTime;
			}
		}

		// Handle collisions
		std::vector<CollisionInfo> collisions = GenerateCollisions(tSystem, rbSystem, colSystem, mSystem);
		for (const CollisionInfo& colInfo : collisions)
			Collide(1, colInfo, tSystem, rbSystem);
		for (const CollisionInfo& colInfo : collisions)
			CorrectPositions(colInfo, tSystem, rbSystem);

		// Update rigidbodies
		const RbConstraints* constraints;
		for (auto const& [rbSeqIndex, rbData] : rbSystem._componentData)
		{
			for (uint32_t i = 0, entityId = rbSeqIndex; i < rbSystem.INDEX_BITS_COMP; ++i, ++entityId)
			{
				constraints = &rbData._constraints[i];
				if (!rbData._reserved[i] || !tSystem.IsReserved(entityId))
					continue;
				if (tSystem._componentData[entityId & tSystem.INDEX_BITS_SEQ]._isStatic[entityId & tSystem.INDEX_BITS_COMP])
					continue;
				tSystem.Translate(entityId, rbData._velocity[i] * deltaTime);
				
				if (!constraints->_freezeRotationX)
					tSystem.Rotate(entityId, rbData._angularVelocity[i].x * deltaTime, glm::vec3(1.0f, 0.0f, 0.0f));
				if (!constraints->_freezeRotationY)
					tSystem.Rotate(entityId, rbData._angularVelocity[i].y * deltaTime, glm::vec3(0.0f, 1.0f, 0.0f));
				if (!constraints->_freezeRotationZ)
					tSystem.Rotate(entityId, rbData._angularVelocity[i].z * deltaTime, glm::vec3(0.0f, 0.0f, 1.0f));
			}
		}

		return collisions;
	}
}