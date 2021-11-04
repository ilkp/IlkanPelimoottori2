#include <glm/gtx/quaternion.hpp>
#include "ComponentSystem.h"

namespace idop
{
	TransformSystem::TransformSystem()
	{
		seqLength = seqBytes / (uint32_t)sizeof(glm::quat);
		seqIndexLength = (uint32_t)std::ceil(std::log2((double)seqLength) + 1.0);
		componentIndexBits = ~(uint32_t(0)) >> seqIndexLength;
		seqIndexBits = ~componentIndexBits;
	}

	void TransformSystem::Reserve(uint32_t entityId)
	{
		std::unordered_map<uint32_t, TransformData>::iterator it = _componentData.find(entityId & seqIndexBits);
		if (it == _componentData.end())
		{
			it = _componentData.insert({ entityId & seqIndexBits, TransformData() }).first;
			(*it).second.Allocate(seqLength);
			_MVP.insert({ entityId & seqIndexBits, new glm::mat4[seqLength] });
		}
		(*it).second._reserved[entityId & componentIndexBits] = true;
	}

	void idop::TransformSystem::Identity(uint32_t entityId)
	{
		uint32_t componentIndex = entityId & componentIndexBits;
		std::unordered_map<uint32_t, TransformData>::iterator it = _componentData.find(entityId & seqIndexBits);
		if (it == _componentData.end())
			it = NCReserve(entityId);
		TransformData& cData = (*it).second;
		cData._isStatic[componentIndex] = false;
		cData._scale[componentIndex] = glm::mat4(1.0f);
		cData._translation[componentIndex] = glm::mat4(1.0f);
		cData._quaternion[componentIndex] = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
	}

	void TransformSystem::CalculateMVP()
	{
		uint32_t componentIndex;
		for (auto const& [entityId, tData] : _componentData)
		{
			componentIndex = entityId & componentIndexBits;
			if (tData._reserved)
				_MVP[entityId & seqIndexBits][componentIndex] = tData._translation[componentIndex] * glm::toMat4(tData._quaternion[componentIndex]) * tData._scale[componentIndex];
		}
	}

	void TransformSystem::SetScale(uint32_t entityId, const glm::vec3& vector)
	{
		glm::mat4& target = _componentData[entityId & seqIndexBits]._scale[entityId & componentIndexBits];
		target[0][0] = vector.x;
		target[1][1] = vector.y;
		target[2][2] = vector.z;
	}

	void TransformSystem::SetScale(uint32_t entityId, float x, float y, float z)
	{
		glm::mat4& target = _componentData[entityId & seqIndexBits]._scale[entityId & componentIndexBits];
		target[0][0] = x;
		target[1][1] = y;
		target[2][2] = z;
	}

	void TransformSystem::SetPosition(uint32_t entityId, const glm::vec3& vector)
	{
		glm::mat4& target = _componentData[entityId & seqIndexBits]._translation[entityId & componentIndexBits];
		target[3][0] = vector.x;
		target[3][1] = vector.y;
		target[3][2] = vector.z;
	}

	void TransformSystem::SetPosition(uint32_t entityId, float x, float y, float z)
	{
		glm::mat4& target = _componentData[entityId & seqIndexBits]._translation[entityId & componentIndexBits];
		target[3][0] = x;
		target[3][1] = y;
		target[3][2] = z;
	}

	void TransformSystem::Translate(uint32_t entityId, const glm::vec3& vector)
	{
		glm::mat4& target = _componentData[entityId & seqIndexBits]._translation[entityId & componentIndexBits];
		target[3][0] += vector.x;
		target[3][1] += vector.y;
		target[3][2] += vector.z;
	}

	void TransformSystem::Translate(uint32_t entityId, float x, float y, float z)
	{
		glm::mat4& target = _componentData[entityId & seqIndexBits]._translation[entityId & componentIndexBits];
		target[3][0] += x;
		target[3][1] += y;
		target[3][2] += z;
	}

	void TransformSystem::Rotate(uint32_t entityId, const glm::vec3& eulerAngles)
	{
		_componentData[entityId & seqIndexBits]._quaternion[entityId & componentIndexBits] *= glm::quat(eulerAngles);
	}

	void TransformSystem::Rotate(uint32_t entityId, float x, float y, float z)
	{
		_componentData[entityId & seqIndexBits]._quaternion[entityId & componentIndexBits] *= glm::quat(glm::vec3(x, y, z));
	}

	glm::vec3 TransformSystem::Position(uint32_t entityId) const
	{
		glm::vec3 rValue;
		const glm::mat4& source = _componentData.at(entityId & seqIndexBits)._translation[entityId & componentIndexBits];
		rValue.x = source[3][0];
		rValue.y = source[3][0];
		rValue.z = source[3][0];
		return rValue;
	}

	glm::vec3 TransformSystem::Scale(uint32_t entityId) const
	{
		glm::vec3 rValue;
		const glm::mat4& source = _componentData.at(entityId & seqIndexBits)._scale[entityId & componentIndexBits];
		rValue.x = source[0][0];
		rValue.y = source[1][1];
		rValue.z = source[2][2];
		return rValue;
	}
}