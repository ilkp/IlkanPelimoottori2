#include <glm/gtx/quaternion.hpp>
#include <math.h>
#include <iostream>
#include "ComponentSystem.h"
#include <future>

namespace idop
{
	bool TransformSystem::IsReserved(uint32_t entityId) const
	{
		auto it = _componentData.find(entityId & INDEX_BITS_SEQ);
		if (it == _componentData.end())
			return false;
		return it->second._reserved[entityId & INDEX_BITS_COMP];
	}

	bool TransformSystem::IsStatic(uint32_t entityId) const
	{
		auto it = _componentData.find(entityId & INDEX_BITS_SEQ);
		if (it == _componentData.end())
			return false;
		return it->second._isStatic[entityId & INDEX_BITS_COMP];
	}

	void TransformSystem::CalculateMVP()
	{
		//for (auto& [seqId, tData] : _componentData)
		//{
		//	for (uint32_t i = 0; i < INDEX_BITS_COMP; ++i)
		//	{
		//		if (tData._reserved[i])
		//			tData._modelViewProjection[i] = tData._translation[i] * glm::toMat4(tData._quaternion[i]) * tData._scale[i];
		//	}
		//}

		uint32_t subSeqLength = std::ceil(INDEX_BITS_COMP / _threadPool.size());
		std::vector<std::future<void>> results(_threadPool.size());
		for (auto it = _componentData.begin(); it != _componentData.end(); ++it)
		{
			TransformData* tData = &it->second;
			for (int i = 0; i < _threadPool.size(); ++i)
			{
				uint32_t start = i * subSeqLength;
				results[i] = _threadPool.push([this, tData, start, subSeqLength](int id) { this->CalculateMVPSeq(tData, start, subSeqLength); });
			}
			for (int i = 0; i < results.size(); ++i)
				results[i].get();
		}
	}

	void TransformSystem::CalculateMVPSeq(TransformData* tData, uint32_t start, uint32_t length)
	{
		for (uint32_t i = start; i < start + length && i < INDEX_BITS_COMP; ++i)
		{
			if (tData->_reserved[i])
				tData->_modelViewProjection[i] = tData->_translation[i] * glm::toMat4(tData->_quaternion[i]) * tData->_scale[i];
		}
	}

	void TransformSystem::SetScale(uint32_t entityId, const glm::vec3& vector)
	{
		glm::mat4& target = _componentData[entityId & INDEX_BITS_SEQ]._scale[entityId & INDEX_BITS_COMP];
		target[0][0] = vector.x;
		target[1][1] = vector.y;
		target[2][2] = vector.z;
	}

	void TransformSystem::SetScale(uint32_t entityId, float x, float y, float z)
	{
		glm::mat4& target = _componentData[entityId & INDEX_BITS_SEQ]._scale[entityId & INDEX_BITS_COMP];
		target[0][0] = x;
		target[1][1] = y;
		target[2][2] = z;
	}

	void TransformSystem::SetPosition(uint32_t entityId, const glm::vec3& vector)
	{
		glm::mat4& target = _componentData[entityId & INDEX_BITS_SEQ]._translation[entityId & INDEX_BITS_COMP];
		target[3][0] = vector.x;
		target[3][1] = vector.y;
		target[3][2] = vector.z;
	}

	void TransformSystem::SetPosition(uint32_t entityId, float x, float y, float z)
	{
		glm::mat4& target = _componentData[entityId & INDEX_BITS_SEQ]._translation[entityId & INDEX_BITS_COMP];
		target[3][0] = x;
		target[3][1] = y;
		target[3][2] = z;
	}

	void TransformSystem::SetRotation(uint32_t entityId, const glm::quat& rotation)
	{
		_componentData[entityId & INDEX_BITS_SEQ]._quaternion[entityId & INDEX_BITS_COMP] = glm::quat(rotation);
	}

	void TransformSystem::SetRotation(uint32_t entityId, const glm::mat4& rotation)
	{
		_componentData[entityId & INDEX_BITS_SEQ]._quaternion[entityId & INDEX_BITS_COMP] = glm::quat_cast(rotation);
	}

	void TransformSystem::Translate(uint32_t entityId, const glm::vec3& vector)
	{
		glm::mat4& target = _componentData[entityId & INDEX_BITS_SEQ]._translation[entityId & INDEX_BITS_COMP];
		target[3][0] += vector.x;
		target[3][1] += vector.y;
		target[3][2] += vector.z;
	}

	void TransformSystem::Translate(uint32_t entityId, float x, float y, float z)
	{
		glm::mat4& target = _componentData[entityId & INDEX_BITS_SEQ]._translation[entityId & INDEX_BITS_COMP];
		target[3][0] += x;
		target[3][1] += y;
		target[3][2] += z;
	}

	void TransformSystem::Rotate(uint32_t entityId, float angle, const glm::vec3& axis)
	{
		_componentData[entityId & INDEX_BITS_SEQ]._quaternion[entityId & INDEX_BITS_COMP]
			= glm::rotate(
				_componentData[entityId & INDEX_BITS_SEQ]._quaternion[entityId & INDEX_BITS_COMP],
				angle,
				axis);
	}

	//void TransformSystem::Rotate(uint32_t entityId, float x, float y, float z)
	//{
	//	_componentData[entityId & INDEX_BITS_SEQ]._quaternion[entityId & INDEX_BITS_COMP] *= glm::quat(glm::vec3(x, y, z));
	//}

	glm::vec3 TransformSystem::GetPosition(uint32_t entityId) const
	{
		glm::vec3 rValue;
		const glm::mat4& source = _componentData.at(entityId & INDEX_BITS_SEQ)._translation[entityId & INDEX_BITS_COMP];
		rValue.x = source[3][0];
		rValue.y = source[3][1];
		rValue.z = source[3][2];
		return rValue;
	}

	glm::mat4 TransformSystem::GetPositionMat(uint32_t entityId) const
	{
		return _componentData.at(entityId & INDEX_BITS_SEQ)._translation[entityId & INDEX_BITS_COMP];
	}

	glm::vec3 TransformSystem::GetScaleVector(uint32_t entityId) const
	{
		glm::vec3 rValue;
		const glm::mat4& source = _componentData.at(entityId & INDEX_BITS_SEQ)._scale[entityId & INDEX_BITS_COMP];
		rValue.x = source[0][0];
		rValue.y = source[1][1];
		rValue.z = source[2][2];
		return rValue;
	}

	glm::quat TransformSystem::GetRotation(uint32_t entityId) const
	{
		return _componentData.at(entityId & INDEX_BITS_SEQ)._quaternion[entityId & INDEX_BITS_COMP];
	}

	bool TransformSystem::GetIsStatic(uint32_t entityId) const
	{
		return _componentData.at(entityId & INDEX_BITS_SEQ)._isStatic[entityId & INDEX_BITS_COMP];
	}
}