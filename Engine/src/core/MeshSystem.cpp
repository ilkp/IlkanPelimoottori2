#include "ComponentSystem.h"

namespace idop
{
	void MeshSystem::Reserve(uint32_t entityId)
	{
		auto it = _componentData.find(entityId & INDEX_BITS_SEQ);
		if (it == _componentData.end())
		{
			it = _componentData.insert(std::make_pair(entityId & INDEX_BITS_SEQ, MeshData())).first;
			it->second.Allocate(INDEX_BITS_COMP + 1);
		}
		it->second._reserved[entityId & INDEX_BITS_COMP] = true;
	}

	void MeshSystem::Release(uint32_t entityId)
	{
		auto it = _componentData.find(entityId & INDEX_BITS_SEQ);
		if (it != _componentData.end())
			it->second._reserved[entityId & INDEX_BITS_COMP] = false;
	}

	void MeshSystem::Identity(uint32_t entityId)
	{
		auto it = _componentData.find(entityId & INDEX_BITS_SEQ);
		if (it == _componentData.end())
			it = NCReserve(entityId);
		it->second.Identity(entityId & INDEX_BITS_COMP);
	}

	void MeshSystem::SetMesh(uint32_t entityId, const Mesh& mesh)
	{
		uint32_t compIndex = entityId & INDEX_BITS_COMP;
		auto it = _componentData.find(entityId & INDEX_BITS_SEQ);
		if (it == _componentData.end())
			it = NCReserve(entityId);
		MeshData& meshData = it->second;
		delete[](meshData._vertices[compIndex]);
		delete[](meshData._triangles[compIndex]);
		delete[](meshData._uvs[compIndex]);
		meshData._verticesLength[compIndex] = mesh._verticesLength;
		meshData._trianglesLength[compIndex] = mesh._trianglesLength;
		meshData._uvsLength[compIndex] = mesh._uvsLength;
		meshData._vertices[compIndex] = new glm::vec3[mesh._verticesLength];
		meshData._triangles[compIndex] = new int[mesh._trianglesLength];
		meshData._uvs[compIndex] = new glm::vec2[mesh._uvsLength];
		for (int i = 0; i < mesh._verticesLength; ++i)
			meshData._vertices[compIndex][i] = mesh._vertices[i];
		for (int i = 0; i < mesh._trianglesLength; ++i)
			meshData._triangles[compIndex][i] = mesh._triangles[i];
		for (int i = 0; i < mesh._uvsLength; ++i)
			meshData._uvs[compIndex][i] = mesh._uvs[i];
		meshData._bounds[compIndex]._min.x = mesh._bounds._min.x;
		meshData._bounds[compIndex]._min.y = mesh._bounds._min.y;
		meshData._bounds[compIndex]._min.z = mesh._bounds._min.z;
		meshData._bounds[compIndex]._max.x = mesh._bounds._max.x;
		meshData._bounds[compIndex]._max.y = mesh._bounds._max.y;
		meshData._bounds[compIndex]._max.z = mesh._bounds._max.z;
	}

	std::unordered_map<uint32_t, MeshData>::iterator MeshSystem::NCReserve(uint32_t entityId)
	{
		auto it = _componentData.insert(std::make_pair(entityId & INDEX_BITS_SEQ, MeshData())).first;
		it->second.Allocate(INDEX_BITS_COMP + 1);
		it->second._reserved[entityId & INDEX_BITS_COMP] = true;
		return it;
	}
}