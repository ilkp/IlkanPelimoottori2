#include "ComponentSystem.h"

namespace idop
{
	void MeshSystem::Reserve(uint32_t entityId)
	{
		auto it = _componentData.find(entityId & INDEX_BITS_SEQ);
		if (it == _componentData.end())
		{
			it = _componentData.insert(std::make_pair(entityId & INDEX_BITS_SEQ, std::move(MeshData()))).first;
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
		uint32_t componentIndex = entityId & INDEX_BITS_COMP;
		std::unordered_map<uint32_t, MeshData>::iterator it = _componentData.find(entityId & INDEX_BITS_SEQ);
		if (it == _componentData.end())
			it = NCReserve(entityId);
		it->second.Identity(componentIndex);
	}

	void MeshSystem::SetMesh(uint32_t entityId, const Mesh& mesh)
	{
		uint32_t compIndex = entityId & INDEX_BITS_COMP;
		std::unordered_map<uint32_t, MeshData>::iterator it = _componentData.find(entityId & INDEX_BITS_SEQ);
		if (it == _componentData.end())
			it = NCReserve(entityId);
		delete[](it->second._vertices[compIndex]);
		delete[](it->second._triangles[compIndex]);
		delete[](it->second._uvs[compIndex]);

		it->second._verticesLength[compIndex] = mesh._verticesLength;
		it->second._trianglesLength[compIndex] = mesh._trianglesLength;
		it->second._uvsLength[compIndex] = mesh._uvsLength;

		it->second._vertices[compIndex] = new glm::vec3[mesh._verticesLength];
		it->second._triangles[compIndex] = new int[mesh._trianglesLength];
		it->second._uvs[compIndex] = new glm::vec2[mesh._uvsLength];
		for (int i = 0; i < mesh._verticesLength; ++i)
			it->second._vertices[compIndex][i] = mesh._vertices[i];
		for (int i = 0; i < mesh._trianglesLength; ++i)
			it->second._triangles[compIndex][i] = mesh._triangles[i];
		for (int i = 0; i < mesh._uvsLength; ++i)
			it->second._uvs[compIndex][i] = mesh._uvs[i];

		it->second._bounds[compIndex]._min.x = mesh._bounds._min.x;
		it->second._bounds[compIndex]._min.y = mesh._bounds._min.y;
		it->second._bounds[compIndex]._min.z = mesh._bounds._min.z;
		it->second._bounds[compIndex]._max.x = mesh._bounds._max.x;
		it->second._bounds[compIndex]._max.y = mesh._bounds._max.y;
		it->second._bounds[compIndex]._max.z = mesh._bounds._max.z;
	}

	std::unordered_map<uint32_t, MeshData>::iterator MeshSystem::NCReserve(uint32_t entityId)
	{
		std::unordered_map<uint32_t, MeshData>::iterator it = _componentData.insert(std::make_pair(entityId & INDEX_BITS_SEQ, std::move(MeshData()))).first;
		it->second.Allocate(INDEX_BITS_COMP + 1);
		it->second._reserved[entityId & INDEX_BITS_COMP] = true;
		return it;
	}
}