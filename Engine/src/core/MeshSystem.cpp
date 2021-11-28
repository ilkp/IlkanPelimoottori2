#include "ComponentSystem.h"

namespace idop
{
	void MeshSystem::SetMesh(uint32_t entityId, const Mesh& mesh)
	{
		uint32_t compIndex = entityId & INDEX_BITS_COMP;
		auto it = _componentData.find(entityId & INDEX_BITS_SEQ);
		if (it == _componentData.end())
			it = NoCheckReserve(entityId);
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
}