#include "ComponentSystem.h"

namespace idop
{
	TransformSystem::TransformSystem()
	{
		seqLength = seqBytes / Size();
		seqIndexLength = (uint32_t)std::ceil(std::log2((double)seqLength) + 1.0);
		componentIndexBits = ~(uint32_t(0)) >> seqIndexLength;
		seqIndexBits = ~componentIndexBits;
	}

	void TransformSystem::Reserve(uint32_t id)
	{
		auto it = _componentData.find(id & seqIndexBits);
		if (it == _componentData.end())
		{
			it = _componentData.insert({ id & seqIndexBits, TransformData() }).first;
			(*it).second.Allocate(seqLength);
		}
		(*it).second._reserved[id & componentIndexBits] = true;
	}
}