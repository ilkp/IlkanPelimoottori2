#pragma once
#include <cstdint>

namespace idop
{
	class Entity
	{
	public:
		const uint32_t _id;
	};

	class EntityBlock
	{
	public:
		uint32_t _idBegin;
		uint32_t _size;
	};
}