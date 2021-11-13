#pragma once
#include <cstdint>

namespace idop
{
	class Entity
	{
	public:
		const uint32_t _entityId;

		Entity(uint32_t entityId) : _entityId(entityId) {}
	};

	class EntityBlock
	{
	public:
		uint32_t _idBegin;
		uint32_t _size;
	};
}