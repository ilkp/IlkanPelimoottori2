#pragma once
#include <cstdint>

namespace idop
{
	class EntitySystem
	{
	public:
		uint32_t Next() { return runningId++; }
	private:
		uint32_t runningId = 0;
	};
}