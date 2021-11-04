#include "ComponentSystem.h"

int main()
{
	idop::TransformSystem transformSystem;
	//transformSystem.Print();
	std::cout << sizeof(glm::quat) << "  " << sizeof(glm::mat4);

	return 0;
}