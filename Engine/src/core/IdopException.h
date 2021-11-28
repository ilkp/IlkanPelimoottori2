#pragma once
#include <stdexcept>

namespace idop
{
	class NotImplementedException : public std::exception
	{
	private:
		const char* _message;

	public:
		NotImplementedException(const char* message) : _message(message) {}

		const char* what() const throw()
		{
			return _message;
		}
	};
}