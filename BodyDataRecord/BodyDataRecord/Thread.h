//#include "SystemDependency.hpp"

#include <process.h>
#include <windows.h>

class Thread {
private:

public:
	void StartThread(unsigned (__stdcall *_StartAddress) (void *), void *Argument);
};
