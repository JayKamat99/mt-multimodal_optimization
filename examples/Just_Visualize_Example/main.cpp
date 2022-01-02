#include <Kin/viewer.h>

int main(int argc, char** argv)
{
	const char* filename = argv[1];
	rai::Configuration C;
    C.addFile(filename);
	C.watch(true);
}