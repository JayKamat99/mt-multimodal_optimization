#include <Kin/viewer.h>
#include <KOMO/komo.h>

int main(int argc, char** argv)
{
	const char* filename = argv[1];
	rai::Configuration C;
    C.addFile(filename);
	C.watch(true);

    KOMO komo;
    komo.verbose = 0;
    komo.setModel(C, true);
    
    komo.setTiming(1., 1, 1, 1);
	komo.add_qControlObjective({}, 1, 1.);

    komo.run_prepare(0);
	komo.plotTrajectory();
	std::string SaveToPath = "z.vid/examples/";

	rai::ConfigurationViewer V;
	V.setPath(C, komo.x, "result", true);
	V.playVideo(true, 1., SaveToPath.c_str());
}