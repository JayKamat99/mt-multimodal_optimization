#include <Kin/viewer.h>
#include <KOMO/komo.h>

int main(int argc, char** argv)
{
	const char* filename = argv[1];
	rai::Configuration C;
    C.addFile(filename);
	std::string filename_s(filename);
	arr start = C.getJointState();
	std::cout << "start: " << start << std::endl;
	arr goal;
	if (filename_s == "../examples/Models/1_kuka_shelf.g"){
		start = {0.718274, -0.388218, -1.83428, -0.971166, -0.322495, 0.284864, 0.00191594};
		goal = {0.560603, -1.05486, -1.71583, -1.68994, -0.051403, 0.266908, 0.000754904};
		// goal = {1.05248, -0.982536, -1.70613, -0.816571, -0.0301295, 0.0453272, 0.000650022};
	}
	else if (filename_s == "../examples/Models/2_Two_Pandas.g"){
		goal =  {-0.24272, 1.2727, 0.131396, -1.10928, -0.795822, 3.0705, 0.00170469, -0.248182, 1.29032, 0.143824, -1.09332, -0.813384, 3.08292, -0.0161561};
	}
	else if (filename_s == "../examples/Models/3_TwoMobileManipulators.g"){
		goal = {-0.555762, 0.000540429, 1.57074, 0.00188429, 0.764456, -0.000160723, -2.21317, -0.00321155, 2.28468, -0.000332939, 0.555647, -0.00012235, -1.57154, 0.00161455, 0.764632, -0.000429018, -2.21257, 0.00103216, 2.28374, 0.00102819};
	}	
	else if (filename_s == "../examples/Models/4_kuka_box.g"){
		goal = {0.00241061, 0.872391, -0.00871117, -2.09305, -0.0101917, 0.314683, 0.000963466};
	}
	else if (filename_s == "../examples/Models/5_disc_obstacle.g"){
		goal = {0.8,-0.25,0};
	}
	else if (filename_s == "../examples/Models/6_rectangle_opening.g"){
		goal = {0.7,0,0};
	}
	else if (filename_s == "../examples/Models/7_disc_rooms.g"){
		goal = {0.7,0,0};
	}
	else if (filename_s == "../examples/Models/8_TwoMobileManipulators_hard.g"){
		goal = {-0.555762, 0.000540429, 1.57074, 0.00188429, 0.764456, -0.000160723, -2.21317, -0.00321155, 2.28468, -0.000332939, 0.555647, -0.00012235, -1.57154, 0.00161455, 0.764632, -0.000429018, -2.21257, 0.00103216, 2.28374, 0.00102819};
	}	
	else if (filename_s == "../examples/Models/9_TwoMobileRobots_hard.g"){
		std::cout << "do I reach here?" << std::endl;
		goal = {-0.755762, 0.000540429, 1.57074, 0.755647, -0.00012235, -1.57154};
	}
	else if (filename_s == "../examples/Models/10_MobileManipulator.g"){
		goal = {-0.555762, 0.000540429, 1.57074, 0.00188429, 0.764456, -0.000160723, -2.21317, -0.00321155, 2.28468, -0.000332939};
	}
	std::cout << goal << std::endl;
	// for(auto f: C.frames){
	// 	arr c = f->getShape().mesh().C;
	// 	std::cout << c.N << std::endl;
	// 	if (c.N ==3)
	// 	{
	// 		std::cout << f->getShape()._type != rai::ST_none << std::endl;
	// 		c.append(0.5);
	// 	}
	// 	f->setColor(c);
	// 	std::cout << "after" << c.N << std::endl;

	// }
	for (auto f : C.frames) {
		auto &ss = f->shape;
		if (ss) {
			arr c = f->getShape().mesh().C;
			if (c.N == 3){
				c.append(0.5);
				f->setColor(c);
			}
		}
	}
	std::cout << "I reach here" << std::endl;
	// C.setJointState(goal);
	C.addFile(filename);
	filename_s.erase(0,19);
	filename_s.erase(filename_s.length()-2);
	arr stateSet;
	stateSet.append(goal);
	stateSet.append(start);
	C.setJointState(stateSet);
	C.watch(true);
	return 0;

    KOMO komo;
    komo.verbose = 0;
    komo.setModel(C, true);
    
    komo.setTiming(1., 1, 1, 1);
	komo.add_qControlObjective({}, 1, 1.);

    komo.run_prepare(0);
	komo.plotTrajectory();
	std::string SaveToPath = "Images/" + filename_s + "/";

	rai::ConfigurationViewer V;
	V.setPath(C, komo.x);
	V.playVideo(false, 1., SaveToPath.c_str());
}