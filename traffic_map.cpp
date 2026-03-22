// traffic_map.cpp : Defines the entry point for the application.
//

#include "traffic_map.hpp"

using namespace std;
using namespace geodesk;
using namespace traffic_sim;
using namespace config;
using namespace pathfinding;


/*
Node* find(Features fs, int64_t id) {
	for (Node f : fs.nodes()) {
		if (f.id() == id) return &f;
	}
	return nullptr;
}
*/

int main(int argc, char *argv[])
{

	ios_base::sync_with_stdio(false);
	cin.tie(nullptr);
	cout.tie(nullptr);
	cout << setprecision(9);

	cout << "Running in ";
	flush(cout);
	system("pwd");
	cout << endl;

	string config_path = "sim.config";

	for(int i = 0; i < argc; i++) {
		char* curr = argv[i];
		if(curr[0] == '\0') continue;

		if(curr[0] == '-') {
			switch(curr[1]) {
				case '\0': break;
				case 'c': if(i+1 < argc) config_path = argv[++i]; break; // config
				case 'i': if(i+1 < argc) {IN_PATH = argv[++i]; cmd_override_in_path = true;} break; // input file
				case 'o': if(i+1 < argc) {OUT_VISUALISATION_PATH = argv[++i]; cmd_override_out_path = true;} break; // output file
				case 'g': if(i+1 < argc) {OUT_GENERATED_PATH = argv[++i]; cmd_override_gen_path = true;} break; // generated file (ignore if using input)
				case 'l': if(i+1 < argc) {GOL_PATH = argv[++i]; cmd_override_gol_path = true;} break; // gol file (ignore if using input)
				case 'q': if(i+1 < argc) {GOL_QUERY = argv[++i]; cmd_override_gol_query = true;} break; // gol query (ignore if using input)
				case 'v': VERBOSE = true; break;
			}
		}
	}

	parse_config(config_path);
	init_io_files();

	if(using_input) {
		parse_input();
	} else {
		generate_data();
	}
	// DONE: Add config file support; check command line arguments and parse input files accordingly
	// DONE: Generate generated objects file
	// DONE: Add input file support
	// TODO: Add population target support using binary search
	// TODO: Add different density coefficients for different building types
	// DONE 80%: Make work assignment based on people, not on workplaces


	auto t1 = chrono::high_resolution_clock::now();

	for(double tick = 0.0; tick < SIM_LENGTH; tick += TIME_STEP) {
		sim_tick(TIME_STEP);
	}

	const auto t2 = chrono::high_resolution_clock::now();
	cout << "SIMULATION FINISHED" << endl;
	cout << duration_cast<chrono::milliseconds>(t2 - t1) << " milliseconds" << endl;


	// while(!events.empty()) {
	// 	event e = events.top();
	// 	events.pop();
	// 	cout << "time - " << e.time << "; type - " << e.t << endl;
	// 	e.call();
	// }

	out_file.close();
	return 0;
}
