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
		//parse input
	} else {
		//generate data
	}
	// DONE: Add config file support; check command line arguments and parse input files accordingly
	// TODO: Generate generated objects file
	// TODO: Add input file support
	// TODO: Add traffic light file support
	// TODO: Add population target support using binary search
	// TODO: Add different density coefficients for different building types
	// TODO: Make work assignment based on people, not on workplaces

	Features features(GOL_PATH.c_str());

	string query = GOL_QUERY;
	//query = "a[admin_level=8][int_name=" + query + "]";

	cout << "querying " << query << endl;
	Feature city = features(query.c_str()).one(); // whole city
	Features features_in_city = features.intersecting(city);
	// temp for testing in vuzrazhdane
	/*
	const Feature* vuzrazhdane;

	for (const Feature& f : features_in_varna) {
		if (f.id() == 44006186) { vuzrazhdane = &f; break; }
	}
	features_in_varna = features_in_varna.intersecting(*vuzrazhdane);
	*/
	//

	Ways roads = features_in_city.ways(WAYS_QUERY.c_str());
	Relations bus_routes = features_in_city.relations("r[route=bus][type=route]");
	Ways residential_areas = features_in_city.ways("a[landuse=residential]");
	Ways parkings = features_in_city.ways("a[amenity=parking]");



	// building graph
	graph_init(roads);

	if(using_gen) {
		gen_file << setprecision(5);
		for(const auto& [p, vs] : graph) {
			gen_file << p << ": ";
			for(const segment& s : vs) {
				gen_file << (point)s.start << ' ' << (point)s.end << ' ' << s.length << ' ' << s.max_speed << "; ";
			}
			gen_file << endl;
		}
	}

	rng::randomize();
	// cout << "STARTING PATHFINDING\n";
	// int iterations = 1;
	// if(argc > 2) iterations = atoi(argv[2]);
	// // int progress = 0;
	// // for(point start : all_nodes) {
	// // 	if(progress++ < 100000) continue;
	// // 	cout << "progress: " << progress << " / " << all_nodes.size() << endl;
	// // 	cout << "Start node: " << coord_from_point(start) << endl;
	// // 	point end = all_nodes[rng::random_int(0, all_nodes.size() - 1)];
	// // 	cout << "End node: " << coord_from_point(end) << endl;
	// // 	vector<segment*> path;
	// // 	cout << (find_path(start, end, path) ? "SUCCESS\n" : "FAILED\n");
	// // 	progress++;
 // //
	// // }
	// for(int i = 0; i < iterations; i++) {
 //
	// 	point start = all_nodes[rng::random_int(0, all_nodes.size() - 1)];
	// 	cout << "Start node: " << coord_from_point(start) << endl;
	// 	point end = all_nodes[rng::random_int(0, all_nodes.size() - 1)];
	// 	cout << "End node: " << coord_from_point(end) << endl;
 //
	// 	deque<segment*> path;
 //
	// 	cout << (find_path(start, end, path) ? "SUCCESS\n" : "FAILED\n");
 //
	// 	//cout << "Path segments: " << endl;
	// 	for (segment* seg : path) {
	// 		//cout << seg.start << " -> " << seg.end << " (length: " << seg.length << " meters)" << endl;
	// 		output_segment(out_file, *seg, 'c');
	// 	}
	// }
	// out_file.close();

	// residential buildings
	(void)(VERBOSE && cout << "PARSING RESIDENTIAL BUILDINGS" << endl);
	auto t1 = chrono::high_resolution_clock::now();

	assert(is_init);
	int count = 0, ind = 0; //building count (current index)
	for (const Way& a : residential_areas) {
		//cout << "Residential Area: " << a["name"] << endl;
		Ways res_buildings_in_area = features_in_city.ways(RESIDENTIAL_BUILDINGS_QUERY.c_str()).intersecting(a);
		for (const Way& f : res_buildings_in_area) {
			// lower density for houses
			residential_buildings[a.id()].emplace_back(f, building::RESIDENTIAL, ((f["building"] == "house") ? 0.002 : 0.0), ind++);
			if(using_gen) gen_file << residential_buildings[a.id()].back() << endl;
			//cout << "Residential Building: " << b.name << ", Capacity: " << b.capacity << ", Location: " << coord_from_point(b.location) << endl;
			ind++;
			count += residential_buildings[a.id()].back().capacity;
		}
	}

	auto t2 = chrono::high_resolution_clock::now();
	(void)(VERBOSE && cout << "Total residents: " << count << endl);
	(void)(VERBOSE && cout << "DONE" << endl);
	(void)(VERBOSE && cout << duration_cast<chrono::milliseconds>(t2 - t1) << " milliseconds" << endl);

	people_init();


	// workplaces
	Features workplace_features = features_in_city(WORK_BUILDINGS_QUERY.c_str());

	workplaces_init(workplace_features);

	if(using_gen)
	for(const person& p : people) {
		gen_file << p << endl;
	}

	events_init();

	for(double tick = 0.0; tick < SIM_LENGTH; tick += TIME_STEP) {
		sim_tick(TIME_STEP);
	}

	// while(!events.empty()) {
	// 	event e = events.top();
	// 	events.pop();
	// 	cout << "time - " << e.time << "; type - " << e.t << endl;
	// 	e.call();
	// }

	out_file.close();
	return 0;
}
