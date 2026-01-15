// traffic_map.cpp : Defines the entry point for the application.
//

#include "traffic_map.h"

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

	cout << "Hello CMake." << endl;

	cout << "Running in ";
	system("pwd");
	cout << endl;

	ios_base::sync_with_stdio(false);
	cin.tie(nullptr);
	cout.tie(nullptr);
	cout << setprecision(9);

	string filepath = "/home/polimerto/Desktop/Coding/varna_traffic_map/bulgaria.gol";
	Features features(filepath.c_str());
	Feature varna = features("a[admin_level=5][name:en=Varna]").one(); // whole varna
	Features features_in_varna = features.intersecting(varna);
	// temp for testing in vuzrazhdane
	/*
	const Feature* vuzrazhdane;

	for (const Feature& f : features_in_varna) {
		if (f.id() == 44006186) { vuzrazhdane = &f; break; }
	}
	features_in_varna = features_in_varna.intersecting(*vuzrazhdane);
	*/
	Ways roads = features_in_varna.ways("[highway=motorway,trunk,primary,secondary,tertiary,residential,road,track,motorway_link,trunk_link,primary_link,secondary_link,tertiary_link,unclassified,service]");
	Relations bus_routes = features_in_varna.relations("r[route=bus][type=route]");
	Ways residential_areas = features_in_varna.ways("a[landuse=residential]");
	Ways parkings = features_in_varna.ways("a[amenity=parking]");



	// building graph
	graph_init(roads);

	rng::randomize();
	cout << "STARTING PATHFINDING\n";
	int iterations = 0;
	if(argc > 1) iterations = atoi(argv[1]);
	// int progress = 0;
	// for(point start : all_nodes) {
	// 	if(progress++ < 100000) continue;
	// 	cout << "progress: " << progress << " / " << all_nodes.size() << endl;
	// 	cout << "Start node: " << coord_from_point(start) << endl;
	// 	point end = all_nodes[rng::random_int(0, all_nodes.size() - 1)];
	// 	cout << "End node: " << coord_from_point(end) << endl;
	// 	vector<segment*> path;
	// 	cout << (find_path(start, end, path) ? "SUCCESS\n" : "FAILED\n");
	// 	progress++;
 //
	// }
	for(int i = 0; i < iterations; i++) {

		point start = all_nodes[rng::random_int(0, all_nodes.size() - 1)];
		cout << "Start node: " << coord_from_point(start) << endl;
		point end = all_nodes[rng::random_int(0, all_nodes.size() - 1)];
		cout << "End node: " << coord_from_point(end) << endl;

		vector<segment*> path;

		cout << (find_path(start, end, path) ? "SUCCESS\n" : "FAILED\n");

		//cout << "Path segments: " << endl;
		for (segment* seg : path) {
			//cout << seg.start << " -> " << seg.end << " (length: " << seg.length << " meters)" << endl;
			output_segment(out_file, *seg, 'c');
		}
	}
	out_file.close();

	// residential buildings
	(void)(VERBOSE && cout << "PARSING RESIDENTIAL BUILDINGS" << endl);
	auto t1 = chrono::high_resolution_clock::now();

	assert(is_init);
	int count = 0;
	for (const Way& a : residential_areas) {
		//cout << "Residential Area: " << a["name"] << endl;
		Ways res_areas = features_in_varna.ways("a[building=residential,house,apartments,detached,terrace]").intersecting(a);
		for (const Way& f : res_areas) {
			building b(f, building::RESIDENTIAL, ((f["building"] == "house") ? 0.002 : 0.0)); // lower density for houses
			residential_buildings[a.id()].push_back(b);
			//cout << "Residential Building: " << b.name << ", Capacity: " << b.capacity << ", Location: " << coord_from_point(b.location) << endl;
			count += b.capacity;
		}
	}

	auto t2 = chrono::high_resolution_clock::now();
	(void)(VERBOSE && cout << "Total residents: " << count << endl);
	(void)(VERBOSE && cout << "DONE" << endl);
	(void)(VERBOSE && cout << duration_cast<chrono::milliseconds>(t2 - t1) << " milliseconds" << endl);

	people_init();


	// workplaces
	Features workplace_features = features_in_varna("a[disused:shop=mall],a[building=office,industrial,school,kindergarten],a[shop],n[shop],n[amenity][amenity!=bicycle_parking,bicycle_repair_station,bicycle_rental,bicycle_wash,bus_station,compressed_air,charging_station,driver_training,grit_bin,motorcycle_parking,parking,parking_entrance,parking_space,taxi,weighbridge,atm,payment_terminal,baby_hatch,fountain,stage,studio,post_box,bbq,bench,check_in,dog_toilet,dressing_room,drinking_water,give_box,lounge,mailroom,parcel_locker,shelter,shower,telephone,toilets,water_point,watering_place,sanitary_dump_station,recycling,waste_basket,waste_disposal,baking_oven,clock,grave_yard,hunting_stand,kitchen,kneipp_water_cure,lounger,photo_booth,place_of_mourning,place_of_worship,public_bath,vending_machine,hydrant]");

	workplaces_init(workplace_features);

	//for(person& p : people) {
		//cout << "age " << p.age << " - home: " <<  (coord_from_point(p.home->location));
		//if(p.work) cout << "; work: " << (coord_from_point(p.work->location));
		//cout << "; can drive: " << p.can_drive << endl;
	//}

	return 0;
}
