// traffic_map.cpp : Defines the entry point for the application.
//

#include "traffic_map.h"

using namespace std;
using namespace geodesk;
using namespace traffic_sim;
using namespace config;

static ostream& operator<<(ostream& out, const Coordinate& coord)
{
	out << "(" << Mercator::latFromY(coord.y) << ", " << Mercator::lonFromX(coord.x) << ")";
	return out;
}

static ostream& operator<<(ostream& out, segment& seg)
{
	out << seg.start << " -> " << seg.end << "; len = " << seg.length();
	return out;
}

double coord_dist(Coordinate a, Coordinate b) {
	return sqrt(pow(b.x - a.x, 2) + pow(b.y - a.y, 2));
}

map<point, vector<segment>> graph; // adjacency list representation of the road network graph
map<point, bool> visited; // for unlinked components
map<point, double> dist;
vector<point> all_nodes; // might contain duplicates
map<seg_hash, vector<segment>> long_segments;
map<seg_hash, segment> short_segments;
void find_path(point from, point to, vector<segment>& path) {
	// A*
	visited.clear();
	dist.clear();
	for (point p : all_nodes) dist[p] = 1e20;
	dist[from] = 0.0;
	visited[from] = true;
	map<point, segment> came_from;

	typedef tuple<double, point> pq_item;
	auto cmp = [](pq_item a, pq_item b) {
		return get<0>(a) > get<0>(b);
		};
	priority_queue < pq_item, vector<pq_item>, decltype(cmp)> pq(cmp);
	pq.emplace( 0.0, from );

	while(!pq.empty()) {
		// TODO: check if node is a part of long segments and iterate over them instead
		// TODO: if we have reached the long segment of the 'to' point, continue on the short segments to the end (graph)

		auto [cur_dist, cur_point] = pq.top();
		//cout << "Current point: " << coord_from_point(cur_point) << " dist: " << cur_dist << endl;
		pq.pop();
		if (cur_point == to) {
			while(came_from.contains(cur_point)) {
				path.push_back(came_from[cur_point]);
				cur_point = (point)came_from[cur_point].start;
			}

			reverse(path.begin(), path.end());

			return;
		}
		visited[cur_point] = true;

		for (segment seg : graph[cur_point]) {
			point next_point = (point)seg.end;
			double seg_length = seg.length();
			//cout << "    Segment: " << seg << endl;
			double new_dist = cur_dist + seg_length;
			if (!visited[next_point] && new_dist < dist[next_point]) {
				dist[next_point] = new_dist;
				came_from[next_point] = seg;
				pq.push({ new_dist + coord_dist(coord_from_point(next_point), coord_from_point(to)), next_point });
			}
		}
	}
}


point closest_node(Coordinate coord) {
	point coord_hash = (point)coord;
	point closest = -1;
	double closest_dist = 1e20;
	for (point n_hash : all_nodes) {
		Coordinate n_coord = coord_from_point(n_hash);
		double dist = coord_dist(coord, n_coord);
		if (dist < closest_dist) {
			closest_dist = dist;
			closest = n_hash;
		}
	}
	return closest;
}

/*
Node* find(Features fs, int64_t id) {
	for (Node f : fs.nodes()) {
		if (f.id() == id) return &f;
	}
	return nullptr;
}
*/

int main()
{


	ios_base::sync_with_stdio(false);
	cin.tie(nullptr);
	cout.tie(nullptr);
	cout << setprecision(9);
	cout << "Hello CMake." << endl;

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
	ofstream out_file("out.txt", ofstream::trunc | ofstream::out);

	//Features workplace_features = features_in_varna("a[disused:shop=mall]");
	//Features workplace_features = features_in_varna("a[shop=mall]");
	//Nodes workplace_nodes = features_in_varna.nodes("[shop],[amenity][amenity!=bicycle_parking,bicycle_repair_station,bicycle_rental,bicycle_wash,bus_station,compressed_air,charging_station,driver_training,grit_bin,motorcycle_parking,parking,parking_entrance,parking_space,taxi,weighbridge,atm,payment_terminal,baby_hatch,fountain,stage,studio,post_box,bbq,bench,check_in,dog_toilet,dressing_room,drinking_water,give_box,lounge,mailroom,parcel_locker,shelter,shower,telephone,toilets,water_point,watering_place,sanitary_dump_station,recycling,waste_basket,waste_disposal,baking_oven,clock,grave_yard,hunting_stand,kitchen,kneipp_water_cure,lounger,photo_booth,place_of_mourning,place_of_worship,public_bath,vending_machine,hydrant]");
	map<int64_t, vector<building>> residential_buildings; // map of residential area id to buildings in that area

	int count = 0;
	// building graph

	for (const Way& r : roads) {
		double speed = (r.hasTag("maxspeed") ? stod(r["maxspeed"]) : DEFAULT_ROAD_SPEED);

		vector<Node> node_list;
		r.nodes().addTo(node_list);

		for (int i = 0; i < node_list.size() - 1; i++) {
			const Node& from = node_list[i];
			const Node& to = node_list[i + 1];

			all_nodes.push_back(node_to_point(from));
			all_nodes.push_back(node_to_point(to));

			segment seg(from, to, speed);

			graph[node_to_point(from)].push_back(seg);
			//auto tmp2 = from;
			//auto tmp1 = *(graph[node_to_point(from)][0].start);
			//assert(tmp1 == tmp2);
			out_file << seg.start.x << " " << seg.start.y << " " << seg.end.x << " " << seg.end.y << " f" << endl;
			count++;

			if (r["oneway"] == "yes" || r["oneway"] == "true" || r["oneway"] == "1") {
				continue;
			}

			segment newseg(to, from, speed);
			graph[node_to_point(to)].push_back(newseg);
			out_file << newseg.start.x << " " << newseg.start.y << " " << newseg.end.x << " " << newseg.end.y << " r" << endl;
			count++;
		}
		//cout << "Road: " << r["name"] << ", Type: " << r["highway"] << ", Length: " << r.length() << " meters" << endl;
	}
	cout << "Total segments: " << count << endl;


	sort(all_nodes.begin(), all_nodes.end());
	all_nodes.erase(unique(all_nodes.begin(), all_nodes.end()), all_nodes.end()); // remove duplicates

	point start = all_nodes[rng::random_int(0, all_nodes.size() - 1)];
	cout << "Start node: " << coord_from_point(start) << endl;
	//rng::mt_gen.discard(100); // advance the state to get a different random number
	point end = all_nodes[rng::random_int(0, all_nodes.size() - 1)];
	cout << "End node: " << coord_from_point(end) << endl;

	vector<segment> path;
	chrono::high_resolution_clock::time_point t1 = chrono::high_resolution_clock::now();
	find_path(start, end, path);
	chrono::high_resolution_clock::time_point t2 = chrono::high_resolution_clock::now();
	cout << duration_cast<chrono::milliseconds>(t2 - t1) << " milliseconds" << endl;
	cout << "Path segments: " << endl;
	for (segment seg : path) {
		cout << seg.start << " -> " << seg.end << " (length: " << seg.length() << " meters)" << endl;
		out_file << seg.start.x << " " << seg.start.y << " " << seg.end.x << " " << seg.end.y << " c" << endl;
	}
	out_file.close();

	// residential buildings
	count = 0;
	for (const Way& a : residential_areas) {
		//cout << "Residential Area: " << a["name"] << endl;
		Ways res_areas = features_in_varna.ways("a[building=residential,house,apartments,detached,terrace]").intersecting(a);
		for (const Way& f : res_areas) {
			building b(f, building::RESIDENTIAL, ((f["building"] == "house") ? 0.002 : 0.0)); // lower density for houses
			residential_buildings[a.id()].push_back(b);
			//cout << "Residential Building: " << b.name << ", Capacity: " << b.capacity << ", Location: " << b.location->centroid() << endl;
			count += b.capacity;
		}
	}
	cout << "Total residents: " << count << endl;

	// workplaces
	Features workplace_features = features_in_varna("a[disused:shop=mall],a[building=office,industrial,school,kindergarten],a[shop],n[shop],n[amenity][amenity!=bicycle_parking,bicycle_repair_station,bicycle_rental,bicycle_wash,bus_station,compressed_air,charging_station,driver_training,grit_bin,motorcycle_parking,parking,parking_entrance,parking_space,taxi,weighbridge,atm,payment_terminal,baby_hatch,fountain,stage,studio,post_box,bbq,bench,check_in,dog_toilet,dressing_room,drinking_water,give_box,lounge,mailroom,parcel_locker,shelter,shower,telephone,toilets,water_point,watering_place,sanitary_dump_station,recycling,waste_basket,waste_disposal,baking_oven,clock,grave_yard,hunting_stand,kitchen,kneipp_water_cure,lounger,photo_booth,place_of_mourning,place_of_worship,public_bath,vending_machine,hydrant]");
	vector<building> workplaces;

	count = 0;
	for (const Feature& f : workplace_features)
	{
		building b(f, building::WORKPLACE);
		workplaces.push_back(b);
		cout << "Workplace: " << b.name << ", Capacity: " << b.capacity << ", Location: " << b.location->centroid() << endl;
		count += b.capacity;
	}
	cout << "Total positions: " << count << endl;
	return 0;
}
