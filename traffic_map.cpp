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

static ostream& operator<<(ostream& out, const segment& seg)
{
	out << seg.start << " -> " << seg.end << "; len = " << seg.length;
	return out;
}

void output_segment(ostream& out, const segment& seg, char type) {
	out << seg.start.x << " " << seg.start.y << " " << seg.end.x << " " << seg.end.y << " " << type << endl;
	return;
}



map<point, vector<segment>> graph; // adjacency list representation of the road network graph
map<point, bool> visited;
map<point, double> dist;
//map<point, int> connected_to; // how many points are connected with one way inward connection to each point
map<point, int> component;
map<point, vector<point>> weak_connections; // connections from another point to the key point
vector<point> all_nodes;
unordered_map<segment, vector<segment>> long_segments; // key - long segment; value - short segments that compose it
unordered_map<segment, segment> short_segments; // key - short segment; value - long segment it is a part of
map<point, vector<segment>> long_graph; // adjacency list for the long segments
ofstream out_file("out.txt", ofstream::trunc | ofstream::out);
//ifstream in_file("in.txt");


int unique_neighbors(point p) {
	// all segments represent a unique neighbor
	return graph[p].size() + weak_connections[p].size();
}

bool is_dead_end(point p) {
	return graph[p].size() == 0;
}

bool is_source(point p) { // called only once, so no need to be very fast
	if(weak_connections[p].size() > 0) return false;
	for(const segment& s : graph[p]) {
		for(const segment& s1 : graph[(point)s.end]) {
			if(s1.end == s.start) return false;
		}
	}
	return true;
}

bool dfs(point p, int comp) {
	if(visited[p]) return false;
	visited[p] = true;
	component[p] = comp;

	for(const segment& s : graph[p]) {
		//output_segment(out_file, s, '0' + comp);
		dfs((point)s.end, comp);
	}
	for(point p2 : weak_connections[p])
		dfs(p2, comp);
	return true;
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


void walk_forward(const segment& s, vector<segment>& out) {
	out.push_back(s);
	point curr = (point)s.end;
	unordered_set<point> cur_visited; // for O(1)
	cur_visited.insert((point)s.start); // this sets the direction to explore - depends on the direction of s
	cur_visited.insert(curr);
	//assert(graph[curr].size() > 0);
	while(graph.contains(curr) && (unique_neighbors(curr) == 2 || unique_neighbors(curr) == 1) ) {
		// if it has one or two neighbours we can be sure that there can't be a split path ahead - there is either 1 or 0 possible nodes to continue to
		int ind = -1; // ind is the index of the next edge on the adjacency list
		for(int i = 0; i < graph[curr].size(); i++) {
			if(cur_visited.contains((point)graph[curr][i].end)) continue;
			ind = i;
			break;
		}
		if(ind == -1) return;
		out.push_back(graph[curr][ind]);
		cur_visited.insert((point)graph[curr][ind].end);
		curr = (point)graph[curr][ind].end;
	}
}

bool find_path(point from, point to, vector<segment>& path) {
	// A* with compressed graph; 86 ms for vladislavovo - zelenika
	if(component[from] != component[to]) return false;
	if(unique_neighbors(from) == 0 || unique_neighbors(to) == 0 || is_dead_end(from) || is_source(to)) return from == to;
	visited.clear();
	for (point p : all_nodes) dist[p] = 1e20;
	dist[from] = 0.0;
	visited[from] = true;
	map<point, segment> came_from;
	// if 'to' has more than 2 neighbors or is a dead end, it is the target.
	point long_target = to;
	segment* target_segment = nullptr;
	if(unique_neighbors(to) <= 2 && !is_dead_end(to)) {
		// not a dead end and not a source, so there is exactly one outward edge
		int ind = 0;

		assert(short_segments.contains(graph[to][ind]));
		long_target = (point)short_segments[graph[to][ind]].start;
		target_segment = &short_segments[graph[to][ind]];
	}

	//assert(long_graph.contains(long_target)); // long target can be at the end - contains checks only for the beginning

	typedef tuple<double, double, point> pq_item;
	auto cmp = [](pq_item a, pq_item b) {
		return get<0>(a) + get<1>(a) > get<0>(b) + get<1>(b);
	};
	priority_queue < pq_item, vector<pq_item>, decltype(cmp)> pq(cmp);

	if(unique_neighbors(from) == 2) { // we know there is at least one real connection - if it's the only one, the point is a start of a long segment
		for(const segment& s : graph[from]) {
			vector<segment> p;
			walk_forward(s, p);
			double total_dist = 0;
			for(const segment& seg : p) {
				came_from[(point)seg.end] = seg;
				total_dist += seg.length;
			}
			point end_point = (point)p[p.size()-1].end;
			assert(long_graph.contains(end_point));
			visited[(point)end_point] = true;
			pq.emplace( total_dist, coord_dist(end_point, to), end_point );
		}
	} else {
		assert(long_graph.contains(from));
		pq.emplace( 0.0, coord_dist(from, long_target), from );
	}
	while(!pq.empty()) {
		// TODO: check if node is a part of long segments and iterate over them instead
		// TODO: if we have reached the long segment of the 'to' point, continue on the short segments to the end (graph)
		// for this loop everything is with long segments

		auto [cur_dist, _, cur_point] = pq.top();
		//assert((double)_ != nan);
		//cout << "Current point: " << coord_from_point(cur_point) << " dist: " << cur_dist << endl;
		pq.pop();
		if (cur_point == long_target) {
			if(target_segment != nullptr) {
				const vector<segment>& final = long_segments[*target_segment];
				for(int i = final.size()-1; i >= 0; i--) { // segment array is pre-reversed on purpose
					path.push_back(final[i]);
					if((point)final[i].end == to) break;
				}
			}
			reverse(path.begin(), path.end());


			while(came_from.contains(cur_point)) {
				segment cur_seg = came_from[cur_point];
				if(long_segments.contains(cur_seg)) {
					for(const segment& short_seg : long_segments[cur_seg]) {
						path.push_back(short_seg);
					}
				} else {
					path.push_back(cur_seg);
				}
				cur_point = (point)came_from[cur_point].start;
			}


			reverse(path.begin(), path.end());

			return true;
		}
		visited[cur_point] = true;

		for (const segment& seg : long_graph[cur_point]) {
			point next_point = (point)seg.end;
			double seg_length = seg.length;
			//cout << "    Segment: " << seg << endl;
			//output_segment(out_file, seg, 'c');
			double new_dist = cur_dist + seg_length;
			if (!visited[next_point] && new_dist < dist[next_point]) {
				dist[next_point] = new_dist;
				came_from[next_point] = seg;
				pq.push({ new_dist, coord_dist(next_point, long_target), next_point });
			}
		}
	}
	return false;
}
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


	//Features workplace_features = features_in_varna("a[disused:shop=mall]");
	//Features workplace_features = features_in_varna("a[shop=mall]");
	//Nodes workplace_nodes = features_in_varna.nodes("[shop],[amenity][amenity!=bicycle_parking,bicycle_repair_station,bicycle_rental,bicycle_wash,bus_station,compressed_air,charging_station,driver_training,grit_bin,motorcycle_parking,parking,parking_entrance,parking_space,taxi,weighbridge,atm,payment_terminal,baby_hatch,fountain,stage,studio,post_box,bbq,bench,check_in,dog_toilet,dressing_room,drinking_water,give_box,lounge,mailroom,parcel_locker,shelter,shower,telephone,toilets,water_point,watering_place,sanitary_dump_station,recycling,waste_basket,waste_disposal,baking_oven,clock,grave_yard,hunting_stand,kitchen,kneipp_water_cure,lounger,photo_booth,place_of_mourning,place_of_worship,public_bath,vending_machine,hydrant]");
	map<int64_t, vector<building>> residential_buildings; // map of residential area id to buildings in that area

	int count = 0;
	// building graph

	cout << "PARSING SEGMENTS...\n";
	for (const Way& r : roads) {
		double speed = (r.hasTag("maxspeed") ? stod(r["maxspeed"]) : DEFAULT_ROAD_SPEED);

		vector<Node> node_list; // nodes, composing the current Way
		r.nodes().addTo(node_list); // converting from Nodes to a vector is neccessary for random access

		for (int i = 0; i < node_list.size() - 1; i++) { // iterate through them in pairs
			point from = node_to_point(node_list[i]);
			point to = node_to_point(node_list[i + 1]);

			all_nodes.push_back(from);
			all_nodes.push_back(to);


			segment seg(from, to, speed);
			graph[from].push_back(seg); // add the segment to the adjacency list for the starting point
			output_segment(out_file, seg, 'f');
			count++;

			if (r["oneway"] == "yes" || r["oneway"] == "true" || r["oneway"] == "1") {
				weak_connections[to].push_back(from);
				continue; // for one way roads skip
			}


			segment newseg(to, from, speed);
			graph[to].push_back(newseg);
			output_segment(out_file, newseg, 'r');
			count++;

		}
		//cout << "Road: " << r["name"] << ", Type: " << r["highway"] << ", Length: " << r.length() << " meters" << endl;
	}
	cout << "DONE\n";
	cout << "Total segments: " << count << endl;

	cout << "MERGING SEGMENTS...\n";
	sort(all_nodes.begin(), all_nodes.end());
	all_nodes.erase(unique(all_nodes.begin(), all_nodes.end()), all_nodes.end()); // remove duplicates

	chrono::high_resolution_clock::time_point t1 = chrono::high_resolution_clock::now();
	for(point p : all_nodes) { // compressing graph - merging series of segments with only one edge (short_segment) into one (long_segment) (for faster pathfinding)
		// RUNS FOR ABOUT 1200ms with O(N) time complexity
		// we need to process only the points with more than 2 neighbors, and the ones with 1 neighbor, as they would be skipped completely otherwise
		if( (unique_neighbors(p) == 0) || (unique_neighbors(p) == 2) ) continue;
		for(const segment& s : graph[p]) {
			// if we have already processed the segment, there's no need to check it again
			if(short_segments.find(s) != short_segments.end()) continue;
			point from = (point)s.start;

			vector<segment> long_segment;
			//long_segment.push_back(s);

			// these two are for setting the long edge's length and max speed
			double total_dist = 0;
			double total_time = 0;

			// this loops through the edges until we find a crossing (point with at least 2 edges that are not our current point)
			walk_forward(s, long_segment);
			reverse(long_segment.begin(), long_segment.end());

			for(segment& ss : long_segment) {
				total_dist += ss.length;
				total_time += ss.max_speed;
			}
			double total_speed = total_dist / total_time;

			point to = (point)long_segment[0].end; //last point

			// here our long segment is (from, to), and its composing short segments are stored in long_segment
			segment l(from, to, total_speed, total_dist);
			output_segment(out_file, l, 'l');
			long_segments[l] = long_segment;
			long_graph[(point)l.start].push_back(l);
			for(const segment &short_segment : long_segment) {
				//assert(short_segment.real);
				//if(short_segments.find(s) == short_segments.end()) {
				short_segments[short_segment] = l;
				//cout << "adding hash " << short_segment.h() << endl;
				//output_segment(out_file, short_segment, 's');
				//}
			}
		}
	}
	chrono::high_resolution_clock::time_point t2 = chrono::high_resolution_clock::now();

	cout << "DONE\n";
	cout << duration_cast<chrono::milliseconds>(t2 - t1) << " milliseconds" << endl;

	rng::randomize();

	cout << "PARSING COMPONENTS\n";
	t1 = chrono::high_resolution_clock::now();

	int comp = 1;
	for(int i = 0; i < all_nodes.size(); i++) {
		if(dfs(all_nodes[i], comp)) comp++;
	}

	t2 = chrono::high_resolution_clock::now();
	cout << comp << " components\n";
	cout << "DONE\n";
	cout << duration_cast<chrono::milliseconds>(t2 - t1) << " milliseconds" << endl;

	cout << "STARTING PATHFINDING\n";
	int iterations = 1;
	if(argc > 1) iterations = atoi(argv[1]);
	for(int i = 0; i < iterations; i++) {

		point start = all_nodes[rng::random_int(0, all_nodes.size() - 1)];
		cout << "Start node: " << coord_from_point(start) << endl;
		point end = all_nodes[rng::random_int(0, all_nodes.size() - 1)];
		cout << "End node: " << coord_from_point(end) << endl;

		vector<segment> path;

		t1 = chrono::high_resolution_clock::now();
		cout << (find_path(start, end, path) ? "SUCCESS\n" : "FAILED\n");
		t2 = chrono::high_resolution_clock::now();
		cout << duration_cast<chrono::milliseconds>(t2 - t1) << " milliseconds" << endl;

		//cout << "Path segments: " << endl;
		for (segment seg : path) {
			//cout << seg.start << " -> " << seg.end << " (length: " << seg.length << " meters)" << endl;
			output_segment(out_file, seg, 'c');
		}
	}
	out_file.close();

	// residential buildings
	cout << "PARSING RESIDENTIAL BUILDINGS...\n";
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
	cout << "DONE\n";
	cout << "Total residents: " << count << endl;

	// workplaces
	cout << "PARSING WORKPLACES...\n";
	Features workplace_features = features_in_varna("a[disused:shop=mall],a[building=office,industrial,school,kindergarten],a[shop],n[shop],n[amenity][amenity!=bicycle_parking,bicycle_repair_station,bicycle_rental,bicycle_wash,bus_station,compressed_air,charging_station,driver_training,grit_bin,motorcycle_parking,parking,parking_entrance,parking_space,taxi,weighbridge,atm,payment_terminal,baby_hatch,fountain,stage,studio,post_box,bbq,bench,check_in,dog_toilet,dressing_room,drinking_water,give_box,lounge,mailroom,parcel_locker,shelter,shower,telephone,toilets,water_point,watering_place,sanitary_dump_station,recycling,waste_basket,waste_disposal,baking_oven,clock,grave_yard,hunting_stand,kitchen,kneipp_water_cure,lounger,photo_booth,place_of_mourning,place_of_worship,public_bath,vending_machine,hydrant]");
	vector<building> workplaces;

	count = 0;
	for (const Feature& f : workplace_features)
	{
		building b(f, building::WORKPLACE);
		workplaces.push_back(b);
		//cout << "Workplace: " << b.name << ", Capacity: " << b.capacity << ", Location: " << b.location->centroid() << endl;
		count += b.capacity;
	}
	cout << "DONE\n";
	cout << "Total positions: " << count << endl;
	return 0;
}
