// traffic_map.h : Include file for standard system include files,
// or project specific include files.

#pragma once

#include <iostream>
#include <geodesk/geodesk.h>
#include <vector>
#include <map>
#include <unordered_map>
#include <tuple>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <queue>
#include <random>
#include <functional>

namespace config {
	bool VERBOSE = true;

	std::ofstream out_file("out.txt", std::ofstream::trunc | std::ofstream::out);


	const double TIME_STEP = 0.1; // time step in seconds
	const double SIM_LENGTH = 86400.0; // how long to run the simulation
	const double DEFAULT_BUILDING_HEIGHT = 5.0; // default building height in meters
	// target workplaces is 150000
	const double EMPLOYEE_DENSITY = 0.038; // people per cubic meter (adding 0.01 increases result by 56309)
	const int MIN_EMPLOYEES = 5; // minimum people per workplace (adds or subtracts 2861)

	// target residents is 350000
	const double RESIDENTIAL_DENSITY = 0.012;
	const int MIN_RESIDENTS = 3;

	const double ENTERTAINMENT_DENSITY = 0.024; 
	const int MIN_VISITORS = 5; 
	
	const double DEFAULT_ROAD_SPEED = 13.88; // default road speed in m/s
	const double DEFAULT_CAR_RADIUS = 6.0; // minimum allowed distance between cars in m
	const double DEFAULT_CAR_ACCELERATION = 3.5; // m/s^2
	const double DEFAULT_CAR_BRAKING = 7;

	const double DEFAULT_TRAFFIC_LIGHT_ON_TIME = 15.0; // on means green, off means red
	const double DEFAULT_TRAFFIC_LIGHT_OFF_TIME = 45.0;
	const int MAX_CARS = 1000000; // total number of cars in the simulation
	const double EPSILON = 0.000000001;

	const double RANDOM_EVENT_CHANCE = 0.2; // chance for an unemployed person to go outside
}

namespace traffic_sim
{
	using namespace std;
	using namespace geodesk;
	using namespace config;

	typedef int64_t point; // hash for node coordinates

	double cur_time = 0; //seconds (0 - 86400)

	double time_hms(int hours, int minutes, double seconds) {
		return hours * 3600 + minutes * 60 + seconds;
	}

	uint64_t first_half(int32_t x) {
		return (uint64_t)x & 0b1111111111111111;
	}

	point node_to_point(Node n) {
		return (point)n.xy();
	}

	Coordinate coord_from_point(point p) {
		int32_t y = (int32_t)(p >> 32);
		int32_t x = (int32_t)(p & 0xFFFFFFFF);
		return Coordinate(x, y);
	}

	double deg_to_rad(double deg) {
		return deg / 180.0 * numbers::pi;
	}

	double hav(double x) {
		return pow(sin(x/2), 2);
	}
	double ahav(double x) {
		return 2 * asin(sqrt(x));
	}

	constexpr double imp_to_m(int imps) {
		return (unsigned int)imps * 0.006801778856644; // conversion factor centered at 43.2 degrees of latitude
	}

	constexpr double kmh_to_ms(double kmh) {
		return kmh / 3.6;
	}

	double coord_dist(Coordinate a, Coordinate b) {
		// euclid distance - inaccurate
		double par1 = (a.x-b.x);
		double par2 = (a.y-b.y);
		// assert(par1 * par1 + par2 * par2 >= 0);
		return imp_to_m(sqrt(par1 * par1 + par2 * par2));


	}
	double coord_dist_accurate(Coordinate a, Coordinate b) {
		// great circle distance - extremely slow
		constexpr double r = 6371000.0;
		double lon1 = deg_to_rad(a.lon());
		double lat1 = deg_to_rad(a.lat());
		double lon2 = deg_to_rad(b.lon());
		double lat2 = deg_to_rad(b.lat());

		double angle = ahav(hav(abs(lat1-lat2)) + (1 - hav(abs(lat1-lat2)) - hav(lat1+lat2)) * hav(abs(lon1-lon2)));
		return r * angle;

	}
	double coord_dist_accurate(point a, point b) {
		return coord_dist_accurate(coord_from_point(a), coord_from_point(b));
	}
	double coord_dist(point a, point b) {
		if(a == b) return 0;
		return coord_dist(coord_from_point(a), coord_from_point(b));
	}

	namespace rng {
		random_device rd;
		static mt19937 mt_gen;

		void randomize() {
			uniform_int_distribution<int> dist;
			mt_gen.seed(dist(rd));
		}

		int random_int(int min_val, int max_val) {
			uniform_int_distribution<int> dist(min_val, max_val);
			return dist(mt_gen);
		}
		double random_double(double min_val, double max_val) {
			uniform_real_distribution<double> dist(min_val, max_val);
			return dist(mt_gen);
		}
		double normal_int(double mean, double stddev) {
			normal_distribution<double> dist(mean, stddev);
			return (int)round(dist(mt_gen));
		}
		double normal_double(double mean, double stddev) {
			normal_distribution<double> dist(mean, stddev);
			return dist(mt_gen);
		}
		bool random_chance(double probability) {
			uniform_real_distribution<double> dist(0.0, 1.0);
			return dist(mt_gen) < probability;
		}
		double power_law_double(double min_val, double max_val, double exponent) {
			double r = random_double(0.0, 1.0);
			double scaled = pow(r, exponent);
			return min_val + (max_val - min_val) * scaled;
		}
	};

	struct segment; // forward declarations
	struct person;
	struct building;
	struct car;


	struct segment { // directed road segment
		Coordinate start;
		Coordinate end;
		double length;

		vector<car*> cars;
		double max_speed; // maximum speed allowed on the segment

		//traffic light, at segment's end
		double on_time = 1;
		double off_time = 0;
		double phase_offset = 0;

		bool is_red() const {
			if(off_time == 0) return false;
			//cout << fmod(cur_time + phase_offset, on_time + off_time) << endl;
			return (fmod(cur_time + phase_offset, on_time + off_time) > on_time);
		}

		bool operator==(const segment& s) const
		{
			return s.start == start && s.end == end;// && s.length == length;
		}

		double angle(const segment& other) {
			return acos
			(
				(
					(end.x - start.x) * (other.end.x - other.start.x) + (end.y - start.y) * (other.end.y - other.start.y)
				) /
				(
					hypot(end.x - start.x, end.y - start.y) * hypot(other.end.x - other.start.x, other.end.y - other.start.y)
				)

			);
		}

		// segment(const Node& s, const Node& e, const double ms, const double len = 0, const double on_time = DEFAULT_TRAFFIC_LIGHT_ON_TIME, const double off_time = 0, const double phase_offset = 0.0) {
		// 	start = s.xy();
		// 	end = e.xy();
		// 	max_speed = ms;
		// 	length = len;
		// 	if(len == 0) length = coord_dist(start, end);
		// 	this->on_time = on_time; // on = green
		// 	this->off_time = off_time;
		// 	if(phase_offset < EPSILON) this->phase_offset = rng::random_double(0, on_time + off_time);
		// 	else this->phase_offset = phase_offset;
  //
		// 	cout << "on time: " << this->on_time << "; off time: " << this->off_time << "; phase: " << this->phase_offset << endl;
		// 	cars = vector<car*>(0);
		// }
		segment(point _p1, point _p2, const double ms, const double len = 0, double off_time = 0.0) {
			start = coord_from_point(_p1);
			end = coord_from_point(_p2);
			max_speed = ms;
			length = len;
			if(len < EPSILON) length = coord_dist(start, end);

			on_time = DEFAULT_TRAFFIC_LIGHT_ON_TIME;
			this->off_time = off_time;
			phase_offset = rng::random_double(0, on_time + off_time);

			cars = vector<car*>(0);
		}
		segment(const segment &s)
		{
			start = s.start;
			end = s.end;
			max_speed = s.max_speed;
			length = s.length;
			on_time = s.on_time;
			off_time = s.off_time;
			phase_offset = s.phase_offset;

			cars = vector<car*>(0);
		}
		segment() = default;

		uint64_t h() const { // hash function (segment direction matters) - preserves as much information as possible while still being a 64 bit int
			return  *(uint64_t *)(&length) ^ ((first_half(start.x) << 48) |
			(first_half(start.y) << 32) |
			(first_half(end.x) << 16) |
			first_half(end.y));
		}
		string h2() const { // hash function (segment direction matters)
			stringstream ss;
			ss << hex << (point)start.x << (point)end.x << length;
			return ss.str();
		}
		bool weak_equals(point p1, point p2) const {
			return ( ((point)start == p1 && (point)end == p2) || ((point)start == p2 && (point)end == p1) );
		}

	};


	static ostream& operator<<(ostream& out, const Coordinate& coord)
	{
		out << coord.x << " " << coord.y;
		//out << Mercator::latFromY(coord.y) << " " << Mercator::lonFromX(coord.x);
		return out;
	}

	// static ostream& operator<<(ostream& out, const segment& seg)
	// {
	// 	out << seg.start << " -> " << seg.end << "; len = " << seg.length;
	// 	return out;
	// }

	void output_segment(ostream& out, const segment& seg, char type) {
		out << "seg " << seg.start.x << " " << seg.start.y << " " << seg.end.x << " " << seg.end.y << " " << type << endl;
		return;
	}

	void output_traffic_light(ostream& out, const segment& seg) {
		out << "trl " << seg.start.x << " " << seg.start.y << " " << seg.end.x << " " << seg.end.y << " " << (seg.is_red() ? 'r' : 'g') << endl;
		return;
	}



	bool is_init = false;
	vector<point> all_nodes;
	vector<point> traffic_light_nodes;

	point closest_point(Coordinate x) {
		point x_p = (point)x;
		point cur_p;
		double min_dist = 1e20;
		for(point& p : all_nodes) {
			double cur_dist = coord_dist(x_p, p);
			if(cur_dist == 0) return x_p;

			if(cur_dist < min_dist) {
				min_dist = cur_dist;
				cur_p = p;
			}
		}
		return cur_p;
	}


	struct building {
		enum building_type {
			RESIDENTIAL,
			WORKPLACE,
			SCHOOL,
			ENTERTAINMENT
		};
		string name;
		building_type type;
		Coordinate location;
		mutable point road_connection = 0;
		int capacity; // number of residents/people/employees
		int cur_employees = 0;

		building(const Feature& loc, building_type t, double people_density = 0.0) {
			assert(is_init);
			name = loc["name"].storedString();
			type = t;

			if(name == "") {
				if (t == RESIDENTIAL) name = "Unnamed Residential Building";
				else if(t == ENTERTAINMENT) name = "Unnamed Entertainment Building";
				else name = "Unnamed Workplace";
			}

			location = loc.centroid();
			//road_connection = closest_point(loc.centroid()); // too slow to do on initialization

			double height = DEFAULT_BUILDING_HEIGHT;

			if(loc.hasTag("height")) {
				height = stod(loc["height"]);
			}

			int min_cap;
			switch (type) {
				case RESIDENTIAL:
					if (people_density == 0.0)
						people_density = RESIDENTIAL_DENSITY; // residents per cubic meter
					min_cap = MIN_RESIDENTS;
					break;
				case ENTERTAINMENT:
					if (people_density == 0.0)
						people_density = ENTERTAINMENT_DENSITY; // people per cubic meter
					min_cap = MIN_VISITORS;
					break;
				case WORKPLACE:
				case SCHOOL:
					if (people_density == 0.0)
						people_density = EMPLOYEE_DENSITY; // employees per cubic meter
					min_cap = MIN_EMPLOYEES;
					break;
			}

			capacity = max((int)(height * loc.area() * people_density), min_cap);
		}

		explicit operator point() const noexcept {
			if(!road_connection) road_connection = closest_point(location);
			return road_connection;
		}

	};


	map<int64_t, vector<building>> residential_buildings; // map of residential area id to buildings in that area; initialized in main
}
template<>
struct std::hash<traffic_sim::segment> {
	size_t operator()(const traffic_sim::segment& s) const {
		return s.h();
		//return hash<string>{}(s.h());
	}
};


namespace pathfinding {
	using namespace std;
	using namespace traffic_sim;
	using namespace geodesk;
	using namespace config;

	map<point, vector<segment>> graph; // adjacency list representation of the road network graph
	map<point, bool> visited;
	map<point, double> dist;
	//map<point, int> connected_to; // how many points are connected with one way inward connection to each point
	map<point, int> component;
	map<point, vector<point>> weak_connections; // connections from another point to the key point
	unordered_map<segment, vector<segment*>> long_segments; // key - long segment; value - short segments that compose it
	unordered_map<segment, segment> short_segments; // key - short segment; value - long segment it is a part of
	map<point, vector<segment>> long_graph; // adjacency list for the long segments
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


	void walk_forward(segment& s, vector<segment*>& out) {
		out.push_back(&s);
		point last = (point)s.start;
		point curr = (point)s.end;
		unordered_set<point> cur_visited; // for O(1)
		cur_visited.insert((point)s.start); // this sets the direction to explore - depends on the direction of s
		cur_visited.insert(curr);
		//assert(graph[curr].size() > 0);
		while(graph.contains(curr) && (unique_neighbors(curr) == 2 || unique_neighbors(curr) == 1) ) {
			// if it has one or two neighbours we can be sure that there can't be a split path ahead - there is either 1 or 0 possible nodes to continue to
			int ind = -1; // ind is the index of the next edge on the adjacency list
			for(int i = 0; i < graph[curr].size(); i++) {
				if(cur_visited.contains((point)graph[curr][i].end) && graph[curr][i].weak_equals(last, curr)) continue;
				ind = i;
				break;
			}
			if(ind == -1) return;
			out.push_back(&graph[curr][ind]);
			cur_visited.insert((point)graph[curr][ind].end);
			last = curr;
			curr = (point)graph[curr][ind].end;
		}
	}

	bool find_path(point from, point to, vector<segment*>& path) {
		// A* with compressed graph; 86 ms for vladislavovo - zelenika

		auto t1 = chrono::high_resolution_clock::now();
		(void)(VERBOSE && cout << "STARTED A*" << endl);

		if(!is_init || component[from] != component[to]) return false;

		vector<segment*> first_steps;

		while(graph[from].size() == 1) {
			first_steps.push_back(&graph[from][0]);
			from = (point)graph[from][0].end;
			if(from == to) {
				for(segment* s : first_steps) {
					path.push_back(s);
				}
			}
		}
		reverse(first_steps.begin(), first_steps.end());
		if(unique_neighbors(from) == 0 || unique_neighbors(to) == 0 || is_dead_end(from) || is_source(to)) return from == to;
		unordered_map<point, bool> astar_visited;
		astar_visited.clear();
		for (point p : all_nodes) dist[p] = 1e20;
		dist[from] = 0.0;
		astar_visited[from] = true;
		map<point, segment*> came_from;
		// if 'to' has more than 2 neighbors or is a dead end, it is the target.
		point long_target = to;
		segment* target_segment = nullptr;
		if(unique_neighbors(to) <= 2 && !is_dead_end(to)) {
			// not a dead end and not a source, so there is exactly one outward edge
			int ind = 0;

			//assert(short_segments.contains(graph[to][ind]));
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
			for(segment& s : graph[from]) {
				vector<segment*> p;
				walk_forward(s, p);
				double total_dist = 0;
				for(segment* seg : p) {
					came_from[(point)seg->end] = seg;
					total_dist += seg->length;
				}
				point end_point = (point)p[p.size()-1]->end;
				assert(long_graph.contains(end_point));
				astar_visited[(point)end_point] = true;
				pq.emplace( total_dist, coord_dist(end_point, to), end_point );
			}
		} else {
			assert(long_graph.contains(from));
			pq.emplace( 0.0, coord_dist(from, long_target), from );
		}
		while(!pq.empty()) {
			//TODO: include segment speed in a*
			auto [cur_dist, _, cur_point] = pq.top();
			//assert((double)_ != nan);
			//cout << "Current point: " << coord_from_point(cur_point) << " dist: " << cur_dist << endl;
			pq.pop();
			if (cur_point == long_target) {
				if(target_segment != nullptr) {
					vector<segment*>& final = long_segments[*target_segment];
					for(int i = final.size()-1; i >= 0; i--) { // segment array is pre-reversed on purpose
						path.push_back(final[i]);
						if((point)final[i]->end == to) break;
					}
				}
				reverse(path.begin(), path.end());


				while(came_from.contains(cur_point)) {
					segment& cur_seg = *came_from[cur_point];
					if(long_segments.contains(cur_seg)) {
						for(segment* short_seg : long_segments[cur_seg]) {
							path.push_back(short_seg);
						}
					} else {
						path.push_back(&cur_seg);
					}
					cur_point = (point)came_from[cur_point]->start;
				}


				for(segment* sp : first_steps) path.push_back(sp);
				reverse(path.begin(), path.end());

				auto t2 = chrono::high_resolution_clock::now();
				(void)(VERBOSE && cout << duration_cast<chrono::milliseconds>(t2 - t1) << " milliseconds" << endl);
				return true;
			}
			astar_visited[cur_point] = true;

			for (segment& seg : long_graph[cur_point]) {
				point next_point = (point)seg.end;
				double seg_length = seg.length / seg.max_speed;
				//cout << "    Segment: " << seg << endl;
				//output_segment(out_file, seg, 'c');
				double new_dist = cur_dist + seg_length;
				if (!astar_visited[next_point] && new_dist < dist[next_point]) {
					dist[next_point] = new_dist;
					came_from[next_point] = &seg;
					pq.push({ new_dist, coord_dist(next_point, long_target), next_point });
				}
			}
		}
		auto t2 = chrono::high_resolution_clock::now();
		(void)(VERBOSE && cout << duration_cast<chrono::milliseconds>(t2 - t1) << " milliseconds" << endl);
		return false;
	}

	bool find_path(const building& from, const building& to, vector<segment*>& path) {
		return find_path((point)from, (point)to, path);
	}

	void graph_init(Ways& roads) {
		int seg_count = 0;
		(void)(VERBOSE && cout << "BUILDING GRAPH" << endl);

		chrono::high_resolution_clock::time_point t1 = chrono::high_resolution_clock::now();


		for (const Way& r : roads) {
			double speed = (r.hasTag("maxspeed") ? kmh_to_ms(stod(r["maxspeed"])) : DEFAULT_ROAD_SPEED);

			vector<Node> node_list; // nodes, composing the current Way
			r.nodes().addTo(node_list); // converting from Nodes to a vector is neccessary for random access

			for (int i = 0; i < node_list.size() - 1; i++) { // iterate through them in pairs
				point from = node_to_point(node_list[i]);
				point to = node_to_point(node_list[i + 1]);

				all_nodes.push_back(from);
				all_nodes.push_back(to);


				double off_time = 0.0;
				if((node_list[i+1].hasTag("highway") && node_list[i+1]["highway"] == "traffic_signals")){// || (node_list[i+1].hasTag("crossing") && node_list[i+1]["crossing"] == "traffic_signals") || (node_list[i+1].hasTag("traffic_signals"))) {
					off_time = DEFAULT_TRAFFIC_LIGHT_OFF_TIME;
					traffic_light_nodes.push_back(from);
				}

				segment seg(from, to, speed, 0.0, off_time);

				graph[from].push_back(seg); // add the segment to the adjacency list for the starting point
				output_segment(out_file, seg, seg.off_time ? 'k' : 'f');
				seg_count++;

				if (r["oneway"] == "yes" || r["oneway"] == "true" || r["oneway"] == "1" || (r.hasTag("junction") && r["junction"] == "roundabout")) {
					weak_connections[to].push_back(from);
					continue; // for one way roads skip
				}

				off_time = 0.0;

				if((node_list[i].hasTag("highway") && node_list[i]["highway"] == "traffic_signals")){// || (node_list[i+1].hasTag("crossing") && node_list[i+1]["crossing"] == "traffic_signals") || (node_list[i+1].hasTag("traffic_signals"))) {
					off_time = DEFAULT_TRAFFIC_LIGHT_OFF_TIME;
					traffic_light_nodes.push_back(to);
				}
				segment newseg(to, from, speed, 0.0, off_time);
				graph[to].push_back(newseg);
				output_segment(out_file, newseg, newseg.off_time ? 'k' : 'r');
				seg_count++;

			}
			//cout << "Road: " << r["name"] << ", Type: " << r["highway"] << ", Length: " << r.length() << " meters" << endl;
		}
		chrono::high_resolution_clock::time_point t2 = chrono::high_resolution_clock::now();

		(void)(VERBOSE && cout << "Segments: " << seg_count << endl);
		(void)(VERBOSE && cout << "DONE" << endl);

		(void)(VERBOSE && cout << duration_cast<chrono::milliseconds>(t2 - t1) << " milliseconds" << endl);

		(void)(VERBOSE && cout << "REMOVING DUPLICATES" << endl);
		if(VERBOSE) t1 = chrono::high_resolution_clock::now();

		sort(all_nodes.begin(), all_nodes.end());
		all_nodes.erase(unique(all_nodes.begin(), all_nodes.end()), all_nodes.end()); // remove duplicates

		if(VERBOSE) t2 = chrono::high_resolution_clock::now();
		(void)(VERBOSE && cout << "DONE" << endl);
		(void)(VERBOSE && cout << duration_cast<chrono::milliseconds>(t2 - t1) << " milliseconds" << endl);

		(void)(VERBOSE && cout << "MERGING GRAPH" << endl);
		if(VERBOSE) t1 = chrono::high_resolution_clock::now();

		for(point p : all_nodes) { // compressing graph - merging series of segments with only one edge (short_segment) into one (long_segment) (for faster pathfinding)
			// RUNS FOR ABOUT 1200ms with O(N) time complexity
			// we need to process only the points with more than 2 neighbors, and the ones with 1 neighbor, as they would be skipped completely otherwise
			if( (unique_neighbors(p) == 0) || (unique_neighbors(p) == 2) ) continue;
			for(segment& s : graph[p]) {
				// if we have already processed the segment, there's no need to check it again
				//if(short_segments.contains(s)) continue;
				point from = (point)s.start;

				vector<segment*> long_segment;
				//long_segment.push_back(s);

				// these two are for setting the long edge's length and max speed
				double total_dist = 0;
				double total_time = 0;

				// this loops through the edges until we find a crossing (point with at least 2 edges that are not our current point)
				walk_forward(s, long_segment);
				reverse(long_segment.begin(), long_segment.end());

				for(segment* ss : long_segment) {
					total_dist += ss->length;
					total_time += ss->length / ss->max_speed;
				}
				double total_speed = total_dist / total_time;

				point to = (point)long_segment[0]->end; //last point

				// here our long segment is (from, to), and its composing short segments are stored in long_segment
				segment l(from, to, total_speed, total_dist);
				output_segment(out_file, l, 'l');
				long_segments[l] = long_segment;
				long_graph[(point)l.start].push_back(l);
				for(segment* short_segment : long_segment) {
					//assert(short_segment.real);
					//if(short_segments.find(s) == short_segments.end()) {
					short_segments[*short_segment] = l;
					//cout << "adding hash " << short_segment.h() << endl;
					//output_segment(out_file, *short_segment, (short_segment->start.x > short_segment->end.x) ? 's' : 'k');
					//}
				}
			}
		}

		//assert(short_segments.size() == seg_count); // 24 loose segments...

		if(VERBOSE) t2 = chrono::high_resolution_clock::now();
		(void)(VERBOSE && cout << "DONE" << endl);
		(void)(VERBOSE && cout << duration_cast<chrono::milliseconds>(t2 - t1) << " milliseconds" << endl);

		(void)(VERBOSE && cout << "MARKING COMPONENTS" << endl);
		if(VERBOSE) t1 = chrono::high_resolution_clock::now();

		int comp = 1;
		for(int i = 0; i < all_nodes.size(); i++) {
			if(dfs(all_nodes[i], comp)) comp++;
		}

		if(VERBOSE) t2 = chrono::high_resolution_clock::now();
		(void)(VERBOSE && cout << "DONE" << endl);
		(void)(VERBOSE && cout << duration_cast<chrono::milliseconds>(t2 - t1) << " milliseconds" << endl);

		is_init = true;
	}
}

namespace traffic_sim {
	using namespace pathfinding;

	double closest_obst(car* c, double vision);

	struct car {
		Coordinate position;
		point target_loc;
		double cur_speed;
		double max_speed;
		double accel;
		double brake;
		double obst = 0.0;
		segment* cur_segment;
		segment* next_segment = nullptr;
		double segment_position = 0.0;
		vector<segment*> path;

		bool should_reset = false;

		person* driver;


		car(const building* start, const building* target, person* dr, const double max_sp = DEFAULT_ROAD_SPEED, double acc = DEFAULT_CAR_ACCELERATION, double br = DEFAULT_CAR_BRAKING) {
			if(find_path(*start, *target, path) && !path.empty()) {
				cur_segment = path.front();
				target_loc = (point)target->road_connection;
				position = cur_segment->start;
				for(const segment* s : path) output_segment(out_file, *s, 'c');
			} else {
				should_reset = true;
			}

			cur_speed = 0.0;
			max_speed = max_sp;
			driver = dr;
			accel = acc;
			brake = br;

		}

		Coordinate update_position() {
			double ratio = segment_position / cur_segment->length;

			position = Coordinate(lerp(cur_segment->start.x, cur_segment->end.x, ratio), lerp(cur_segment->start.y, cur_segment->end.y, ratio));
			return position;
		}

		//explicit bool operator==(const car &other) const noexcept {
		//	return cur_segm
		//}
		void reset() {
			should_reset = true;
		}
		void set_speed(double s) { // sets and clamps speed
			cur_speed = min(min(max(s, 0.0), max_speed), cur_segment->max_speed);
		}
		double get_max_speed() {
			return min(max_speed, cur_segment->max_speed);
		}
		double max_dist(double dt) { // calculate maximum distance reachable in dt (delta time)
			double max_v = get_max_speed();
			double base_dist = cur_speed * dt;
			double max_point = (max_v - cur_speed) / accel;
			/*
			 * 13.8  ---------------------
			 *      /|                   |
			 *     / |        2          |
			 *    / 1|                   |
			 *   |---|-------------------| <
			 *   |   |                   | < base_dist
			 *   |   |                   | <
			 * 0 -------------------------
			         ^
			       max_point

			 */

			double s_1 = max_point < EPSILON ? 0.0 : min(max_point, dt) * max_v / max_point / 2.0;
			//                 (1)   (                 2                )
			return base_dist + s_1 + max(dt - max_point, 0.0) * max_v;
		}
		double min_dist(double dt) { // calculate minimum distance reachable in dt (delta time)
			double min_point = min(cur_speed / brake, dt);
			double dv = min_point * brake;
			double top = (dv) * min_point / 2.0;

			/*
			 * cur_speed
			 *   ↓
			 *   |\
			 *   | \ (top)
			 *   |  \
			 *   |---|  < cur_speed - dv
			 *   |   |
			 *   | 1 |
			 *   |   |
			 * 0 -----
			         ^
				   min_point
			 */
			//           (             1             )
			return top + (cur_speed - dv) * min_point;
		}


		constexpr double area(double v1, double accel, double len) {
			double v2 = v1 + accel * len;
			return min(v1, v2) * len + len * abs(v2-v1) / 2.0;
		}

		void move_dist(double dist, double dt = -1.0) {
			//if(dist < MIN_DIST) dist = 0.0;
			if(obst > 0.0) dist = max(min(dist, obst - DEFAULT_CAR_RADIUS), 0.0);
			if(dt > 0.0) { // dt being positive signals recalculaion of cur_speed
				double mid = dist/dt;
				set_speed(cur_speed + (mid-cur_speed) * 2.0); // linear extrapolation
			}
			if(cur_speed > EPSILON) segment_position += dist;
			while(segment_position >= cur_segment->length) { // moving into next segment
				segment_position -= cur_segment->length;


				for (vector<car*>::iterator it = cur_segment->cars.begin(); it != cur_segment->cars.end(); it++)
				{
					if (*it == this) {
						cur_segment->cars.erase(it);
						break;
					}
				}
				path.erase(path.begin());
				//(void)(VERBOSE && cout << "segments left: " << path.size() << endl);
				if(path.empty() || (point)path.front()->start == target_loc) {
					reset();
					return;
				}

				// slow down for turns, formula: https://www.desmos.com/calculator/to2fkqowrh
				double angle = cur_segment->angle(*path.front());
				cur_speed *= abs(cos(angle / (get_max_speed() / cur_speed * 2.3)));

				cur_segment = path.front();
				cur_segment->cars.push_back(this);

				// setting next_segment for lane checking
				if(next_segment == &short_segments[*cur_segment] || next_segment == nullptr) {
					for(segment* s : path) {
						if(short_segments[*s] != short_segments[*cur_segment]) {
							next_segment = &short_segments[*s];
							goto endif;
						}
					}
					next_segment = nullptr;
				}
				endif:;
			}
		}
		constexpr double calc_k(double obst, double v_max) {
			// k is coefficient of similar triangles - negative values means that v(t) does not cross v_max
			double common = brake * (-cur_speed * cur_speed - 2 * accel * obst);
			return (v_max * v_max * (brake + accel) + common)
			/ (2 * v_max * cur_speed * (brake + accel) - v_max * v_max * (brake - accel) + common);
		}

		constexpr double calc_tstop2(double obst) {
			// formula for t_stop2 when k < 0
			return (-(brake + accel) * cur_speed + sqrt((cur_speed * cur_speed * brake) * (brake + accel) + 2*(accel+brake+obst) * (accel * brake)))
				/ ((brake + accel) * accel);
		}
		//constexpr double area(double v1, double v2) {}

		// TODO: fix movement calculation, implement proper visualization, document, then finish up
		void move(double dt) { // dt - delta time
			double min_braking_dist = get_max_speed() * (get_max_speed() / brake) / 2.0;
			double vision = max_dist(dt) + min_braking_dist;
			obst = closest_obst(this, vision);
			double min_d = min_dist(dt);
			double max_d = max_dist(dt);

			if(obst < 0.0) {
				move_dist(max_d); // no obstacles ahead, go full power
				set_speed(cur_speed + dt * accel);
				return;
			} else {
				if(obst <= DEFAULT_CAR_RADIUS) {obst = 0.0; move_dist(0.0); cur_speed = 0.0; return;}
				double v_max = get_max_speed();
				if(v_max - cur_speed <= EPSILON) {
					double t_stop = (2*brake*obst - cur_speed * cur_speed)
								/(2*brake*cur_speed);
					if(t_stop < 0) move_dist(min_d, dt);
					else if(t_stop > dt) move_dist(max_d, dt);
					else {
						move_dist(t_stop * cur_speed);
						move_dist(area(cur_speed, -brake, dt-t_stop), dt-t_stop);
					}
					return;
				}
				// formula for movement calculation: https://www.desmos.com/calculator/zlqnsvq0d1
				// calculates the time at which the car should stop to minimize the time it takes to reach obst
				double k = calc_k(v_max, obst);

				double t_stop = (v_max - cur_speed) / (accel * (1-k)); // negative t_stop means that the crossing is behind the x axis
				if(t_stop <= 0) move_dist(min_d, dt); // t_stop
				else if(k > 0){
					double t_max1 = max(0.0, (v_max - cur_speed) / accel);
					double t_max2 = t_stop + (accel * t_stop * k) / brake;
					if(t_max2 > dt) move_dist(max_d, dt);
					else {
						move_dist(area(cur_speed, accel, t_max1));
						set_speed(v_max);
						move_dist(area(cur_speed, 0, t_max2-t_max1));
						move_dist(area(cur_speed, brake, dt-t_max2), dt-t_max2);
					}
				} else {
					// negative k: obst is too far to reach maximum speed before meeting
					// negative k messes up the calculation for the other faces, so a different formula is needed
					//t_stop = (2 * brake * obst - cur_speed * cur_speed) * (accel + brake) / (2 * cur_speed + )
					double t_stop2 = calc_tstop2(obst);
					if(t_stop2 > dt) move_dist(max_d, dt);
					else if(t_stop2 < 0) move_dist(min_d, dt);
					else {
						move_dist(area(cur_speed, accel, t_stop2), t_stop2);
						move_dist(area(cur_speed, -brake, dt - t_stop2), dt - t_stop2);
					}
				}

				//cout << "approaching" << endl;

			}
		}
	};


	// returns the number of segments that can be reached by moving dist
	int segments_ahead(car* c, double dist, const vector<segment*>& segments) {
		int r = 0;
		segment* cur_seg = segments[r];
		dist -= (cur_seg->length - c->segment_position);
		while(dist > 0 && r < segments.size()-1) {
			cur_seg = segments[++r];
			dist -= cur_seg->length;
		}
		return r;
	}

	double closest_obst(car* c, double vision) {
		double min_dist = 1e20;
		for(car* other : c->cur_segment->cars) { // special case for current segment - check if the cars are in front
			// ignore different lanes
			if(c->next_segment == nullptr || other->next_segment == nullptr) continue;
			if(*(c->next_segment) != *(other->next_segment)) continue;
			if(other->segment_position - c->segment_position > 0.0) min_dist = min(min_dist, other->segment_position - c->segment_position);
		}
		if(c->cur_segment->is_red()) min_dist = min(min_dist, c->cur_segment->length - c->segment_position);

		// min_dist == 1e20 if no obstacles were found
		if(min_dist < 1e20) return min_dist;// - DEFAULT_CAR_RADIUS; // adding radius in order to leave distance between the cars

		// cur dist is the distance covered so far
		double cur_dist = c->cur_segment->length - c->segment_position;

		// segments_ahead returns how many segments are covered by vision
		int seg_count = segments_ahead(c, vision, c->path);
		for(int i = 1; i <= seg_count; i++) { // skip current segment
			// check cars
			for(car* other : c->path[i]->cars) {
				// ignore different lanes
				if(c->next_segment == nullptr || other->next_segment == nullptr) continue;
				if(*(c->next_segment) != *(other->next_segment)) continue;
				min_dist = min(min_dist, cur_dist + other->segment_position);
			}
			// check red light
			if(c->path[i]->is_red()) min_dist = min(min_dist, c->path[i]->length + cur_dist);

			// min_dist == 1e20 if no obstacles were found
			if(min_dist < 1e20) return min_dist;// - DEFAULT_CAR_RADIUS; // adding radius in order to leave distance between the cars

			// add current segment's length to cur_dist
			cur_dist += c->path[i]->length;
		}

		return -1.0;
	}

	/*
	bool is_colliding(Coordinate cor, car* c) {
		return coord_dist(cor, c->position) < DEFAULT_CAR_RADIUS;
	}

	bool is_colliding(Coordinate cor, segment* s) {
		if(s->is_red() && coord_dist(s->end, cor) < DEFAULT_CAR_RADIUS) return true;
		for(car* c : s->cars) {
			if(is_colliding(cor, c)) return true;
		}
		return false;
	}

	bool is_colliding(Coordinate cor, vector<segment*> vs) {
		for(segment* s : vs) {
			if(is_colliding(cor, s)) return true;
		}
		return false;
	}*/

	static ostream& operator<<(ostream& out, const car& c)
	{
		out << "car " << c.position;
		return out;
	}

	struct bus : car {
		int route_id;
		vector<Coordinate> stops;
		vector<person*> passengers;
	};

	//deque<car> cars; // cars will be stored in person

	struct person {
		string name;
		int age;
		bool can_drive;
		const building* home;
		optional<car> p_car;

		building* work = nullptr; // or any place that is visited regularly

		person(string n, int a, bool cd, const building& h) {
			name = n;
			age = a;
			can_drive = cd;
			home = &h;
		}
		bool check_car() { // returns whether the car exists
			if(p_car && p_car->should_reset) {
				p_car.reset();
				return false;
			}
			return true;
		}
		void make_car(const building* from, const building* to, double max_speed = DEFAULT_ROAD_SPEED) {
			p_car = car(from, to, this, max_speed);
		}
	};

	vector<person> people;
	vector<building> workplaces;

	void people_init() {
		(void)(VERBOSE && cout << "GENERATING PEOPLE" << endl);
		auto t1 = chrono::high_resolution_clock::now();

		for(auto& rb : residential_buildings) {
			for(const building& b : rb.second) {
				for(int i = 0; i < b.capacity; i++) {
					int age = rng::random_int(0, 80);
					bool can_drive;
					if(age < 18) can_drive = false;
					else if(age < 50) can_drive = rng::random_chance(0.8);
					else can_drive = rng::random_chance(0.5);

					person p("", age, can_drive, b);
					people.push_back(p);
				}
			}
		}

		shuffle(people.begin(), people.end(), rng::mt_gen); // randomize ordering


		auto t2 = chrono::high_resolution_clock::now();
		(void)(VERBOSE && cout << "DONE" << endl);
		(void)(VERBOSE && cout << duration_cast<chrono::milliseconds>(t2 - t1) << " milliseconds" << endl);

	}
	void workplaces_init(Features& workplace_features) {
		(void)(VERBOSE && cout << "PARSING WORKPLACES" << endl);
		auto t1 = chrono::high_resolution_clock::now();

		int count = 0;
		for (const Feature& f : workplace_features)
		{
			workplaces.emplace_back(f, (f["building"] == "school") ? building::SCHOOL : building::WORKPLACE);
			cout << "Workplace: " << workplaces.back().name << ", Capacity: " << workplaces.back().capacity << ", Location: " << workplaces.back().location << endl;
			count += workplaces.back().capacity;
		}

		for(building& w : workplaces) {
			for(int i = 0; i < w.capacity; i++) {

				while(true) {
					person& p = people[rng::random_int(0, people.size()-1)];
					if(p.work) continue;
					if(p.age <= 6) break;
					if(w.type == building::SCHOOL) {
						if(p.age <= 18 || rng::random_chance(0.1)) {p.work = &w; break;}
					} else {
						if(p.age >= 18 && p.age <= 63) {p.work = &w; break;}
					}

				}

			}
		}
		auto t2 = chrono::high_resolution_clock::now();
		(void)(VERBOSE && cout << "Total positions: " << count << endl);
		(void)(VERBOSE && cout << "DONE" << endl);
		(void)(VERBOSE && cout << duration_cast<chrono::milliseconds>(t2 - t1) << " milliseconds" << endl);
	}

	struct event {
		double time;
		enum type {
			GO_TO_WORK,
			GO_HOME,
			GO_TO_ENTERTAINMENT,
			GO_RANDOM
		};
		type t;
		person* target;
		void call() const {
			switch(t) {
				case GO_TO_WORK: target->make_car(target->home, target->work); break;
				case GO_HOME: target->make_car(target->work, target->home); break;
				case GO_TO_ENTERTAINMENT: target->make_car(target->work, target->home); break;
				case GO_RANDOM: target->make_car(target->home, &workplaces[rng::random_int(0, workplaces.size()-1)]); break;
			}

			if(target->check_car()) target->p_car->cur_segment->cars.push_back(&(target->p_car.value()));
		}
		event(double time, type t, person* target) {
			this->time = time;
			this->t = t;
			this->target = target;
		}
		bool operator>(const event other) const noexcept{
			return time > other.time;
		}
	};

	priority_queue<event, vector<event>, greater<event>> events;

	void events_init() {
		(void)(VERBOSE && cout << "CREATING EVENTS" << endl);
		auto t1 = chrono::high_resolution_clock::now();
		for(person& p : people) {
			if(!p.can_drive) continue;
			if(!p.work) {
				// random events
				if (rng::random_chance(RANDOM_EVENT_CHANCE)) {
					events.emplace(
						rng::random_double(0, SIM_LENGTH),
						event::GO_RANDOM,
						&p
					);
				}
				continue;
			}
			events.emplace(
				time_hms(8, 30+rng::normal_int(-30, 30), rng::random_double(-60, 60)), // going to work
				//time_hms(0, 1, rng::random_double(-60, 60)),
				//time_hms(0, 0, 0),
				event::GO_TO_WORK,
				&p
			);
			events.emplace(
				time_hms(17, 30+rng::normal_int(-30, 30), rng::random_double(-60, 60)),// going home
				//time_hms(17, 30+rng::normal_int(-30, 30), rng::random_double(-60, 60)),
				event::GO_HOME,
				&p
			);
			if(events.size() > MAX_CARS) break;
			//cout << "age " << p.age << " - home: " <<  (coord_from_point(p.home->location));
			//if(p.work) cout << "; work: " << (coord_from_point(p.work->location));
			//cout << "; can drive: " << p.can_drive << endl;
		}
		auto t2 = chrono::high_resolution_clock::now();
		(void)(VERBOSE && cout << "Total events: " << events.size() << endl);
		(void)(VERBOSE && cout << "DONE" << endl);
		(void)(VERBOSE && cout << duration_cast<chrono::milliseconds>(t2 - t1) << " milliseconds" << endl);
	}
	void sim_tick(double dt) {
		cur_time += dt;
		cout << "PROGRESS: " << cur_time << " / " << SIM_LENGTH << endl;
		out_file << "t " << cur_time << endl;
		while(!events.empty() && events.top().time <= cur_time) {
			events.top().call();
			events.pop();
		}

		for(person &p : people) {
			if (p.p_car && p.check_car()) {
				p.p_car->move(dt);
				p.p_car->update_position();
				out_file << p.p_car.value() << endl;
			}
		}
		for(point p : traffic_light_nodes) {
			for(segment &s : graph[p]) {
				if(s.off_time > 0) output_traffic_light(out_file, s);
			}
		}
	}
}
