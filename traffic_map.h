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

// TODO: Reference additional headers your program requires here.

namespace config {
	const double TIME_STEP = 1.0; // time step in seconds
	const double DEFAULT_BUILDING_HEIGHT = 5.0; // default building height in meters
	// target workplaces is 150000
	const double EMPLOYEE_DENSITY = 0.038; // people per cubic meter (adding 0.01 increases result by 56309)
	const int MIN_EMPLOYEES = 5; // minimum people per workplace (adds or subtracts 2861)

	// target residents is 350000
	const double RESIDENTIAL_DENSITY = 0.012;
	const int MIN_RESIDENTS = 3;

	const double ENTERTAINMENT_DENSITY = 0.024; 
	const int MIN_VISITORS = 5; 
	
	const double DEFAULT_ROAD_SPEED = 50.0; // default road speed in km/h

	//const int NUM_CARS = 1000; // total number of cars in the simulation
}

namespace traffic_sim
{
	using namespace std;
	using namespace geodesk;
	using namespace config;

	typedef int64_t point; // hash for node coordinates

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
		return coord_dist(coord_from_point(a), coord_from_point(b));
	}

	struct segment; // forward declarations
	struct person; 
	struct workplace;
	struct car;

	struct person {
		string name;
		int age;
		bool can_drive;
		Feature* home;

		workplace* work; // or any place that is visited regularly

		person(string n, int a, bool cd, Feature* h, workplace* w) {
			name = n;
			age = a;
			can_drive = cd;
			home = h;
			work = w;
		}
	};

	struct segment { // directed road segment
		Coordinate start;
		Coordinate end;
		double length;

		vector<car*> cars;
		double max_speed; // maximum speed allowed on the segment
		bool operator==(const segment& s) const
		{
			return s.start == start && s.end == end && s.length == length;
		}

		segment(const Node& s, const Node& e, const double ms, const double len = 0) {
			start = s.xy();
			end = e.xy();
			max_speed = ms;
			length = len;
			if(len == 0) length = coord_dist(start, end);
		}
		segment(point _p1, point _p2, const double ms, const double len = 0) {
			start = coord_from_point(_p1);
			end = coord_from_point(_p2);
			max_speed = ms;
			length = len;
			if(len == 0) length = coord_dist(start, end);
		}
		segment(const segment &s)
		{
			start = s.start;
			end = s.end;
			max_speed = s.max_speed;
			length = s.length;
		}
		segment() = default;

		uint64_t h() const { // hash function (segment direction matters) - preserves as much information as possible while still being a 64 bit int
			return (first_half(start.x) << 48) |
			(first_half(start.y) << 32) |
			(first_half(end.x) << 16) |
			first_half(end.y);
		}

	};


	struct car {
		Coordinate position;
		double cur_speed;
		double max_speed;
		segment* cur_segment;

		person* driver;

		car(const Coordinate pos, const double max_sp, segment& seg, person& dr) {
			position = pos;
			cur_speed = 0.0;
			max_speed = max_sp;
			cur_segment = &seg;
			driver = &dr;
		}
	};

	struct building {
		enum building_type {
			RESIDENTIAL,
			WORKPLACE,
			ENTERTAINMENT
		};
		string name;
		building_type type;
		Feature* location;
		int capacity; // number of residents/people/employees
		int cur_employees = 0;

		building(Feature loc, building_type t, double people_density = 0.0) {
			name = loc["name"].storedString();
			type = t;
			
			if(name == "") {
				if (t == RESIDENTIAL) name = "Unnamed Residential Building";
				else if(t == ENTERTAINMENT) name = "Unnamed Entertainment Building";
				else name = "Unnamed Workplace";
			}

			location = &loc;
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
					if (people_density == 0.0)
						people_density = EMPLOYEE_DENSITY; // employees per cubic meter
					min_cap = MIN_EMPLOYEES;
					break;
			}

			capacity = max((int)(height * loc.area() * people_density), min_cap);
		}

	};


	struct bus : car {
		int route_id;
		vector<Coordinate> stops;
		vector<person*> passengers;
	};

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
}
template<>
struct std::hash<traffic_sim::segment> {
	size_t operator()(const traffic_sim::segment& s) const {
		return s.h();
	}
};
