// traffic_map.h : Include file for standard system include files,
// or project specific include files.

#pragma once

#include <iostream>
#include <geodesk/geodesk.h>
#include <windows.h>
#include <vector>
#include <map>
#include <tuple>
#include <algorithm>
#include <cmath>
// TODO: Reference additional headers your program requires here.

namespace config {
	const double TIME_STEP = 1.0; // time step in seconds
	const double DEFAULT_BUILDING_HEIGHT = 5.0; // default building height in meters
	// target workplaces is 150000
	const double EMPLOYEE_DENSITY = 0.024; // people per cubic meter (adding 0.01 increases result by 56309)
	const int MIN_EMPLOYEES = 5; // minimum people per workplace (adds or subtracts 2861)

	// target residents is 350000
	const double RESIDENTIAL_DENSITY = 0.012;
	const int MIN_RESIDENTS = 3;

	const double ENTERTAINMENT_DENSITY = 0.024; 
	const int MIN_VISITORS = 5; 
	

	//const int NUM_CARS = 1000; // total number of cars in the simulation
}

namespace traffic_sim
{
	using namespace std;
	using namespace geodesk;
	using namespace config;

	struct segment;
	struct person; // forward declaration
	struct workplace; // forward declaration
	struct car; // forward declaration

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
		Node start;
		Node end;
		double length() {
			return sqrt(pow(end.x() - start.x(), 2) + pow(end.y() - start.y(), 2));
		}
		vector<car*> cars;
		double max_speed; // maximum speed allowed on the segment
	};
	struct car {
		Coordinate position;
		double cur_speed;
		double max_speed;
		segment* cur_segment;

		person* driver;

		car(Coordinate pos, double max_sp, segment seg, person dr) {
			position = pos;
			cur_speed = 0.0;
			max_speed = max_sp;
			cur_segment = &seg;
			driver = &dr;
		}
	};

	struct building {
		static enum building_type {
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
			name = loc["name"];
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
}
