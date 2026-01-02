// traffic_map.cpp : Defines the entry point for the application.
//

#include "traffic_map.h"


#pragma execution_character_set("utf-8")

using namespace std;
using namespace geodesk;
using namespace traffic_sim;
using namespace config;

static ostream& operator<<(ostream& out, const Coordinate& coord)
{
	out << "(" << Mercator::latFromY(coord.y) << ", " << Mercator::lonFromX(coord.x) << ")";
	return out;
}

int main()
{


	ios_base::sync_with_stdio(false);
	cin.tie(0);
	cout.tie(0);
	SetConsoleOutputCP(65001); // CP_UTF8
	cout << setprecision(9);
	cout << "Hello CMake." << endl;

	Features features("D:\\storage\\traffic_map\\traffic_map\\bulgaria.gol");
	Feature varna = features("a[admin_level=5][name:en=Varna]").one(); // whole varna
	Features features_in_varna = features.within(varna);


	// temp for testing in vuzrazhdane
	/*
	const Feature* vuzrazhdane;
	
	for (const Feature& f : features_in_varna) {
		if (f.id() == 16951190) { vuzrazhdane = &f; break; }
	}
	features_in_varna = features_in_varna.intersecting(*vuzrazhdane);
	*/
	Ways roads = features_in_varna.ways("[highway=motorway,trunk,primary,secondary,tertiary,residential,road]");
	Relations bus_routes = features_in_varna.relations("r[route=bus][type=route]");
	Ways residential_areas = features_in_varna.ways("a[landuse=residential]");
	Ways parkings = features_in_varna.ways("a[amenity=parking]");
	
	//Features workplace_features = features_in_varna("a[disused:shop=mall]");
	//Features workplace_features = features_in_varna("a[shop=mall]");
	//Nodes workplace_nodes = features_in_varna.nodes("[shop],[amenity][amenity!=bicycle_parking,bicycle_repair_station,bicycle_rental,bicycle_wash,bus_station,compressed_air,charging_station,driver_training,grit_bin,motorcycle_parking,parking,parking_entrance,parking_space,taxi,weighbridge,atm,payment_terminal,baby_hatch,fountain,stage,studio,post_box,bbq,bench,check_in,dog_toilet,dressing_room,drinking_water,give_box,lounge,mailroom,parcel_locker,shelter,shower,telephone,toilets,water_point,watering_place,sanitary_dump_station,recycling,waste_basket,waste_disposal,baking_oven,clock,grave_yard,hunting_stand,kitchen,kneipp_water_cure,lounger,photo_booth,place_of_mourning,place_of_worship,public_bath,vending_machine,hydrant]");
	map<int64_t, vector<building>> residential_buildings; // map of residential area id to buildings in that area

	int count = 0;

	for (const Way& a : residential_areas) {
		cout << "Residential Area: " << a["name"] << endl;
		Ways res_areas = features_in_varna.ways("a[building=residential,house,apartments,detached,terrace]").intersecting(a);
		for (const Way& f : res_areas) {
			building b(f, building::RESIDENTIAL, ((f["building"] == "house") ? 0.002 : 0.0)); // lower density for houses
			residential_buildings[a.id()].push_back(b);
			cout << "Residential Building: " << b.name << ", Capacity: " << b.capacity << ", Location: " << b.location->centroid() << endl;
			count += b.capacity;
		}
	}
	cout << "Total residents: " << count << endl;

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
