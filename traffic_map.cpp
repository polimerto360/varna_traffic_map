// traffic_map.cpp : Defines the entry point for the application.
//

#include "traffic_map.h"
#include <windows.h>

#pragma execution_character_set("utf-8")

using namespace std;
using namespace geodesk;

static ostream& operator<<(ostream& out, const Coordinate& coord)
{
	out << "(" << coord.x << ", " << coord.y << ")";
	return out;
}

int main()
{
	ios_base::sync_with_stdio(false);
	cin.tie(0);
	cout.tie(0);
	SetConsoleOutputCP(65001); // CP_UTF8

	cout << "Hello CMake." << endl;

	Features features("D:\\storage\\traffic_map\\traffic_map\\bulgaria.gol");
	Feature varna = features("a[admin_level=5][name:en=Varna]").one();
	Features features_in_varna = features.intersecting(varna);

	Ways roads = features_in_varna.ways("[highway=motorway,trunk,primary,secondary,tertiary,residential,road]");
	Relations bus_routes = features_in_varna.relations("r[route=bus][type=route]");
	Ways residential_areas = features_in_varna.ways("a[landuse=residential]");
	Ways workplaces = features_in_varna.ways("a[building=office,industrial]");
	Nodes workplace_nodes = features_in_varna.nodes("[shop],[amenity][amenity!=bicycle_parking,bicycle_repair_station,bicycle_rental,bicycle_wash,bus_station,compressed_air,charging_station,driver_training,grit_bin,motorcycle_parking,parking,parking_entrance,parking_space,taxi,weighbridge,atm,payment_terminal,baby_hatch,fountain,stage,studio,post_box,bbq,bench,check_in,dog_toilet,dressing_room,drinking_water,give_box,lounge,mailroom,parcel_locker,shelter,shower,telephone,toilets,water_point,watering_place,sanitary_dump_station,recycling,waste_basket,waste_disposal,baking_oven,clock,grave_yard,hunting_stand,kitchen,kneipp_water_cure,lounger,photo_booth,place_of_mourning,place_of_worship,public_bath,vending_machine,hydrant]");
	Ways parkings = features_in_varna.ways("a[amenity=parking]");

	int count = 0;

	for (const Node& node : workplace_nodes)
	{
		count++;
		
		cout << "workplace: " << node["name"] << "; pos: " << node.xy() << endl;
	}
	cout << "Total workplace: " << count << endl;
	return 0;
}
