// This is the main backend code with all the used functions
// neccesary comments have been put to make it more easy to  unnderstand

#include <bits/stdc++.h>
#include <thread>
#include <atomic>
#include <random>
#include <cmath>

using namespace std;
using ll = long long;

// Represents a road connecting two cities
struct Road //edge
{
    int connectedCity; // Which city this road goes to
    ll travelTime;     // How long it takes to travel 
    bool isOpen;       // Is the road open or blocked?
    string roadName;   // Name of the road 

    // constructor to create a new road
    Road(int city = 0, ll time = 0, bool open = true, string name = "")
    {
        connectedCity = city;
        travelTime = time;
        isOpen = open;
        roadName = name;
    }
};

struct RoadMap // struct to create graph
{
    // store all roads for cities
    vector<vector<Road>> roadsFromCity;

    // Convert city names to numbers
    unordered_map<string, int> cityToNumber;
    //conert city number to name
    vector<string> numberToCity;

    // to store location of cities
    vector<double> cityLatitudes;
    vector<double> cityLongitudes;

    int totalCities = 0;

    // to find if the city exists
    void makeSureCityExists(const string &cityName)
    {

        // check if city already exists
        if (cityToNumber.find(cityName) == cityToNumber.end())
        {
            // City doesn't exist, so add it
            cityToNumber[cityName] = totalCities;
            totalCities++;
            numberToCity.push_back(cityName);
            roadsFromCity.emplace_back();

            // Add empty coordinate
            cityLatitudes.push_back(NAN);
            cityLongitudes.push_back(NAN);
        }
    }

   // Add a new city to the map
    bool addNewCity(const string &cityName)
    {
        // If city already exists, return false
        if (cityToNumber.find(cityName) != cityToNumber.end())
            return false;

        // Add the new city
        cityToNumber[cityName] = totalCities;
        totalCities++;
        numberToCity.push_back(cityName);
        roadsFromCity.emplace_back();
        cityLatitudes.push_back(NAN);
        cityLongitudes.push_back(NAN);

        return true;
    }

  // Set the GPS coordinates for a city
    void setCityLocation(const string &cityName, double lat, double lon)
    {
        // If city doesn't exist, create it first
        if (cityToNumber.find(cityName) == cityToNumber.end())
        {
            cityToNumber[cityName] = totalCities;
            totalCities++;
            numberToCity.push_back(cityName);
            roadsFromCity.emplace_back();
            cityLatitudes.push_back(lat);
            cityLongitudes.push_back(lon);
        }
        else
        {
            // City exists, just update coordinates
            int cityNumber = cityToNumber[cityName];
            // if citylatitudes doesnt have enough space then increase it
            if (cityNumber >= cityLatitudes.size())
            {
            cityLatitudes.resize(cityNumber + 1, NAN);
            cityLongitudes.resize(cityNumber + 1, NAN);
            }
            // store the coordinates
            cityLatitudes[cityNumber] = lat;
            cityLongitudes[cityNumber] = lon;
        }
    }

    // get the coordinates of a city by its number
    pair<double, double> getCityLocation(int cityNumber) const
    {
        // Check if city number is valid
        if (cityNumber < 0 || cityNumber >= totalCities)
            return {NAN, NAN};

        return {cityLatitudes[cityNumber], cityLongitudes[cityNumber]};
    }

  // Find the closest city to given GPS coordinates for GUI pin point locations
    int findClosestCity(double lat, double lon) const
    {

        double smallestDistance = INFINITY;
        int closestCityNumber = -1;

        for (int i = 0; i < totalCities; i++)
        {
            double cityLat = cityLatitudes[i];
            double cityLon = cityLongitudes[i];

            if (isnan(cityLat) || isnan(cityLon))
                continue;

            double latDiff = cityLat - lat;
            double lonDiff = cityLon - lon;
            double distanceSquared = latDiff * latDiff + lonDiff * lonDiff;

            if (distanceSquared < smallestDistance)
            {
                smallestDistance = distanceSquared;
                closestCityNumber = i;
            }
        }

        return closestCityNumber;
    }

    // Add a road connecting two cities (works both ways)
    void connectCities(const string &city1, const string &city2, ll travelTime, const string &roadName = "")
    {
      
        // Make sure both cities exist, if not then add
        // city 1
        if (cityToNumber.find(city1) == cityToNumber.end())
        {
            cityToNumber[city1] = totalCities;
            totalCities++;
            numberToCity.push_back(city1);
            roadsFromCity.emplace_back();
            cityLatitudes.push_back(NAN);
            cityLongitudes.push_back(NAN);
        }
        //city2
        if (cityToNumber.find(city2) == cityToNumber.end())
        {
            cityToNumber[city2] = totalCities;
            totalCities++;
            numberToCity.push_back(city2);
            roadsFromCity.emplace_back();
            cityLatitudes.push_back(NAN);
            cityLongitudes.push_back(NAN);
        }

        int city1Number = cityToNumber[city1];
        int city2Number = cityToNumber[city2];

        roadsFromCity[city1Number].push_back(Road(city2Number, travelTime, true, roadName));
        roadsFromCity[city2Number].push_back(Road(city1Number, travelTime, true, roadName));
    }

    // Show all cities and roads in a readable format
    void displayMapInfo(string &output) const
    {
        output.clear();
        output += "Cities (" + to_string(totalCities) + "):\n";

        for (int i = 0; i < totalCities; i++)
        {
            output += "  - " + numberToCity[i];
            double lat = cityLatitudes[i];
            double lon = cityLongitudes[i];
            if (!isnan(lat) && !isnan(lon))
                output += " [lat=" + to_string(lat) + ", lon=" + to_string(lon) + "]";
            output += "\n";
        }
        
        output += "\nRoads:\n";
        set<pair<int, int>> shown;

        for (int city = 0; city < totalCities; city++)
        {
            for (const auto &road : roadsFromCity[city])
            {
                int other = road.connectedCity;
                if (city < other && shown.insert({city, other}).second)
                {
                    output += "  " + numberToCity[city] + " <-> " + numberToCity[other];
                    output += " | time: " + to_string(road.travelTime);
                    output += " | " + string(road.isOpen ? "open" : "blocked");
                    if (!road.roadName.empty())
                        output += " | name: " + road.roadName;
                    output += "\n";
                }
            }
        }
    }

    // Find which road connects to a specific city
    int findRoadTo(int fromCity, int toCity)
    {
        for (int i = 0; i < (int)roadsFromCity[fromCity].size(); i++)
        {
            if (roadsFromCity[fromCity][i].connectedCity == toCity)
                return i;
        }

        return -1;
    }

    // Open or close a road between two cities
    bool setRoadStatus(const string &city1, const string &city2, bool shouldBeOpen)
    {

        auto city1Lookup = cityToNumber.find(city1);
        auto city2Lookup = cityToNumber.find(city2);

        if (city1Lookup == cityToNumber.end() || city2Lookup == cityToNumber.end())
            return false;

        int city1Num = city1Lookup->second;
        int city2Num = city2Lookup->second;

        int roadIndex1 = -1;
        for (int i = 0; i < (int)roadsFromCity[city1Num].size(); i++)
        {
            if (roadsFromCity[city1Num][i].connectedCity == city2Num)
            {
                roadIndex1 = i;
                break;
            }
        }

        int roadIndex2 = -1;
        for (int i = 0; i < (int)roadsFromCity[city2Num].size(); i++)
        {
            if (roadsFromCity[city2Num][i].connectedCity == city1Num)
            {
                roadIndex2 = i;
                break;
            }
        }

        if (roadIndex1 == -1 || roadIndex2 == -1)
            return false;

        roadsFromCity[city1Num][roadIndex1].isOpen = shouldBeOpen;
        roadsFromCity[city2Num][roadIndex2].isOpen = shouldBeOpen;

        return true;
    }
    // Change the travel time on a road
    bool updateRoadTime(const string &city1, const string &city2, ll newTravelTime)
    {

        auto city1Lookup = cityToNumber.find(city1);
        auto city2Lookup = cityToNumber.find(city2);

        if (city1Lookup == cityToNumber.end() || city2Lookup == cityToNumber.end())
            return false;

        int city1Num = city1Lookup->second;
        int city2Num = city2Lookup->second;

        int roadIndex1 = -1;
        for (int i = 0; i < (int)roadsFromCity[city1Num].size(); i++)
        {
            if (roadsFromCity[city1Num][i].connectedCity == city2Num)
            {
                roadIndex1 = i;
                break;
            }
        }

        int roadIndex2 = -1;
        for (int i = 0; i < (int)roadsFromCity[city2Num].size(); i++)
        {
            if (roadsFromCity[city2Num][i].connectedCity == city1Num)
            {
                roadIndex2 = i;
                break;
            }
        }

        if (roadIndex1 == -1 || roadIndex2 == -1)
            return false;

        roadsFromCity[city1Num][roadIndex1].travelTime = newTravelTime;
        roadsFromCity[city2Num][roadIndex2].travelTime = newTravelTime;

        return true;
    }

    pair<vector<ll>, vector<int>> findShortestPaths(const string &startCity) const
    {
        vector<ll> shortestDistances(totalCities, VERY_LARGE_NUMBER);
        vector<int> previousCity(totalCities, -1);

        auto cityLookup = cityToNumber.find(startCity);
        if (cityLookup == cityToNumber.end())
            return {shortestDistances, previousCity};

        int startCityNum = cityLookup->second;

        using CityDistance = pair<ll, int>;
        priority_queue<CityDistance, vector<CityDistance>, greater<CityDistance>> citiesToCheck;

        shortestDistances[startCityNum] = 0;
        citiesToCheck.push({0, startCityNum});

        while (!citiesToCheck.empty())
        {
            auto [currentDistance, currentCity] = citiesToCheck.top();
            citiesToCheck.pop();

            if (currentDistance != shortestDistances[currentCity])
                continue;

            for (const auto &road : roadsFromCity[currentCity])
            {
                if (!road.isOpen)
                    continue;

                int neighborCity = road.connectedCity;
                ll newDistance = currentDistance + road.travelTime;

                if (newDistance < shortestDistances[neighborCity])
                {
                    shortestDistances[neighborCity] = newDistance;
                    previousCity[neighborCity] = currentCity;
                    citiesToCheck.push({newDistance, neighborCity});
                }
            }
        }

        return {shortestDistances, previousCity};
    }
    pair<vector<ll>, vector<int>> findShortestPathsFromNumber(int startCityNum) const
    {
        vector<ll> shortestDistances(totalCities, VERY_LARGE_NUMBER);
        vector<int> previousCity(totalCities, -1);

        if (startCityNum < 0 || startCityNum >= totalCities)
            return {shortestDistances, previousCity};

        using CityDistance = pair<ll, int>;
        priority_queue<CityDistance, vector<CityDistance>, greater<CityDistance>> citiesToCheck;

        shortestDistances[startCityNum] = 0;
        citiesToCheck.push({0, startCityNum});

        while (!citiesToCheck.empty())
        {
            auto [currentDistance, currentCity] = citiesToCheck.top();
            citiesToCheck.pop();

            if (currentDistance != shortestDistances[currentCity])
                continue;

            for (const auto &road : roadsFromCity[currentCity])
            {
                if (!road.isOpen)
                    continue;

                int neighborCity = road.connectedCity;
                ll newDistance = currentDistance + road.travelTime;

                if (newDistance < shortestDistances[neighborCity])
                {
                    shortestDistances[neighborCity] = newDistance;
                    previousCity[neighborCity] = currentCity;
                    citiesToCheck.push({newDistance, neighborCity});
                }
            }
        }

        return {shortestDistances, previousCity};
    }
};


