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
    bool setRoadStatus(const string &city1, const string &city2, bool curStatus)
    {

        auto city1Lookup = cityToNumber.find(city1);
        auto city2Lookup = cityToNumber.find(city2);
        // chrck if the cities exist
        if (cityToNumber.find(city1) == cityToNumber.end() || cityToNumber.find(city2) == cityToNumber.end())
            return false;
        // get the node or number of the cities
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

        roadsFromCity[city1Num][roadIndex1].isOpen = curStatus;
        roadsFromCity[city2Num][roadIndex2].isOpen = curStatus;

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


    // Find shortest path from a starting city to all other cities (Dijkstra Algo)
    pair<vector<ll>, vector<int>> findShortestPaths(const string &startCity) const
    {
        
        vector<ll> shortestDistances(totalCities, INT_MAX);
        vector<int> previousCity(totalCities, -1);

        // Look up the starting city
        auto cityLookup = cityToNumber.find(startCity);
        if (cityLookup == cityToNumber.end())
            return {shortestDistances, previousCity};

        int startCityNum = cityLookup->second;
        using CityDistance = pair<ll, int>;
        priority_queue<CityDistance, vector<CityDistance>, greater<CityDistance>> citiesToCheck;

        shortestDistances[startCityNum] = 0;
        citiesToCheck.push({0, startCityNum});

        // Dijkstra loop
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

    // Same function as above but it finds the using the number of the starting city
    pair<vector<ll>, vector<int>> findShortestPathsFromNumber(int startCityNum) const
    {
        vector<ll> shortestDistances(totalCities, INT_MAX);
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


    // Build the actual path by going backwards from destination
    vector<int> buildPath(const vector<int> &previousCity, int destinationCity) const
    {
        vector<int> path;
        int currentCity = destinationCity;

        while (currentCity != -1)
        {
            path.push_back(currentCity);
            currentCity = previousCity[currentCity];
        }

        reverse(path.begin(), path.end());

        return path;
    }

    // Save the map to a file
    bool saveMapToFile(const string &filename) const
    {
        ofstream file(filename);
        if (!file)
            return false;

        // Save number of cities
        file << totalCities << "\n";
        // Save city names + coordinates
        for (int i = 0; i < totalCities; i++)
        {
            double lat = cityLatitudes[i];
            double lon = cityLongitudes[i];
            file << numberToCity[i];
            // Write coordinates only if valid
            if (!isnan(lat) && !isnan(lon))
                file << " " << fixed << setprecision(8) << lat << " " << lon;
            file << "\n";
        }

    // Save roads (avoid duplicate undirected edges)
        set<pair<int, int>> savedRoads;

        for (int cityNum = 0; cityNum < totalCities; cityNum++)
        {
            for (const auto &road : roadsFromCity[cityNum])
            {
                int otherCity = road.connectedCity;
                // Only save each road once
                if (cityNum >= otherCity || !savedRoads.insert({cityNum, otherCity}).second)
                    continue;
                // Basic road info
                file << numberToCity[cityNum] << " "
                     << numberToCity[otherCity] << " "
                     << road.travelTime << " "
                     << (road.isOpen ? 1 : 0) << " ";
                // Convert spaces in road name to underscores
                string roadNameForFile = road.roadName;
                for (char &c : roadNameForFile)
                    if (c == ' ')
                        c = '_';

                file << roadNameForFile << "\n";
            }
        }

        return true;
    }


    // Load a map from a file
    bool loadMapFromFile(const string &filename)
    {
        ifstream file(filename);

        if (!file)
            return false;

        // Clear existing data
        cityToNumber.clear();
        numberToCity.clear();
        roadsFromCity.clear();
        cityLatitudes.clear();
        cityLongitudes.clear();
        totalCities = 0;

        // Read number of cities
        int numCities;
        file >> numCities;
        string line;
        getline(file, line);

        for (int i = 0; i < numCities; i++)
        {
            getline(file, line);
            if (!line.empty() && line.back() == '\r')
                line.pop_back();
            if (line.empty())
                continue;
            stringstream ss(line);
            string cityName;
            ss >> cityName;
            double lat, lon;
            if (ss >> lat >> lon)
            {
                makeSureCityExists(cityName);
                int cityNum = cityToNumber[cityName];
                cityLatitudes[cityNum] = lat;
                cityLongitudes[cityNum] = lon;
            }
            else
            {
                makeSureCityExists(cityName);
            }
        }

        string city1, city2, roadNameFromFile;
        ll travelTime;
        int isOpenInt;

        while (file >> city1 >> city2 >> travelTime >> isOpenInt >> roadNameFromFile)
        {
            for (char &c : roadNameFromFile)
                if (c == '_')
                    c = ' ';
            connectCities(city1, city2, travelTime, roadNameFromFile);
            bool isOpen = (isOpenInt == 1);
            setRoadStatus(city1, city2, isOpen);
        }

        return true;
    }

//function to change the trafficweight using the gui menu button
   bool simulateTrafficChange(mt19937 &randomGen, double minFactor, double maxFactor,
                           string &affectedCity1, string &affectedCity2, ll &newTime)
{
    // If no cities exist, nothing to update
    if (totalCities < 1)
        return false;

    // Collect all open roads (each undirected road only once)
    vector<pair<int, int>> openRoads;

    for (int cityNum = 0; cityNum < totalCities; cityNum++)
    {
        for (int roadIdx = 0; roadIdx < (int)roadsFromCity[cityNum].size(); roadIdx++)
        {
            const auto &road = roadsFromCity[cityNum][roadIdx];

            // Only consider open roads; avoid duplicates by checking cityNum < connectedCity
            if (road.isOpen && cityNum < road.connectedCity)
                openRoads.emplace_back(cityNum, roadIdx);
        }
    }

    // No open roads to modify
    if (openRoads.empty())
        return false;

    // Pick a random open road
    uniform_int_distribution<size_t> pickRoad(0, openRoads.size() - 1);
    size_t chosenRoad = pickRoad(randomGen);

    int city1Num = openRoads[chosenRoad].first;
    int roadIdx = openRoads[chosenRoad].second;
    int city2Num = roadsFromCity[city1Num][roadIdx].connectedCity;

    // Random traffic multiplier
    uniform_real_distribution<double> pickFactor(minFactor, maxFactor);
    double trafficMultiplier = pickFactor(randomGen);

    // Calculate updated travel time
    ll oldTime = roadsFromCity[city1Num][roadIdx].travelTime;
    ll updatedTime = max(1LL, (ll)round(oldTime * trafficMultiplier));

    // Find reverse road index (city2 -> city1)
    int reverseRoadIdx = -1;
    for (int i = 0; i < (int)roadsFromCity[city2Num].size(); i++)
    {
        if (roadsFromCity[city2Num][i].connectedCity == city1Num)
        {
            reverseRoadIdx = i;
            break;
        }
    }

    // Roads must always be bidirectional, so missing reverse road means error
    if (reverseRoadIdx == -1)
        return false;

    // Update travel time in both directions
    roadsFromCity[city1Num][roadIdx].travelTime = updatedTime;
    roadsFromCity[city2Num][reverseRoadIdx].travelTime = updatedTime;

    // Return which cities were affected and the new time
    affectedCity1 = numberToCity[city1Num];
    affectedCity2 = numberToCity[city2Num];
    newTime = updatedTime;

    return true;
}

    // Find shortest path between two GPS coordinates
    pair<bool, vector<string>> findPathByCoordinates(double startLat, double startLon,
                                                     double endLat, double endLon,
                                                     ll &totalTime) const
    {
        // Find nearest cities to the GPS points
        int startCity = findClosestCity(startLat, startLon);
        int endCity = findClosestCity(endLat, endLon);

        if (startCity == -1 || endCity == -1)
        {
            totalTime = -1;
            return {false, {}};
        }

        // Find shortest path
        auto pathData = findShortestPathsFromNumber(startCity);
        auto distances = pathData.first;
        auto previousCities = pathData.second;

        // Check if destination is reachable
        if (endCity < 0 || endCity >= (int)distances.size() ||
            distances[endCity] >= VERY_LARGE_NUMBER)
        {
            totalTime = -1;
            return {false, {}};
        }

        // Build the path
        auto pathNumbers = buildPath(previousCities, endCity);

        // Convert city numbers to city names
        vector<string> cityNames;
        {
            for (int cityNum : pathNumbers)
                cityNames.push_back(numberToCity[cityNum]);
        }

        totalTime = distances[endCity];
        return {true, cityNames};
    }
    // Find a route that goes through multiple cities in order
    pair<bool, vector<string>> findRouteWithStops(const vector<string> &citiesInOrder,
                                                  ll &totalTime) const
    {
        vector<string> fullPath;
        totalTime = 0;

        // Go through each pair of cities
        for (size_t i = 0; i < citiesInOrder.size() - 1; i++)
        {
            auto [distances, previousCities] = findShortestPaths(citiesInOrder[i]);


            auto cityLookup = cityToNumber.find(citiesInOrder[i + 1]);
            if (cityLookup == cityToNumber.end())
                return {false, {}};

            int nextCityNum = cityLookup->second;

            // Check if next city is reachable
            if (distances[nextCityNum] >= VERY_LARGE_NUMBER)
                return {false, {}};

            // Add distance to total
            totalTime += distances[nextCityNum];

            // Build this part of the path
            auto pathSegment = buildPath(previousCities, nextCityNum);

            if (i > 0 && !pathSegment.empty())
                pathSegment.erase(pathSegment.begin());

            for (int cityNum : pathSegment)
                fullPath.push_back(numberToCity[cityNum]);
        }

        return {true, fullPath};
    }

};

void backgroundTrafficSimulation(RoadMap &map,
                                 atomic<bool> &programRunning,
                                 atomic<bool> &simulationOn,
                                 function<void(const string &)> printMessage)
{
    mt19937 rng((unsigned)chrono::steady_clock::now().time_since_epoch().count());

    const double minFactor = 0.7;   // 30% faster
    const double maxFactor = 1.4;   // 40% slower
    const int waitSec = 5;

    while (true)
    {
        // If program not running yet, wait a bit
        if (!programRunning)
        {
            this_thread::sleep_for(chrono::milliseconds(200));
            continue;
        }

        // Wait before trying to update
        this_thread::sleep_for(chrono::seconds(waitSec));

        // Skip if program stopped or simulation paused
        if (!programRunning || !simulationOn)
            continue;

        string cityA, cityB;
        ll newTime = -1;

        bool ok = map.simulateTrafficChange(rng, minFactor, maxFactor,
                                            cityA, cityB, newTime);

        if (ok && printMessage)
        {
            printMessage("[Traffic Update] " + cityA + " <-> " + cityB +
                         " now has travel time " + to_string(newTime));
        }
    }
}

int main()
{
    RoadMap indianRoads;

    // Add some sample roads between major cities
    indianRoads.connectCities("Delhi", "Jaipur", 280, "NH48");
    indianRoads.connectCities("Delhi", "Chandigarh", 250, "NH44");
    indianRoads.connectCities("Delhi", "Lucknow", 530, "NH27");
    indianRoads.connectCities("Delhi", "Bengaluru", 2100, "NH44");

    // Add state capitals with their GPS coordinates
    vector<pair<string, pair<double, double>>> stateCapitals = {
        {"Bhubaneswar", {20.2961, 85.8245}},
        {"Jaipur", {26.9124, 75.7873}},
        {"Gangtok", {27.3389, 88.6065}},
        {"Chennai", {13.0827, 80.2707}},
        {"Hyderabad", {17.3850, 78.4867}},
        {"Agartala", {23.8315, 91.2868}},
        {"Lucknow", {26.8467, 80.9462}},
        {"Dehradun", {30.3165, 78.0322}},
        {"Kolkata", {22.5726, 88.3639}}};

    // Set GPS coordinates for all state capitals
    for (auto &capital : stateCapitals)
    {
        indianRoads.setCityLocation(capital.first, capital.second.first, capital.second.second);
    }

    // Display the current map
    string mapInfo;
    indianRoads.displayMapInfo(mapInfo);
    cout << mapInfo << endl;

    // Find and display the shortest route from Delhi to Jaipur
    auto [distances, previousCities] = indianRoads.findShortestPaths("Delhi");
    int jaipurNumber = indianRoads.cityToNumber["Jaipur"];

    if (distances[jaipurNumber] < VERY_LARGE_NUMBER)
    {
        auto pathNumbers = indianRoads.buildPath(previousCities, jaipurNumber);

        cout << "\nShortest path from Delhi to Jaipur: ";
        for (size_t i = 0; i < pathNumbers.size(); i++)
        {
            cout << indianRoads.numberToCity[pathNumbers[i]];
            if (i + 1 < pathNumbers.size())
                cout << " -> ";
        }
        cout << "\nTotal distance: " << distances[jaipurNumber] << endl;
    }

    // Save the map to a file
    indianRoads.saveMapToFile("map.dat");

    // Start the traffic simulation in the background
    atomic<bool> keepProgramRunning(true);
    atomic<bool> trafficSimulationOn(true);

    thread trafficThread(backgroundTrafficSimulation, ref(indianRoads),
                         ref(keepProgramRunning), ref(trafficSimulationOn),
                         [](const string &msg)
                         { cout << msg << endl; });

    // Let the program run for 30 seconds
    this_thread::sleep_for(chrono::seconds(30));

    // Stop the program
    keepProgramRunning = false;

    // Wait for the traffic thread to finish
    if (trafficThread.joinable())
        trafficThread.join();

    return 0;
}
