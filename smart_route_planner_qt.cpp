// This is the main backend code with all the used functions
// neccesary comments have been put to make it more easy to  unnderstand

#include <bits/stdc++.h>
#include <thread>
#include <atomic>
#include <random>
#include <mutex>

using namespace std;
using ll = long long;

// flag to control if the program is running
static atomic<bool> programRunning(true);

// Represents a road connecting two cities
struct Road
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




