//this is backenb code integrated with QT GUI 
//for implentation of raw backend code visit backend_code.cpp

#include <QtWidgets>
#include <QtWebEngineWidgets>
#include <QtWebChannel>
#include <bits/stdc++.h>
#include <thread>
#include <atomic>
#include <random>
#include <mutex>
#include <QtWebEngine/QtWebEngine>

using namespace std;
using ll = long long;
const ll INFINITY_VALUE = (1LL << 60);
static atomic<bool> isRunning(true);

struct Edge
{
    int destination;
    ll weight;
    bool isActive;
    string roadName;
    Edge(int dest = 0, ll w = 0, bool active = true, string name = "")
        : destination(dest), weight(w), isActive(active), roadName(name) {}
};

struct Graph
{
    vector<vector<Edge>> adjacencyList;
    unordered_map<string, int> cityNameToIndex;
    vector<string> indexToCityName;
    vector<double> latitudes;
    vector<double> longitudes;
    int totalNodes = 0;
    mutable mutex graphMutex;

    void ensureNodeExists(const string &cityName)
    {
        lock_guard<mutex> lock(graphMutex);
        if (cityNameToIndex.find(cityName) == cityNameToIndex.end())
        {
            cityNameToIndex[cityName] = totalNodes++;
            indexToCityName.push_back(cityName);
            adjacencyList.emplace_back();
            latitudes.push_back(numeric_limits<double>::quiet_NaN());
            longitudes.push_back(numeric_limits<double>::quiet_NaN());
        }
    }

    bool addCity(const string &cityName)
    {
        lock_guard<mutex> lock(graphMutex);
        if (cityNameToIndex.find(cityName) != cityNameToIndex.end())
            return false;
        cityNameToIndex[cityName] = totalNodes++;
        indexToCityName.push_back(cityName);
        adjacencyList.emplace_back();
        latitudes.push_back(numeric_limits<double>::quiet_NaN());
        longitudes.push_back(numeric_limits<double>::quiet_NaN());
        return true;
    }

    void setCityCoordinates(const string &cityName, double latitude, double longitude)
    {
        lock_guard<mutex> lock(graphMutex);
        if (cityNameToIndex.find(cityName) == cityNameToIndex.end())
        {
            cityNameToIndex[cityName] = totalNodes++;
            indexToCityName.push_back(cityName);
            adjacencyList.emplace_back();
            latitudes.push_back(latitude);
            longitudes.push_back(longitude);
        }
        else
        {
            int nodeIndex = cityNameToIndex[cityName];
            if (nodeIndex >= (int)latitudes.size())
            {
                latitudes.resize(nodeIndex + 1, numeric_limits<double>::quiet_NaN());
                longitudes.resize(nodeIndex + 1, numeric_limits<double>::quiet_NaN());
            }
            latitudes[nodeIndex] = latitude;
            longitudes[nodeIndex] = longitude;
        }
    }

    pair<double, double> getCoordinatesByIndex(int nodeIndex) const
    {
        lock_guard<mutex> lock(graphMutex);
        if (nodeIndex < 0 || nodeIndex >= totalNodes)
            return {numeric_limits<double>::quiet_NaN(), numeric_limits<double>::quiet_NaN()};
        return {latitudes[nodeIndex], longitudes[nodeIndex]};
    }

    int findNearestNodeIndex(double latitude, double longitude) const
    {
        lock_guard<mutex> lock(graphMutex);
        double bestDistance = numeric_limits<double>::infinity();
        int bestIndex = -1;
        for (int i = 0; i < totalNodes; ++i)
        {
            double nodeLat = latitudes[i];
            double nodeLon = longitudes[i];
            if (isnan(nodeLat) || isnan(nodeLon))
                continue;
            double deltaLat = nodeLat - latitude;
            double deltaLon = nodeLon - longitude;
            double distanceSquared = deltaLat * deltaLat + deltaLon * deltaLon;
            if (distanceSquared < bestDistance)
            {
                bestDistance = distanceSquared;
                bestIndex = i;
            }
        }
        return bestIndex;
    }

    void addRoad(const string &cityA, const string &cityB, ll weight, const string &roadName = "")
    {
        lock_guard<mutex> lock(graphMutex);
        if (cityNameToIndex.find(cityA) == cityNameToIndex.end())
        {
            cityNameToIndex[cityA] = totalNodes++;
            indexToCityName.push_back(cityA);
            adjacencyList.emplace_back();
            latitudes.push_back(numeric_limits<double>::quiet_NaN());
            longitudes.push_back(numeric_limits<double>::quiet_NaN());
        }
        if (cityNameToIndex.find(cityB) == cityNameToIndex.end())
        {
            cityNameToIndex[cityB] = totalNodes++;
            indexToCityName.push_back(cityB);
            adjacencyList.emplace_back();
            latitudes.push_back(numeric_limits<double>::quiet_NaN());
            longitudes.push_back(numeric_limits<double>::quiet_NaN());
        }
        int indexA = cityNameToIndex[cityA];
        int indexB = cityNameToIndex[cityB];
        adjacencyList[indexA].push_back(Edge(indexB, weight, true, roadName));
        adjacencyList[indexB].push_back(Edge(indexA, weight, true, roadName));
    }

    void showMapToString(QString &output) const
    {
        lock_guard<mutex> lock(graphMutex);
        QString stringResult;
        stringResult += QString("Cities (%1):\n").arg(totalNodes);
        for (int i = 0; i < totalNodes; ++i)
        {
            stringResult += "  - " + QString::fromStdString(indexToCityName[i]);
            auto coordinates = make_pair(latitudes[i], longitudes[i]);
            if (!isnan(coordinates.first) && !isnan(coordinates.second))
                stringResult += QString(" [lat=%1 lon=%2]").arg(coordinates.first).arg(coordinates.second);
            stringResult += "\n";
        }
        stringResult += "\nRoads:\n";
        set<pair<int, int>> shownRoads;
        for (int nodeIndex = 0; nodeIndex < totalNodes; ++nodeIndex)
        {
            for (const auto &edge : adjacencyList[nodeIndex])
            {
                int neighborIndex = edge.destination;
                if (nodeIndex < neighborIndex && shownRoads.insert({nodeIndex, neighborIndex}).second)
                {
                    stringResult += QString("  %1 <-> %2 | weight: %3 | %4")
                                        .arg(QString::fromStdString(indexToCityName[nodeIndex]),
                                             QString::fromStdString(indexToCityName[neighborIndex]),
                                             QString::number(edge.weight),
                                             edge.isActive ? "open" : "blocked");
                    if (!edge.roadName.empty())
                        stringResult += " | name: " + QString::fromStdString(edge.roadName);
                    stringResult += "\n";
                }
            }
        }
        output = stringResult;
    }

    int findEdgeIndex(int sourceNode, int destinationNode)
    {
        lock_guard<mutex> lock(graphMutex);
        for (int i = 0; i < (int)adjacencyList[sourceNode].size(); ++i)
            if (adjacencyList[sourceNode][i].destination == destinationNode)
                return i;
        return -1;
    }

    bool setRoadActive(const string &cityA, const string &cityB, bool active)
    {
        lock_guard<mutex> lock(graphMutex);
        auto iteratorA = cityNameToIndex.find(cityA);
        auto iteratorB = cityNameToIndex.find(cityB);
        if (iteratorA == cityNameToIndex.end() || iteratorB == cityNameToIndex.end())
            return false;
        int indexA = iteratorA->second;
        int indexB = iteratorB->second;
        int edgeIndexA = -1;
        int edgeIndexB = -1;
        for (int i = 0; i < (int)adjacencyList[indexA].size(); ++i)
            if (adjacencyList[indexA][i].destination == indexB)
            {
                edgeIndexA = i;
                break;
            }
        for (int i = 0; i < (int)adjacencyList[indexB].size(); ++i)
            if (adjacencyList[indexB][i].destination == indexA)
            {
                edgeIndexB = i;
                break;
            }
        if (edgeIndexA == -1 || edgeIndexB == -1)
            return false;
        adjacencyList[indexA][edgeIndexA].isActive = active;
        adjacencyList[indexB][edgeIndexB].isActive = active;
        return true;
    }

    bool changeRoadWeight(const string &cityA, const string &cityB, ll newWeight)
    {
        lock_guard<mutex> lock(graphMutex);
        auto iteratorA = cityNameToIndex.find(cityA);
        auto iteratorB = cityNameToIndex.find(cityB);
        if (iteratorA == cityNameToIndex.end() || iteratorB == cityNameToIndex.end())
            return false;
        int indexA = iteratorA->second;
        int indexB = iteratorB->second;
        int edgeIndexA = -1;
        int edgeIndexB = -1;
        for (int i = 0; i < (int)adjacencyList[indexA].size(); ++i)
            if (adjacencyList[indexA][i].destination == indexB)
            {
                edgeIndexA = i;
                break;
            }
        for (int i = 0; i < (int)adjacencyList[indexB].size(); ++i)
            if (adjacencyList[indexB][i].destination == indexA)
            {
                edgeIndexB = i;
                break;
            }
        if (edgeIndexA == -1 || edgeIndexB == -1)
            return false;
        adjacencyList[indexA][edgeIndexA].weight = newWeight;
        adjacencyList[indexB][edgeIndexB].weight = newWeight;
        return true;
    }

    pair<vector<ll>, vector<int>> dijkstraAlgorithm(const string &sourceCityName) const
    {
        lock_guard<mutex> lock(graphMutex);
        vector<ll> distances(totalNodes, INFINITY_VALUE);
        vector<int> parentNodes(totalNodes, -1);
        auto iterator = cityNameToIndex.find(sourceCityName);
        if (iterator == cityNameToIndex.end())
            return {distances, parentNodes};
        int sourceIndex = iterator->second;

        using PairLongInt = pair<ll, int>;
        priority_queue<PairLongInt, vector<PairLongInt>, greater<PairLongInt>> priorityQueue;
        distances[sourceIndex] = 0;
        priorityQueue.push({0, sourceIndex});

        while (!priorityQueue.empty())
        {
            auto [currentDistance, currentNode] = priorityQueue.top();
            priorityQueue.pop();
            if (currentDistance != distances[currentNode])
                continue;
            for (const auto &edge : adjacencyList[currentNode])
            {
                if (!edge.isActive)
                    continue;
                int neighborNode = edge.destination;
                ll newDistance = currentDistance + edge.weight;
                if (newDistance < distances[neighborNode])
                {
                    distances[neighborNode] = newDistance;
                    parentNodes[neighborNode] = currentNode;
                    priorityQueue.push({newDistance, neighborNode});
                }
            }
        }
        return {distances, parentNodes};
    }

    pair<vector<ll>, vector<int>> dijkstraFromIndex(int sourceIndex) const
    {
        lock_guard<mutex> lock(graphMutex);
        vector<ll> distances(totalNodes, INFINITY_VALUE);
        vector<int> parentNodes(totalNodes, -1);
        if (sourceIndex < 0 || sourceIndex >= totalNodes)
            return {distances, parentNodes};

        using PairLongInt = pair<ll, int>;
        priority_queue<PairLongInt, vector<PairLongInt>, greater<PairLongInt>> priorityQueue;
        distances[sourceIndex] = 0;
        priorityQueue.push({0, sourceIndex});

        while (!priorityQueue.empty())
        {
            auto [currentDistance, currentNode] = priorityQueue.top();
            priorityQueue.pop();
            if (currentDistance != distances[currentNode])
                continue;
            for (const auto &edge : adjacencyList[currentNode])
            {
                if (!edge.isActive)
                    continue;
                int neighborNode = edge.destination;
                ll newDistance = currentDistance + edge.weight;
                if (newDistance < distances[neighborNode])
                {
                    distances[neighborNode] = newDistance;
                    parentNodes[neighborNode] = currentNode;
                    priorityQueue.push({newDistance, neighborNode});
                }
            }
        }
        return {distances, parentNodes};
    }

    vector<int> reconstructPath(const vector<int> &parentNodes, int destinationNode) const
    {
        vector<int> path;
        int currentNode = destinationNode;
        while (currentNode != -1)
        {
            path.push_back(currentNode);
            currentNode = parentNodes[currentNode];
        }
        reverse(path.begin(), path.end());
        return path;
    }

    bool saveToFile(const string &filename) const
    {
        lock_guard<mutex> lock(graphMutex);
        ofstream outputFile(filename);
        if (!outputFile)
            return false;
        outputFile << totalNodes << "\n";
        for (int i = 0; i < totalNodes; ++i)
        {
            double latitude = latitudes[i];
            double longitude = longitudes[i];
            if (!isnan(latitude) && !isnan(longitude))
                outputFile << indexToCityName[i] << " " << fixed << setprecision(8) << latitude << " " << longitude << "\n";
            else
                outputFile << indexToCityName[i] << "\n";
        }
        set<pair<int, int>> savedRoads;
        for (int nodeIndex = 0; nodeIndex < totalNodes; ++nodeIndex)
        {
            for (const auto &edge : adjacencyList[nodeIndex])
            {
                int neighborIndex = edge.destination;
                if (nodeIndex < neighborIndex && savedRoads.insert({nodeIndex, neighborIndex}).second)
                {
                    outputFile << indexToCityName[nodeIndex] << " " << indexToCityName[neighborIndex] << " "
                               << edge.weight << " " << (edge.isActive ? 1 : 0) << " ";
                    string roadNameFormatted = edge.roadName;
                    for (char &character : roadNameFormatted)
                        if (character == ' ')
                            character = '_';
                    outputFile << roadNameFormatted << "\n";
                }
            }
        }
        return true;
    }

    bool loadFromFile(const string &filename)
    {
        ifstream inputFile(filename);
        if (!inputFile)
            return false;
        lock_guard<mutex> lock(graphMutex);
        cityNameToIndex.clear();
        indexToCityName.clear();
        adjacencyList.clear();
        latitudes.clear();
        longitudes.clear();
        totalNodes = 0;
        int numberOfNodes;
        inputFile >> numberOfNodes;
        string line, tempString;
        getline(inputFile, line);
        for (int i = 0; i < numberOfNodes; ++i)
        {
            getline(inputFile, tempString);
            if (!tempString.empty() && tempString.back() == '\r')
                tempString.pop_back();
            if (tempString.empty())
                continue;
            stringstream stringStream(tempString);
            string cityName;
            stringStream >> cityName;
            double latitude, longitude;
            if (stringStream >> latitude >> longitude)
            {
                ensureNodeExists(cityName);
                int nodeIndex = cityNameToIndex[cityName];
                latitudes[nodeIndex] = latitude;
                longitudes[nodeIndex] = longitude;
            }
            else
            {
                ensureNodeExists(cityName);
            }
        }
        string cityA, cityB, roadNameFormatted;
        ll weight;
        int activeInteger;
        while (inputFile >> cityA >> cityB >> weight >> activeInteger >> roadNameFormatted)
        {
            for (char &character : roadNameFormatted)
                if (character == '_')
                    character = ' ';
            addRoad(cityA, cityB, weight, roadNameFormatted);
            bool active = (activeInteger == 1);
            setRoadActive(cityA, cityB, active);
        }
        return true;
    }

    bool randomTrafficUpdate(mt19937 &randomGenerator, double minFactor, double maxFactor,
                             string &updatedCityA, string &updatedCityB, ll &newWeight)
    {
        lock_guard<mutex> lock(graphMutex);
        if (totalNodes < 1)
            return false;
        vector<pair<int, int>> candidateRoads;
        for (int nodeIndex = 0; nodeIndex < totalNodes; ++nodeIndex)
        {
            for (int edgeIndex = 0; edgeIndex < (int)adjacencyList[nodeIndex].size(); ++edgeIndex)
            {
                if (adjacencyList[nodeIndex][edgeIndex].isActive &&
                    nodeIndex < adjacencyList[nodeIndex][edgeIndex].destination)
                    candidateRoads.emplace_back(nodeIndex, edgeIndex);
            }
        }
        if (candidateRoads.empty())
            return false;
        uniform_int_distribution<size_t> indexDistribution(0, candidateRoads.size() - 1);
        size_t selectedIndex = indexDistribution(randomGenerator);
        int sourceNode = candidateRoads[selectedIndex].first;
        int edgeIndex = candidateRoads[selectedIndex].second;
        int destinationNode = adjacencyList[sourceNode][edgeIndex].destination;
        double trafficFactor;
        uniform_real_distribution<double> factorDistribution(minFactor, maxFactor);
        trafficFactor = factorDistribution(randomGenerator);
        ll oldWeight = adjacencyList[sourceNode][edgeIndex].weight;
        ll updatedWeight = max(1LL, (ll)round(oldWeight * trafficFactor));
        int reverseEdgeIndex = -1;
        for (int i = 0; i < (int)adjacencyList[destinationNode].size(); ++i)
            if (adjacencyList[destinationNode][i].destination == sourceNode)
            {
                reverseEdgeIndex = i;
                break;
            }
        if (reverseEdgeIndex == -1)
            return false;
        adjacencyList[sourceNode][edgeIndex].weight = updatedWeight;
        adjacencyList[destinationNode][reverseEdgeIndex].weight = updatedWeight;
        updatedCityA = indexToCityName[sourceNode];
        updatedCityB = indexToCityName[destinationNode];
        newWeight = updatedWeight;
        return true;
    }
};

// Bridge object exposed to JS via QWebChannel
class MapBridge : public QObject
{
    Q_OBJECT
public:
    explicit MapBridge(Graph &graph, QObject *parent = nullptr) : QObject(parent), graphReference(graph) {}

public slots:
    Q_INVOKABLE void onMapClick(double latitude, double longitude)
    {
        QString cityName = QString("City_%1_%2").arg(latitude, 0, 'f', 6).arg(longitude, 0, 'f', 6);
        string cityNameString = cityName.toStdString();
        {
            graphReference.setCityCoordinates(cityNameString, latitude, longitude);
        }
        emit cityAdded(cityName, latitude, longitude);

        if (waitingForSource)
        {
            sourceLat = latitude;
            sourceLon = longitude;
            sourceName = cityName;
            waitingForSource = false;
            emit showMessage(QString("Map: source set near %1").arg(cityName));
        }
        else
        {
            double destLat = latitude;
            double destLon = longitude;
            waitingForSource = true;

            int sourceIndex = graphReference.findNearestNodeIndex(sourceLat, sourceLon);
            int destIndex = graphReference.findNearestNodeIndex(destLat, destLon);

            if (sourceIndex == -1 || destIndex == -1)
            {
                emit routeComputed(QString("Route: could not find nearby graph nodes for the clicked points."));
                return;
            }

            auto pathResult = graphReference.dijkstraFromIndex(sourceIndex);
            auto &distances = pathResult.first;
            auto &parentNodes = pathResult.second;

            if (distances[destIndex] >= INFINITY_VALUE)
            {
                QString output = QString("No route between nearest nodes: %1 and %2")
                                     .arg(QString::fromStdString(graphReference.indexToCityName[sourceIndex]),
                                          QString::fromStdString(graphReference.indexToCityName[destIndex]));
                emit routeComputed(output);
                return;
            }

            auto path = graphReference.reconstructPath(parentNodes, destIndex);
            QStringList cityNames;
            {
                lock_guard<mutex> lock(graphReference.graphMutex);
                for (int nodeIndex : path)
                    cityNames << QString::fromStdString(graphReference.indexToCityName[nodeIndex]);
            }
            QString pathString = cityNames.join(" -> ");
            QString output = QString("Route (nearest nodes): %1").arg(pathString);
            emit routeComputed(output);
        }
    }

    Q_INVOKABLE void findShortestPath(double lat1, double lon1, double lat2, double lon2)
    {
        int sourceIndex = graphReference.findNearestNodeIndex(lat1, lon1);
        int destIndex = graphReference.findNearestNodeIndex(lat2, lon2);

        if (sourceIndex == -1 || destIndex == -1)
        {
            emit routeComputed(QString("Route: no nearby graph nodes found for one or both points."));
            return;
        }

        auto pathResult = graphReference.dijkstraFromIndex(sourceIndex);
        auto distances = pathResult.first;
        auto parentNodes = pathResult.second;

        if (destIndex < 0 || destIndex >= (int)distances.size() || distances[destIndex] >= INFINITY_VALUE)
        {
            emit routeComputed(QString("Route: no path between nearest nodes %1 and %2")
                                   .arg(QString::fromStdString(graphReference.indexToCityName[sourceIndex]),
                                        QString::fromStdString(graphReference.indexToCityName[destIndex])));
            return;
        }

        auto path = graphReference.reconstructPath(parentNodes, destIndex);
        QStringList cityNames;
        {
            lock_guard<mutex> lock(graphReference.graphMutex);
            for (int nodeIndex : path)
                cityNames << QString::fromStdString(graphReference.indexToCityName[nodeIndex]);
        }
        QString pathString = cityNames.join(" -> ");
        QString output = QString("Route between nearest nodes: %1").arg(pathString);
        emit routeComputed(output);
    }

signals:
    void cityAdded(QString name, double lat, double lon);
    void routeComputed(QString path);
    void showMessage(QString msg);

private:
    Graph &graphReference;
    bool waitingForSource = true;
    double sourceLat = 0;
    double sourceLon = 0;
    QString sourceName;
};

// GUI Main Window
class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    MainWindow(Graph &graph, QWidget *parent = nullptr) : QMainWindow(parent), graphReference(graph)
    {
        setWindowTitle("Smart Route Planner (Qt + Map)");
        resize(1200, 700);

        QWidget *centralWidget = new QWidget(this);
        setCentralWidget(centralWidget);

        QHBoxLayout *mainLayout = new QHBoxLayout(centralWidget);

        QVBoxLayout *leftLayout = new QVBoxLayout();

        QPushButton *btnAddCity = new QPushButton("Add City (text)");
        QPushButton *btnAddRoad = new QPushButton("Add Road");
        QPushButton *btnShowMap = new QPushButton("Show Text Map");
        QPushButton *btnFindRoute = new QPushButton("Find Fastest Route");
        QPushButton *btnBlockRoad = new QPushButton("Block/Unblock Road");
        QPushButton *btnChangeWeight = new QPushButton("Change Road Weight");
        QPushButton *btnSimTraffic = new QPushButton("Toggle Traffic Simulation");
        QPushButton *btnSave = new QPushButton("Save Map");
        QPushButton *btnLoad = new QPushButton("Load Map");
        QPushButton *btnWaypoints = new QPushButton("Find Route with Waypoints");

        leftLayout->addWidget(btnAddCity);
        leftLayout->addWidget(btnAddRoad);
        leftLayout->addWidget(btnShowMap);
        leftLayout->addWidget(btnFindRoute);
        leftLayout->addWidget(btnWaypoints);
        leftLayout->addWidget(btnBlockRoad);
        leftLayout->addWidget(btnChangeWeight);
        leftLayout->addWidget(btnSimTraffic);
        leftLayout->addWidget(btnSave);
        leftLayout->addWidget(btnLoad);
        leftLayout->addStretch();

        QVBoxLayout *rightLayout = new QVBoxLayout();
        mapTextWidget = new QTextEdit();
        mapTextWidget->setReadOnly(true);
        mapTextWidget->setMinimumHeight(140);

        mapViewWidget = new QWebEngineView();

        logTextWidget = new QTextEdit();
        logTextWidget->setReadOnly(true);
        logTextWidget->setMinimumHeight(180);

        rightLayout->addWidget(new QLabel("Interactive Map: (click to add city)"));
        rightLayout->addWidget(mapViewWidget, 1);
        rightLayout->addWidget(new QLabel("Map / Output (text):"));
        rightLayout->addWidget(mapTextWidget, 0);
        rightLayout->addWidget(new QLabel("Event Log:"));
        rightLayout->addWidget(logTextWidget, 0);

        mainLayout->addLayout(leftLayout, 0);
        mainLayout->addLayout(rightLayout, 1);

        connect(btnAddCity, &QPushButton::clicked, this, &MainWindow::onAddCity);
        connect(btnAddRoad, &QPushButton::clicked, this, &MainWindow::onAddRoad);
        connect(btnShowMap, &QPushButton::clicked, this, &MainWindow::onShowMap);
        connect(btnFindRoute, &QPushButton::clicked, this, &MainWindow::onFindRoute);
        connect(btnBlockRoad, &QPushButton::clicked, this, &MainWindow::onBlockUnblock);
        connect(btnChangeWeight, &QPushButton::clicked, this, &MainWindow::onChangeWeight);
        connect(btnSimTraffic, &QPushButton::clicked, this, &MainWindow::onToggleSim);
        connect(btnSave, &QPushButton::clicked, this, &MainWindow::onSave);
        connect(btnLoad, &QPushButton::clicked, this, &MainWindow::onLoad);
        connect(btnWaypoints, &QPushButton::clicked, this, &MainWindow::onWaypoints);

        // Add initial roads
        graphReference.addRoad("Delhi", "Jaipur", 280, "NH48");
        graphReference.addRoad("Delhi", "Chandigarh", 250, "NH44");
        graphReference.addRoad("Delhi", "Lucknow", 530, "NH27");
        graphReference.addRoad("Delhi", "Dehradun", 250, "NH334");
        graphReference.addRoad("Delhi", "Bhopal", 780, "NH46");
        graphReference.addRoad("Delhi", "Patna", 1080, "NH19");
        graphReference.addRoad("Delhi", "Raipur", 1220, "NH44");
        graphReference.addRoad("Jaipur", "Bhopal", 520, "NH52");
        graphReference.addRoad("Jaipur", "Gandhinagar", 640, "NH48");
        graphReference.addRoad("Jaipur", "Lucknow", 580, "NH27");
        graphReference.addRoad("Gandhinagar", "Mumbai", 520, "NH48");
        graphReference.addRoad("Gandhinagar", "Bhopal", 740, "NH47");
        graphReference.addRoad("Mumbai", "Panaji", 590, "NH66");
        graphReference.addRoad("Mumbai", "Hyderabad", 710, "NH65");
        graphReference.addRoad("Mumbai", "Bhopal", 780, "NH160");
        graphReference.addRoad("Panaji", "Bengaluru", 560, "NH48");
        graphReference.addRoad("Panaji", "Hyderabad", 660, "NH65");
        graphReference.addRoad("Hyderabad", "Bengaluru", 570, "NH44");
        graphReference.addRoad("Hyderabad", "Amaravati", 350, "NH65");
        graphReference.addRoad("Hyderabad", "Raipur", 610, "NH30");
        graphReference.addRoad("Hyderabad", "Bhopal", 780, "NH44");
        graphReference.addRoad("Bhopal", "Raipur", 590, "NH47");
        graphReference.addRoad("Bhopal", "Lucknow", 580, "NH30");
        graphReference.addRoad("Bhopal", "Patna", 960, "NH31");
        graphReference.addRoad("Lucknow", "Patna", 530, "NH27");
        graphReference.addRoad("Lucknow", "Dehradun", 530, "NH734");
        graphReference.addRoad("Lucknow", "Chandigarh", 720, "NH44");
        graphReference.addRoad("Lucknow", "Ranchi", 650, "NH19");
        graphReference.addRoad("Raipur", "Ranchi", 620, "NH43");
        graphReference.addRoad("Raipur", "Bhubaneswar", 550, "NH53");
        graphReference.addRoad("Raipur", "Nagpur", 290, "NH53");
        graphReference.addRoad("Ranchi", "Patna", 330, "NH33");
        graphReference.addRoad("Ranchi", "Bhubaneswar", 460, "NH16");
        graphReference.addRoad("Bhubaneswar", "Kolkata", 440, "NH16");
        graphReference.addRoad("Bhubaneswar", "Amaravati", 950, "NH16");
        graphReference.addRoad("Kolkata", "Patna", 590, "NH19");
        graphReference.addRoad("Kolkata", "Agartala", 1360, "NH8");
        graphReference.addRoad("Kolkata", "Aizawl", 1270, "NH306");
        graphReference.addRoad("Kolkata", "Gangtok", 690, "NH10");
        graphReference.addRoad("Kolkata", "Imphal", 1380, "NH37");
        graphReference.addRoad("Kolkata", "Dispur", 970, "NH27");
        graphReference.addRoad("Dispur", "Imphal", 480, "NH102");
        graphReference.addRoad("Dispur", "Agartala", 550, "NH8");
        graphReference.addRoad("Dispur", "Shillong", 100, "NH6");
        graphReference.addRoad("Shillong", "Imphal", 360, "NH37");
        graphReference.addRoad("Shillong", "Aizawl", 350, "NH306");
        graphReference.addRoad("Imphal", "Agartala", 520, "NH8");
        graphReference.addRoad("Aizawl", "Agartala", 330, "NH108");
        graphReference.addRoad("Gangtok", "Dispur", 890, "NH27");
        graphReference.addRoad("Gangtok", "Patna", 670, "NH27");
        graphReference.addRoad("Gangtok", "Ranchi", 780, "NH27");
        graphReference.addRoad("Bengaluru", "Chennai", 340, "NH48");
        graphReference.addRoad("Bengaluru", "Thiruvananthapuram", 730, "NH44");
        graphReference.addRoad("Bengaluru", "Puducherry", 310, "NH77");
        graphReference.addRoad("Bengaluru", "Hyderabad", 570, "NH44");
        graphReference.addRoad("Chennai", "Puducherry", 170, "NH32");
        graphReference.addRoad("Chennai", "Amaravati", 640, "NH16");
        graphReference.addRoad("Chennai", "Thiruvananthapuram", 760, "NH44");
        graphReference.addRoad("Amaravati", "Bhubaneswar", 950, "NH16");
        graphReference.addRoad("Amaravati", "Bengaluru", 660, "NH16");
        graphReference.addRoad("Amaravati", "Raipur", 780, "NH30");
        graphReference.addRoad("Thiruvananthapuram", "Kavaratti", 450, "NH47");
        graphReference.addRoad("Thiruvananthapuram", "Puducherry", 560, "NH66");
        graphReference.addRoad("Dehradun", "Chandigarh", 170, "NH7");
        graphReference.addRoad("Dehradun", "Shimla", 230, "NH7");
        graphReference.addRoad("Dehradun", "Srinagar", 710, "NH7");
        graphReference.addRoad("Chandigarh", "Shimla", 115, "NH5");
        graphReference.addRoad("Chandigarh", "Srinagar", 650, "NH44");
        graphReference.addRoad("Shimla", "Srinagar", 700, "NH44");
        graphReference.addRoad("Srinagar", "Leh", 420, "NH1");
        graphReference.addRoad("Leh", "Chandigarh", 760, "NH3");
        graphReference.addRoad("Leh", "Dehradun", 790, "NH7");
        graphReference.addRoad("Bhopal", "Gandhinagar", 740, "NH47");
        graphReference.addRoad("Jaipur", "Raipur", 870, "NH27");
        graphReference.addRoad("Patna", "Dispur", 980, "NH27");
        graphReference.addRoad("Patna", "Ranchi", 330, "NH33");
        graphReference.addRoad("Ranchi", "Raipur", 620, "NH43");
        graphReference.addRoad("Raipur", "Hyderabad", 610, "NH30");
        graphReference.addRoad("Raipur", "Nagpur", 290, "NH53");
        graphReference.addRoad("Nagpur", "Bhopal", 360, "NH47");
        graphReference.addRoad("Nagpur", "Hyderabad", 500, "NH44");
        graphReference.addRoad("Nagpur", "Raipur", 290, "NH53");
        graphReference.addRoad("Nagpur", "Mumbai", 830, "NH53");
        graphReference.addRoad("Nagpur", "Raipur", 300, "NH53");
        graphReference.addRoad("Nagpur", "Amaravati", 370, "NH53");
        graphReference.addRoad("Gandhinagar", "Jaipur", 640, "NH48");
        graphReference.addRoad("Bhopal", "Nagpur", 360, "NH47");
        graphReference.addRoad("Kolkata", "Raipur", 830, "NH53");
        graphReference.addRoad("Lucknow", "Raipur", 870, "NH30");
        graphReference.addRoad("Ranchi", "Raipur", 620, "NH43");
        graphReference.addRoad("Bhopal", "Mumbai", 780, "NH160");
        graphReference.addRoad("Delhi", "Nagpur", 980, "NH44");
        graphReference.addRoad("Delhi", "Mumbai", 1450, "NH48");
        graphReference.addRoad("Delhi", "Chennai", 2200, "NH44");
        graphReference.addRoad("Delhi", "Kolkata", 1500, "NH19");
        graphReference.addRoad("Delhi", "Hyderabad", 1550, "NH44");
        graphReference.addRoad("Delhi", "Bengaluru", 2100, "NH44");

        addStateCapitals();
        setupMapIntegration();

        isRunning = true;
        simulationThread = std::thread(&MainWindow::simulateTrafficThread, this);

        onShowMap();
    }

    ~MainWindow()
    {
        isRunning = false;
        if (simulationThread.joinable())
            simulationThread.join();
    }

private slots:
    void onAddCity()
    {
        bool dialogOk;
        QString cityName = QInputDialog::getText(this, "Add City", "City name:", QLineEdit::Normal, "", &dialogOk);
        if (!dialogOk || cityName.trimmed().isEmpty())
            return;
        string cityNameString = cityName.toStdString();
        if (graphReference.addCity(cityNameString))
            appendLog(QString("City added: %1").arg(cityName));
        else
            appendLog(QString("City '%1' already exists.").arg(cityName));
        onShowMap();
    }

    void onAddRoad()
    {
        QDialog dialog;
        QFormLayout formLayout(&dialog);
        QLineEdit *lineEditA = new QLineEdit(&dialog);
        QLineEdit *lineEditB = new QLineEdit(&dialog);
        QLineEdit *lineEditWeight = new QLineEdit(&dialog);
        QLineEdit *lineEditRoadName = new QLineEdit(&dialog);
        formLayout.addRow("City A:", lineEditA);
        formLayout.addRow("City B:", lineEditB);
        formLayout.addRow("Weight (int):", lineEditWeight);
        formLayout.addRow("Road name (optional):", lineEditRoadName);
        QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
        formLayout.addRow(&buttonBox);
        connect(&buttonBox, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
        connect(&buttonBox, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);

        dialog.setWindowModality(Qt::ApplicationModal);
        dialog.show();
        dialog.exec();
        if (lineEditA->text().isEmpty() || lineEditB->text().isEmpty() || lineEditA->text() == lineEditB->text())
        {
            appendLog("Invalid cities for road.");
            return;
        }
        bool dialogOk;
        ll weight = lineEditWeight->text().toLongLong(&dialogOk);
        if (!dialogOk || weight <= 0)
        {
            appendLog("Invalid weight.");
            return;
        }
        graphReference.addRoad(lineEditA->text().toStdString(), lineEditB->text().toStdString(),
                               weight, lineEditRoadName->text().toStdString());
        appendLog(QString("Road added between %1 and %2 weight=%3")
                      .arg(lineEditA->text(), lineEditB->text(), QString::number(weight)));
        onShowMap();
    }

    void onShowMap()
    {
        QString mapString;
        graphReference.showMapToString(mapString);
        mapTextWidget->setPlainText(mapString);
    }

    void onFindRoute()
    {
        bool dialogOk;
        QString sourceCity = QInputDialog::getText(this, "Fastest Route", "Source city:",
                                                   QLineEdit::Normal, "", &dialogOk);
        if (!dialogOk || sourceCity.trimmed().isEmpty())
            return;
        QString destinationCity = QInputDialog::getText(this, "Fastest Route", "Destination city:",
                                                        QLineEdit::Normal, "", &dialogOk);
        if (!dialogOk || destinationCity.trimmed().isEmpty())
            return;
        if (graphReference.cityNameToIndex.find(sourceCity.toStdString()) == graphReference.cityNameToIndex.end())
        {
            appendLog(QString("Unknown city: %1").arg(sourceCity));
            return;
        }
        if (graphReference.cityNameToIndex.find(destinationCity.toStdString()) == graphReference.cityNameToIndex.end())
        {
            appendLog(QString("Unknown city: %1").arg(destinationCity));
            return;
        }
        auto [distances, parentNodes] = graphReference.dijkstraAlgorithm(sourceCity.toStdString());
        int destinationIndex = graphReference.cityNameToIndex.at(destinationCity.toStdString());
        if (distances[destinationIndex] >= INFINITY_VALUE)
        {
            appendLog(QString("No route from %1 to %2 (might be blocked).").arg(sourceCity, destinationCity));
            return;
        }
        auto path = graphReference.reconstructPath(parentNodes, destinationIndex);
        QString pathString;
        for (size_t i = 0; i < path.size(); ++i)
        {
            pathString += QString::fromStdString(graphReference.indexToCityName[path[i]]);
            if (i + 1 < path.size())
                pathString += " -> ";
        }
        appendLog(QString("Shortest travel time from %1 to %2 = %3")
                      .arg(sourceCity, destinationCity, QString::number(distances[destinationIndex])));
        appendLog("Path: " + pathString);

        QString javascriptCode = buildRouteJS(path);
        if (!javascriptCode.isEmpty())
            mapViewWidget->page()->runJavaScript(javascriptCode);
        else
            appendLog("Route could not be drawn on map (missing coordinates for some cities).");
    }

    void onBlockUnblock()
    {
        bool dialogOk;
        QString cityA = QInputDialog::getText(this, "Block/Unblock Road", "City A:",
                                              QLineEdit::Normal, "", &dialogOk);
        if (!dialogOk || cityA.trimmed().isEmpty())
            return;
        QString cityB = QInputDialog::getText(this, "Block/Unblock Road", "City B:",
                                              QLineEdit::Normal, "", &dialogOk);
        if (!dialogOk || cityB.trimmed().isEmpty())
            return;
        bool roadFound = false;
        bool roadIsActive = false;
        {
            lock_guard<mutex> lock(graphReference.graphMutex);
            auto iteratorA = graphReference.cityNameToIndex.find(cityA.toStdString());
            auto iteratorB = graphReference.cityNameToIndex.find(cityB.toStdString());
            if (iteratorA == graphReference.cityNameToIndex.end() ||
                iteratorB == graphReference.cityNameToIndex.end())
            {
                appendLog("Unknown city.");
                return;
            }
            int indexA = iteratorA->second;
            int indexB = iteratorB->second;
            for (auto &edge : graphReference.adjacencyList[indexA])
            {
                if (edge.destination == indexB)
                {
                    roadFound = true;
                    roadIsActive = edge.isActive;
                    break;
                }
            }
        }
        if (!roadFound)
        {
            appendLog("Road not found.");
            return;
        }
        bool operationOk = graphReference.setRoadActive(cityA.toStdString(), cityB.toStdString(), !roadIsActive);
        if (operationOk)
            appendLog(QString("%1 road between %2 and %3")
                          .arg(!roadIsActive ? "Unblocked" : "Blocked", cityA, cityB));
        else
            appendLog("Failed to change road state.");
        onShowMap();
    }

    void onChangeWeight()
    {
        bool dialogOk;
        QString cityA = QInputDialog::getText(this, "Change Weight", "City A:",
                                              QLineEdit::Normal, "", &dialogOk);
        if (!dialogOk || cityA.trimmed().isEmpty())
            return;
        QString cityB = QInputDialog::getText(this, "Change Weight", "City B:",
                                              QLineEdit::Normal, "", &dialogOk);
        if (!dialogOk || cityB.trimmed().isEmpty())
            return;
        QStringList weightModes = {"Set absolute weight", "Multiply by factor", "Add extra time"};
        QString selectedMode = QInputDialog::getItem(this, "Change Weight", "Mode:",
                                                     weightModes, 0, false, &dialogOk);
        if (!dialogOk)
            return;
        if (selectedMode == weightModes[0])
        {
            ll newWeight = QInputDialog::getInt(this, "New weight", "Enter new weight:",
                                                1, 1, 1000000, 1, &dialogOk);
            if (!dialogOk)
                return;
            if (graphReference.changeRoadWeight(cityA.toStdString(), cityB.toStdString(), newWeight))
                appendLog(QString("Road weight updated to %1").arg(newWeight));
            else
                appendLog("Failed to update weight (road not found)");
        }
        else if (selectedMode == weightModes[1])
        {
            double multiplierFactor = QInputDialog::getDouble(this, "Multiply", "Enter multiplier:",
                                                              1.0, 0.01, 100.0, 2, &dialogOk);
            if (!dialogOk)
                return;
            if (graphReference.cityNameToIndex.find(cityA.toStdString()) == graphReference.cityNameToIndex.end() ||
                graphReference.cityNameToIndex.find(cityB.toStdString()) == graphReference.cityNameToIndex.end())
            {
                appendLog("Unknown city.");
                return;
            }
            int indexA = graphReference.cityNameToIndex.at(cityA.toStdString());
            int indexB = graphReference.cityNameToIndex.at(cityB.toStdString());
            int edgeIndexA = graphReference.findEdgeIndex(indexA, indexB);
            if (edgeIndexA == -1)
            {
                appendLog("Road not found.");
                return;
            }
            ll currentWeight;
            {
                lock_guard<mutex> lock(graphReference.graphMutex);
                currentWeight = graphReference.adjacencyList[indexA][edgeIndexA].weight;
            }
            ll newWeight = max(1LL, (ll)round(currentWeight * multiplierFactor));
            if (graphReference.changeRoadWeight(cityA.toStdString(), cityB.toStdString(), newWeight))
                appendLog(QString("Road weight multiplied. New weight = %1").arg(newWeight));
        }
        else
        {
            ll extraTime = QInputDialog::getInt(this, "Add extra", "Enter extra time to add:",
                                                1, 0, 1000000, 1, &dialogOk);
            if (!dialogOk)
                return;
            if (graphReference.cityNameToIndex.find(cityA.toStdString()) == graphReference.cityNameToIndex.end() ||
                graphReference.cityNameToIndex.find(cityB.toStdString()) == graphReference.cityNameToIndex.end())
            {
                appendLog("Unknown city.");
                return;
            }
            int indexA = graphReference.cityNameToIndex.at(cityA.toStdString());
            int indexB = graphReference.cityNameToIndex.at(cityB.toStdString());
            int edgeIndexA = graphReference.findEdgeIndex(indexA, indexB);
            if (edgeIndexA == -1)
            {
                appendLog("Road not found.");
                return;
            }
            ll currentWeight;
            {
                lock_guard<mutex> lock(graphReference.graphMutex);
                currentWeight = graphReference.adjacencyList[indexA][edgeIndexA].weight;
            }
            ll newWeight = currentWeight + extraTime;
            if (graphReference.changeRoadWeight(cityA.toStdString(), cityB.toStdString(), newWeight))
                appendLog(QString("Road weight increased. New weight = %1").arg(newWeight));
        }
        onShowMap();
    }

    void onToggleSim()
    {
        simulationEnabled = !simulationEnabled;
        appendLog(QString("Traffic simulation %1").arg(simulationEnabled ? "enabled" : "disabled"));
        if (simulationEnabled)
            isRunning = true;
        else
            isRunning = false;
    }

    void onSave()
    {
        QString fileName = QFileDialog::getSaveFileName(this, "Save Map", QString(),
                                                        "Map files (*.map);;All files (*.*)");
        if (fileName.isEmpty())
            return;
        if (graphReference.saveToFile(fileName.toStdString()))
            appendLog(QString("Map saved to %1").arg(fileName));
        else
            appendLog("Failed to save to file.");
    }

    void onLoad()
    {
        QString fileName = QFileDialog::getOpenFileName(this, "Load Map", QString(),
                                                        "Map files (*.map);;All files (*.*)");
        if (fileName.isEmpty())
            return;
        if (graphReference.loadFromFile(fileName.toStdString()))
        {
            appendLog(QString("Map loaded from %1").arg(fileName));
            onShowMap();
            addAllMarkersToMap();
        }
        else
            appendLog("Failed to load from file.");
    }

    void onWaypoints()
    {
        bool dialogOk;
        int waypointCount = QInputDialog::getInt(this, "Waypoints", "Number of cities in route (>=2):",
                                                 2, 2, 100, 1, &dialogOk);
        if (!dialogOk)
            return;
        vector<string> routeCities(waypointCount);
        for (int i = 0; i < waypointCount; ++i)
        {
            QString cityName = QInputDialog::getText(this, "Waypoints",
                                                     QString("City %1:").arg(i + 1),
                                                     QLineEdit::Normal, "", &dialogOk);
            if (!dialogOk || cityName.trimmed().isEmpty())
                return;
            routeCities[i] = cityName.toStdString();
        }
        bool allCitiesValid = true;
        for (auto &cityName : routeCities)
        {
            if (graphReference.cityNameToIndex.find(cityName) == graphReference.cityNameToIndex.end())
            {
                appendLog(QString::fromStdString("Unknown city: " + cityName));
                allCitiesValid = false;
            }
        }
        if (!allCitiesValid)
            return;
        ll totalDistance = 0;
        vector<int> completePathIndices;
        bool allSegmentsOk = true;
        for (int i = 0; i < waypointCount - 1; ++i)
        {
            auto [distances, parentNodes] = graphReference.dijkstraAlgorithm(routeCities[i]);
            int destinationIndex = graphReference.cityNameToIndex.at(routeCities[i + 1]);
            if (distances[destinationIndex] >= INFINITY_VALUE)
            {
                appendLog(QString::fromStdString("No route between " + routeCities[i] +
                                                 " and " + routeCities[i + 1]));
                allSegmentsOk = false;
                break;
            }
            totalDistance += distances[destinationIndex];
            auto segmentPath = graphReference.reconstructPath(parentNodes, destinationIndex);
            if (i > 0)
                segmentPath.erase(segmentPath.begin());
            completePathIndices.insert(completePathIndices.end(), segmentPath.begin(), segmentPath.end());
        }
        if (allSegmentsOk)
        {
            QString pathString;
            for (size_t i = 0; i < completePathIndices.size(); ++i)
            {
                pathString += QString::fromStdString(graphReference.indexToCityName[completePathIndices[i]]);
                if (i + 1 < completePathIndices.size())
                    pathString += " -> ";
            }
            appendLog(QString("Total travel time: %1").arg(QString::number(totalDistance)));
            appendLog("Path: " + pathString);
            QString javascriptCode = buildRouteJS(completePathIndices);
            if (!javascriptCode.isEmpty())
                mapViewWidget->page()->runJavaScript(javascriptCode);
            else
                appendLog("Route could not be drawn on map (missing coordinates).");
        }
    }

    void onMapCityAdded(QString cityName, double latitude, double longitude)
    {
        appendLog(QString("City added from map: %1 (lat=%2 lon=%3)")
                      .arg(cityName)
                      .arg(latitude)
                      .arg(longitude));
        onShowMap();
    }

    void onRouteComputed(QString routeString)
    {
        appendLog(routeString);
        mapTextWidget->append(routeString);

        QString escapedString = routeString;
        escapedString.replace("\\", "\\\\");
        escapedString.replace("\"", "\\\"");
        escapedString.replace("\n", "\\n");
        QString javascriptCode = QString("if (window.showPath) showPath(\"%1\");").arg(escapedString);
        if (mapViewWidget && mapViewWidget->page())
            mapViewWidget->page()->runJavaScript(javascriptCode);
    }

    void onBridgeMessage(QString message)
    {
        appendLog(message);
    }

private:
    Graph &graphReference;
    QTextEdit *mapTextWidget;
    QTextEdit *logTextWidget;
    QWebEngineView *mapViewWidget;
    std::thread simulationThread;
    bool simulationEnabled = true;

    MapBridge *bridgeObject = nullptr;
    QWebChannel *webChannelObject = nullptr;

    void appendLog(const QString &logString)
    {
        QString timeString = QDateTime::currentDateTime().toString(Qt::ISODate);
        logTextWidget->append(QString("[%1] %2").arg(timeString, logString));
        cout << logString.toStdString() << endl;
    }

    void addStateCapitals()
    {
        vector<pair<string, pair<double, double>>> stateCapitals = {
            {"Amaravati", {16.5417, 80.5150}},
            {"Itanagar", {27.0844, 93.6053}},
            {"Dispur", {26.1433, 91.7898}},
            {"Patna", {25.5941, 85.1376}},
            {"Raipur", {21.2514, 81.6296}},
            {"Panaji", {15.4909, 73.8278}},
            {"Gandhinagar", {23.2156, 72.6369}},
            {"Chandigarh", {30.7333, 76.7794}},
            {"Shimla", {31.1048, 77.1734}},
            {"Ranchi", {23.3441, 85.3096}},
            {"Bengaluru", {12.9716, 77.5946}},
            {"Thiruvananthapuram", {8.5241, 76.9366}},
            {"Bhopal", {23.2599, 77.4126}},
            {"Mumbai", {19.0760, 72.8777}},
            {"Imphal", {24.8170, 93.9368}},
            {"Shillong", {25.5788, 91.8933}},
            {"Aizawl", {23.7271, 92.7176}},
            {"Kohima", {25.6740, 94.1106}},
            {"Bhubaneswar", {20.2961, 85.8245}},
            {"Jaipur", {26.9124, 75.7873}},
            {"Gangtok", {27.3389, 88.6065}},
            {"Chennai", {13.0827, 80.2707}},
            {"Hyderabad", {17.3850, 78.4867}},
            {"Agartala", {23.8315, 91.2868}},
            {"Lucknow", {26.8467, 80.9462}},
            {"Dehradun", {30.3165, 78.0322}},
            {"Kolkata", {22.5726, 88.3639}}};

        for (auto &capitalData : stateCapitals)
        {
            graphReference.setCityCoordinates(capitalData.first, capitalData.second.first,
                                              capitalData.second.second);
        }
        appendLog("All Indian state capitals added as isolated nodes (coords set).");
    }

    void setupMapIntegration()
    {
        bridgeObject = new MapBridge(graphReference, this);
        webChannelObject = new QWebChannel(this);
        webChannelObject->registerObject(QStringLiteral("bridge"), bridgeObject);
        connect(bridgeObject, &MapBridge::cityAdded, this, &MainWindow::onMapCityAdded);
        connect(bridgeObject, &MapBridge::routeComputed, this, &MainWindow::onRouteComputed);
        connect(bridgeObject, &MapBridge::showMessage, this, &MainWindow::onBridgeMessage);

        QString executablePath = QCoreApplication::applicationDirPath();
        QString mapFilePath = QDir(executablePath).filePath("map.html");
        QFileInfo fileInfo(mapFilePath);
        if (!fileInfo.exists())
        {
            mapFilePath = QDir::current().filePath("map.html");
            fileInfo.setFile(mapFilePath);
        }
        if (!fileInfo.exists())
        {
            appendLog("map.html not found next to executable. Please place map.html beside the exe. Loading openstreetmap.org as fallback.");
            mapViewWidget->setUrl(QUrl("https://www.openstreetmap.org"));
            return;
        }

        mapViewWidget->page()->setWebChannel(webChannelObject);
        mapViewWidget->setUrl(QUrl::fromLocalFile(fileInfo.absoluteFilePath()));

        connect(mapViewWidget, &QWebEngineView::loadFinished, this, [this](bool loadSuccess)
                {
            if (loadSuccess)
            {
                addAllMarkersToMap();
            } });
    }

    void addAllMarkersToMap()
    {
        QString javascriptCode;
        {
            lock_guard<mutex> lock(graphReference.graphMutex);
            for (int i = 0; i < graphReference.totalNodes; ++i)
            {
                double latitude = graphReference.latitudes[i];
                double longitude = graphReference.longitudes[i];
                if (!isnan(latitude) && !isnan(longitude))
                {
                    QString cityName = QString::fromStdString(graphReference.indexToCityName[i]).replace("'", "\\'");
                    javascriptCode += QString("addMarker('%1', %2, %3);\n").arg(cityName).arg(latitude).arg(longitude);
                }
            }
        }
        if (!javascriptCode.isEmpty())
            mapViewWidget->page()->runJavaScript(javascriptCode);
    }

    QString buildRouteJS(const vector<int> &pathIndices)
    {
        QString coordinatesArray;
        bool pathValid = true;
        {
            lock_guard<mutex> lock(graphReference.graphMutex);
            for (int nodeIndex : pathIndices)
            {
                if (nodeIndex < 0 || nodeIndex >= graphReference.totalNodes)
                {
                    pathValid = false;
                    break;
                }
                double latitude = graphReference.latitudes[nodeIndex];
                double longitude = graphReference.longitudes[nodeIndex];
                if (isnan(latitude) || isnan(longitude))
                {
                    pathValid = false;
                    break;
                }
                coordinatesArray += QString("[%1,%2],").arg(latitude).arg(longitude);
            }
        }
        if (!pathValid || coordinatesArray.isEmpty())
            return QString();
        if (coordinatesArray.endsWith(','))
            coordinatesArray.chop(1);
        QString javascriptCode = QString("drawRoute([%1]);").arg(coordinatesArray);
        return javascriptCode;
    }

    void simulateTrafficThread()
    {
        mt19937 randomGenerator((unsigned)chrono::steady_clock::now().time_since_epoch().count());
        const double minimumFactor = 0.7;
        const double maximumFactor = 1.4;
        const int sleepDurationSeconds = 5;
        while (true)
        {
            if (!isRunning)
            {
                std::this_thread::sleep_for(chrono::milliseconds(200));
                continue;
            }
            std::this_thread::sleep_for(chrono::seconds(sleepDurationSeconds));
            if (!isRunning)
                continue;
            string updatedCityA, updatedCityB;
            ll newWeight = -1;
            bool updateSuccessful = graphReference.randomTrafficUpdate(randomGenerator, minimumFactor,
                                                                       maximumFactor, updatedCityA,
                                                                       updatedCityB, newWeight);
            if (updateSuccessful)
            {
                QString logMessage = QString::fromStdString("[Traffic Update] " + updatedCityA +
                                                            " <-> " + updatedCityB +
                                                            " now has weight " + to_string(newWeight));
                QMetaObject::invokeMethod(this, "appendLog", Qt::QueuedConnection, Q_ARG(QString, logMessage));
                QMetaObject::invokeMethod(this, "onShowMap", Qt::QueuedConnection);
            }
        }
    }
};

int main(int argc, char **argv)
{
    QApplication application(argc, argv);

    QtWebEngine::initialize();

    Graph roadGraph;
    MainWindow mainWindow(roadGraph);
    mainWindow.show();

    int returnCode = application.exec();

    isRunning = false;
    return returnCode;
}

#include "smart_route_planner_qt.moc"
