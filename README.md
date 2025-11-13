# 🧭 Smart Route Planner
A DSA-focused route planning system built using C++ (Qt Framework) that visualizes and computes the shortest path between Indian cities using Dijkstra’s Algorithm. The project demonstrates practical use of graphs and adjacency lists in a real-world scenario — integrated with an interactive map (Leaflet.js) for user input and visualization.

## 🚀 Features

### 🏙️ City & Road Management
- Add, update, and manage cities and roads with ease  
- Assigns unique coordinates to each city to identify them uiquely
- Supports bi-directional roads with:
  - Custom travel times  
  - Road names (e.g., “NH48”)  
  - Open/Closed (traffic-blocked) status  

### 🧭 Smart Pathfinding
- Implements **Dijkstra’s Algorithm** to find the **shortest path** between:
  - Any two cities  
  - Two GPS coordinates  
  - Multiple waypoints (multi-stop routes)

### 🌐 GPS Integration
- Each city can have real-world latitude and longitude
- Supports finding the nearest city to a given GPS coordinate  

### ⚙️ Real-Time Traffic Simulation
- Background thread simulates dynamic traffic changes every few seconds  
- Randomly increases or decreases road travel times  
- Prints live updates

### 💾 File Persistence
- Save and load the complete map from file
- we have added it to simplify the addition and removal of roads 

### 🧠 Thread-Safe Architecture
- Uses **mutex locks** for safe concurrent access  
- Separate thread handles **traffic simulation** independently  

## IMPORTANT NOTE 

 - All the GUI functions and methods are implemented after utilizing various online resources because,
 we didnt have much knowledge about funtioninga and syntaxes of QT
 - Most the logics ,functions and Data Structures used in backend_code.cpp, we tried to implement it
ourselves,of course we have taken help of online resources but that too is for optimization and smplification 
- There were one or two advanced level things (such as threading and autotraffic change simulator) but we first learned about it then implemented it.

## File Structure

Smart-Route-Planner/
│
├── src/
│   └── smart_route_planner_qt.cpp      # Main source code file integrated with Qt GUI
│
├── web/
│   └── map.html                        # Displays an interactive map using Leaflet.js and OpenStreetMap
│                                       # Allows users to directly select cities from the map interface
│
├── project/
│   └── smart_route_planner_qt.pro      # Qt project file containing library and build configurations
│
├── backend/
│   └── backend_code.cpp                # (Important) Contains all core DSA logic and backend functions
│                                       # Handles route calculations, graph algorithms, and data management
│
├── README.md                           # Project documentation and overview
│
└── LICENSE                             # Open-source license (optional but recommended)


## How to run

### Requirements- Qt for GUI (Linux)

- Make the folder and save all the files in the same waywidgets webenginewidgets webengine webchannel given above under **File Structure**.
- Now Downlaod all the required QT Libraries (widgets webenginewidgets webengine webchannel)
- then open the project folder in the terminal
- after that run qmake -> make
- a execulable file will be created in the same project folder with name smart_route_planner_qt
- run ./smart_route_planner_qt

 ## Core DSA Concepts used
 
 - 🕸️ Graph Representation (Adjacency List) -> for graph creation
 - ⚡ Dijkstra’s Algorithm ->for finding the shortest path between cities
 - ⏳ Priority Queue (Min-Heap)
 - 📦 Hash Maps (Unordered Map) -> to map cities to their alloted number
 - 🗃️ Arrays & Vectors
 - 🧩 Path Reconstruction (Backtracking) -> update the path frequently after change in the traffic weight
 - 🔐 Thread Safety with Mutex and Atomic Variables
 - 🔄 Randomization & Simulation -> to update the traffic weights randomly
 - 💾 File I/O -> save and load map directly from a file

