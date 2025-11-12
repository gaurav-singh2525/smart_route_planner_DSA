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


## FILE STRUCTURE

Smart-Route-Planner/
│
├── 
│   └── smart_route_planner_qt.cpp     # main source code file integrated with GUI
│
├──                      
│   └── map.html                       # this is used to display map using leafflet.js and openstreetmap which allows user to directly select cities 
│                                         from the map
├── 
│   └── smart_route_planner_qt.pro     # this contains the required QT libraries for running the GUI
│
├──
│   └── backend_code.cpp               # (IMP) this file contains all the main DSA loigcs and functions that work in backend (without GUI)
│
├── README.md                          # project description


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
