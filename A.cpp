#include <iostream>
#include <vector>
#include <queue>
#include <set>
#include <cmath>
#include <chrono>
#include <iomanip>
#include <map>
#include <random>
#include <functional>
#include <algorithm>

using namespace std;

// Grid size and obstacle ratio
const int GRID_WIDTH = 30;
const int GRID_HEIGHT = 30;
const double OBSTACLE_RATIO = 0.25;

// Movement cost definitions
const double COST_STRAIGHT = 1.0;
const double COST_DIAGONAL = 1.414;

// Cell state constants
const int CELL_EMPTY = 0;
const int CELL_OBSTACLE = 1;

// Structure for grid coordinates
struct Point {
    int x, y;
    bool operator==(const Point& other) const { return x == other.x && y == other.y; }
    bool operator<(const Point& other) const { return x < other.x || (x == other.x && y < other.y); }
};

// Node data used in A* algorithm
struct Node {
    Point position;
    double gCost;  // cost from start
    double hCost;  // heuristic cost to goal
    double fCost;  // f = g + h
    Point parent;

    Node(Point p, double g, double h, Point parentPoint)
        : position(p), gCost(g), hCost(h), fCost(g + h), parent(parentPoint) {}

    bool operator>(const Node& other) const { return fCost > other.fCost; }
};

//  Heuristic Functions : 

double manhattanDistance(Point a, Point b) {
    return abs(a.x - b.x) + abs(a.y - b.y);
}

double euclideanDistance(Point a, Point b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

double diagonalDistance(Point a, Point b) {
    return max(abs(a.x - b.x), abs(a.y - b.y));
}

//  A* Search Implementation : 

class AStarSolver {
public:
    typedef vector<vector<int>> Grid;
    typedef function<double(Point, Point)> Heuristic;

    struct Stats {
        double totalPathLength = 0.0;
        int expandedNodes = 0;
        long long timeMicroseconds = 0;
        vector<Point> path;
    };

    Stats run(Grid& grid, Point start, Point goal, Heuristic heuristic) {
        auto startTime = chrono::high_resolution_clock::now();

        Stats stats;
        priority_queue<Node, vector<Node>, greater<Node>> openSet;
        set<Point> closedSet;

        openSet.push(Node(start, 0.0, heuristic(start, goal), start));

        const vector<tuple<int, int, double>> directions = {
            {0, 1, COST_STRAIGHT}, {1, 0, COST_STRAIGHT}, {0, -1, COST_STRAIGHT}, {-1, 0, COST_STRAIGHT},
            {1, 1, COST_DIAGONAL}, {1, -1, COST_DIAGONAL}, {-1, 1, COST_DIAGONAL}, {-1, -1, COST_DIAGONAL}
        };

        map<Point, Point> cameFrom;
        map<Point, double> gValues;
        gValues[start] = 0.0;

        while (!openSet.empty()) {
            Node current = openSet.top();
            openSet.pop();
            stats.expandedNodes++;

            if (current.position == goal) {
                stats.totalPathLength = current.gCost;
                Point trace = goal;
                while (!(trace == start)) {
                    stats.path.push_back(trace);
                    trace = cameFrom[trace];
                }
                stats.path.push_back(start);
                reverse(stats.path.begin(), stats.path.end());
                break;
            }

            if (closedSet.count(current.position)) continue;
            closedSet.insert(current.position);

            for (size_t i = 0; i < directions.size(); ++i) {
                int dx = get<0>(directions[i]);
                int dy = get<1>(directions[i]);
                double stepCost = get<2>(directions[i]);

                Point neighbor = {current.position.x + dx, current.position.y + dy};

                if (neighbor.x < 0 || neighbor.x >= GRID_WIDTH || neighbor.y < 0 || neighbor.y >= GRID_HEIGHT)
                    continue;

                if (grid[neighbor.y][neighbor.x] == CELL_OBSTACLE)
                    continue;

                double tentativeCost = current.gCost + stepCost;

                if (!gValues.count(neighbor) || tentativeCost < gValues[neighbor]) {
                    gValues[neighbor] = tentativeCost;
                    double h = heuristic(neighbor, goal);
                    openSet.push(Node(neighbor, tentativeCost, h, current.position));
                    cameFrom[neighbor] = current.position;
                }
            }
        }

        auto endTime = chrono::high_resolution_clock::now();
        stats.timeMicroseconds = chrono::duration_cast<chrono::microseconds>(endTime - startTime).count();
        return stats;
    }

    //  Grid Functions : 

    Grid generateGrid(Point& start, Point& goal) {
        Grid grid(GRID_HEIGHT, vector<int>(GRID_WIDTH, CELL_EMPTY));
        random_device rd;
        mt19937 engine(rd());
        uniform_real_distribution<> prob(0.0, 1.0);
        uniform_int_distribution<> randX(0, GRID_WIDTH - 1);
        uniform_int_distribution<> randY(0, GRID_HEIGHT - 1);

        // Random obstacle generation
        for (int y = 0; y < GRID_HEIGHT; ++y)
            for (int x = 0; x < GRID_WIDTH; ++x)
                if (prob(engine) < OBSTACLE_RATIO)
                    grid[y][x] = CELL_OBSTACLE;

        do { start = {randX(engine), randY(engine)}; }
        while (grid[start.y][start.x] == CELL_OBSTACLE);

        do { goal = {randX(engine), randY(engine)}; }
        while (grid[goal.y][goal.x] == CELL_OBSTACLE || start == goal);

        return grid;
    }

    void displayGrid(const Grid& grid, const Stats& stats) {
        map<Point, char> pathMap;
        for (size_t i = 0; i < stats.path.size(); ++i)
            pathMap[stats.path[i]] = '*';

        Point start = stats.path.empty() ? Point{-1, -1} : stats.path.front();
        Point goal = stats.path.empty() ? Point{-1, -1} : stats.path.back();

        for (int y = 0; y < GRID_HEIGHT; ++y) {
            for (int x = 0; x < GRID_WIDTH; ++x) {
                Point p = {x, y};
                if (p == start) cout << "S ";
                else if (p == goal) cout << "G ";
                else if (pathMap.count(p)) cout << "* ";
                else if (grid[y][x] == CELL_OBSTACLE) cout << "# ";
                else cout << ". ";
            }
            cout << endl;
        }
    }
};

int main() {
    AStarSolver solver;
    int runs = 100;

    map<string, AStarSolver::Heuristic> heuristics;
    heuristics["Manhattan"] = manhattanDistance;
    heuristics["Euclidean"] = euclideanDistance;
    heuristics["Diagonal"] = diagonalDistance;

    map<string, AStarSolver::Stats> totalStats;
    map<string, int> pathsFound;

    cout << "Running " << runs << " tests for each heuristic..." << endl;
    cout << "Grid: " << GRID_WIDTH << "x" << GRID_HEIGHT
         << " | Obstacle Density: " << (OBSTACLE_RATIO * 100) << "%" << endl;

    for (int i = 0; i < runs; ++i) {
        Point start, goal;
        AStarSolver::Grid grid = solver.generateGrid(start, goal);

        for (map<string, AStarSolver::Heuristic>::const_iterator it = heuristics.begin(); it != heuristics.end(); ++it) {
            string name = it->first;
            AStarSolver::Heuristic func = it->second;

            AStarSolver::Stats result = solver.run(grid, start, goal, func);
            if (!result.path.empty()) {
                totalStats[name].expandedNodes += result.expandedNodes;
                totalStats[name].totalPathLength += result.totalPathLength;
                totalStats[name].timeMicroseconds += result.timeMicroseconds;
                pathsFound[name]++;
            }
        }
    }

    cout << "\n Average Results (Final Report) " << endl;
    cout << left << setw(12) << "Heuristic" << " | "
         << setw(15) << "Avg Nodes" << " | "
         << setw(15) << "Avg Path" << " | "
         << setw(15) << "Avg Time (us)" << " | Paths Found" << endl;
    cout << string(70, '-') << endl;

    for (map<string, AStarSolver::Heuristic>::const_iterator it = heuristics.begin(); it != heuristics.end(); ++it) {
        string name = it->first;
        int count = pathsFound[name];
        if (count > 0) {
            cout << left << setw(12) << name << " | "
                 << setw(15) << (totalStats[name].expandedNodes / count) << " | "
                 << setw(15) << fixed << setprecision(2)
                 << (totalStats[name].totalPathLength / count) << " | "
                 << setw(15) << (totalStats[name].timeMicroseconds / count) << " | "
                 << count << "/" << runs << endl;
        } else {
            cout << left << setw(12) << name << " | N/A" << endl;
        }
    }

    cout << "\nDemonstration Run (using Euclidean)" << endl;
    Point start, goal;
    AStarSolver::Grid grid = solver.generateGrid(start, goal);
    AStarSolver::Stats demo = solver.run(grid, start, goal, euclideanDistance);
    solver.displayGrid(grid, demo);
    cout << "\nPath Length: " << demo.totalPathLength << endl;
    cout << "Nodes Expanded: " << demo.expandedNodes << endl;
    cout << "Time Taken: " << demo.timeMicroseconds << " us" << endl;

    return 0;
}
