#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <cmath>
#include <limits>
#include <tuple>
#include <functional>
#include <algorithm>
#include <time.h>

struct Node {
    double lon;
    double lat;

    bool operator==(const Node& other) const {
        return lon == other.lon && lat == other.lat;
    }
    bool operator!=(const Node& other) const {
        return !(*this == other);
    }

};

struct Edge {
    Node target;
    double weight;
};

struct NodeHash {
    std::size_t operator()(const Node& node) const {
        return std::hash<std::string>()(std::to_string(node.lon) + "," + std::to_string(node.lat));
    }
};

using Graph = std::unordered_map<Node, std::vector<Edge>, NodeHash>;

double heuristic(const Node& a, const Node& b) {
    return std::sqrt((a.lon - b.lon) * (a.lon - b.lon) + (a.lat - b.lat) * (a.lat - b.lat));
}

// Нахождение ближайшей ноды
Node findClosestNode(const Graph& graph, const Node& point) {
    Node closestNode;
    double minDistance = std::numeric_limits<double>::infinity();

    for (const auto& pair : graph) {
        const Node& node = pair.first;
        double distance = heuristic(node, point);
        if (distance < minDistance) {
            minDistance = distance;
            closestNode = node;
        }
    }

    return closestNode;
}

// Парсинг из файла
Graph parseGraph(const std::string& filename) {
    Graph graph;
    std::ifstream file(filename);
    std::string line;

    while (std::getline(file, line)) {
        std::istringstream ss(line);
        std::string nodePart, edgesPart;

        std::getline(ss, nodePart, ':');
        Node parent;
        sscanf_s(nodePart.c_str(), "%lf,%lf", &parent.lon, &parent.lat);

        while (std::getline(ss, edgesPart, ';')) {
            if (edgesPart.empty()) continue;
            Node target;
            double weight;
            sscanf_s(edgesPart.c_str(), "%lf,%lf,%lf", &target.lon, &target.lat, &weight);
            graph[parent].push_back({ target, weight });
        }
    }

    return graph;
}

std::vector<Node> aStar(const Graph& graph, const Node& start, const Node& goal) {
    std::unordered_map<Node, double, NodeHash> gScore;
    std::unordered_map<Node, double, NodeHash> fScore;
    std::unordered_map<Node, Node, NodeHash> cameFrom;
    auto compare = [&](const Node& lhs, const Node& rhs) { return fScore[lhs] > fScore[rhs]; };
    std::priority_queue<Node, std::vector<Node>, decltype(compare)> openSet(compare);

    gScore[start] = 0;
    fScore[start] = heuristic(start, goal);
    openSet.push(start);

    while (!openSet.empty()) {
        Node current = openSet.top();
        openSet.pop();

        if (current == goal) {
            std::vector<Node> path;
            while (current != start) {
                path.push_back(current);
                current = cameFrom[current];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            return path;
        }

        if (graph.find(current) == graph.end()) {
            continue;
        }

        for (const auto& edge : graph.at(current)) {
            double tentativeGScore = gScore[current] + edge.weight;
            if (!gScore.count(edge.target) || tentativeGScore < gScore[edge.target]) {
                cameFrom[edge.target] = current;
                gScore[edge.target] = tentativeGScore;
                fScore[edge.target] = gScore[edge.target] + heuristic(edge.target, goal);
                openSet.push(edge.target);
            }
        }
    }

    return {};
}

std::vector<Node> bfs(const Graph& graph, const Node& start, const Node& goal) {
    std::queue<Node> q;
    std::unordered_map<Node, Node, NodeHash> cameFrom;
    std::unordered_map<Node, bool, NodeHash> visited;

    q.push(start);
    visited[start] = true;

    while (!q.empty()) {
        Node current = q.front();
        q.pop();

        if (current == goal) {
            std::vector<Node> path;
            while (current != start) {
                path.push_back(current);
                current = cameFrom[current];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            return path;
        }

        if (graph.find(current) != graph.end()) {
            for (const auto& edge : graph.at(current)) {
                if (!visited[edge.target]) {
                    visited[edge.target] = true;
                    cameFrom[edge.target] = current;
                    q.push(edge.target);
                }
            }
        }
    }

    return {};
}

std::vector<Node> dfsUtil(const Graph& graph, const Node& current, const Node& goal,
    std::unordered_map<Node, bool, NodeHash>& visited,
    std::unordered_map<Node, Node, NodeHash>& cameFrom) {
    visited[current] = true;

    if (current == goal) {
        std::vector<Node> path;
        Node temp = current;
        while (temp != Node{ -1, -1 }) {
            path.push_back(temp);
            temp = cameFrom[temp];
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

    if (graph.find(current) != graph.end()) {
        for (const auto& edge : graph.at(current)) {
            if (!visited[edge.target]) {
                cameFrom[edge.target] = current;
                auto path = dfsUtil(graph, edge.target, goal, visited, cameFrom);
                if (!path.empty()) return path;
            }
        }
    }

    return {};
}

std::vector<Node> dfs(const Graph& graph, const Node& start, const Node& goal) {
    std::unordered_map<Node, bool, NodeHash> visited;
    std::unordered_map<Node, Node, NodeHash> cameFrom;
    cameFrom[start] = Node{ -1, -1 };
    return dfsUtil(graph, start, goal, visited, cameFrom);
}

std::vector<Node> dijkstra(const Graph& graph, const Node& start, const Node& goal) {
    std::unordered_map<Node, double, NodeHash> dist;
    std::unordered_map<Node, Node, NodeHash> cameFrom;
    auto compare = [&](const Node& lhs, const Node& rhs) { return dist[lhs] > dist[rhs]; };
    std::priority_queue<Node, std::vector<Node>, decltype(compare)> pq(compare);

    for (const auto& pair : graph) {
        dist[pair.first] = std::numeric_limits<double>::infinity();
    }

    dist[start] = 0;
    pq.push(start);

    while (!pq.empty()) {
        Node current = pq.top();
        pq.pop();

        if (current == goal) {
            std::vector<Node> path;
            while (current != start) {
                path.push_back(current);
                current = cameFrom[current];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            return path;
        }

        if (graph.find(current) != graph.end()) {
            for (const auto& edge : graph.at(current)) {
                double newDist = dist[current] + edge.weight;
                if (newDist < dist[edge.target]) {
                    dist[edge.target] = newDist;
                    cameFrom[edge.target] = current;
                    pq.push(edge.target);
                }
            }
        }
    }

    return {};
}

int main() {
    Graph graph = parseGraph("spb_graph.txt");

    Node start{ 29.922172, 59.936539 }; // Стрипбар на Невском
    Node goal{ 30.208108, 60.057238 }; // ИТМО Кронверкский 49

    Node closestNode_start = findClosestNode(graph, start);
    Node closestNode_goal = findClosestNode(graph, goal);

    std::cout << "Поиск пути с использованием алгоритма A*..." << std::endl;
    clock_t startTimer1 = clock();
    auto path1 = aStar(graph, closestNode_start, closestNode_goal);
    clock_t endTimer1 = clock();
    double seconds1 = (double)(endTimer1 - startTimer1) / CLOCKS_PER_SEC;
    if (!path1.empty()) {
        std::cout << "Путь найден:" << std::endl;
        for (const auto& node : path1) {
            std::cout << "(" << node.lon << ", " << node.lat << ")" << std::endl;
        }
    }
    else {
        std::cout << "Алгоритм не смог найти путь." << std::endl;
    }
    std::cout << "Времени затрачено: " << seconds1 << " секунд\n" << "~~~" << std::endl;

    
    std::cout << "Поиск пути с использованием алгоритма Дейкстры..." << std::endl;
    clock_t startTimer2 = clock();
    auto path2 = dijkstra(graph, closestNode_start, closestNode_goal);
    clock_t endTimer2 = clock();
    double seconds2 = (double)(endTimer2 - startTimer2) / CLOCKS_PER_SEC;
    if (!path2.empty()) {
        std::cout << "Путь найден:" << std::endl;
        for (const auto& node : path2) {
            std::cout << "(" << node.lon << ", " << node.lat << ")\n";
        }
    }
    else {
        std::cout << "Алгоритм не смог найти путь." << std::endl;
    }
    std::cout << "Времени затрачено: " << seconds2 << " секунд\n" << "~~~" << std::endl;


    std::cout << "Поиск пути с использованием алгоритма обхода в ширину..." << std::endl;
    clock_t startTimer3 = clock();
    auto path3 = bfs(graph, closestNode_start, closestNode_goal);
    clock_t endTimer3 = clock();
    double seconds3 = (double)(endTimer3 - startTimer3) / CLOCKS_PER_SEC;
    if (!path3.empty()) {
        std::cout << "Путь найден:" << std::endl;
        for (const auto& node : path3) {
            std::cout << "(" << node.lon << ", " << node.lat << ")\n";
        }
    }
    else {
        std::cout << "Алгоритм не смог найти путь." << std::endl;
    }
    std::cout << "Времени затрачено: " << seconds3 << " секунд\n" << "~~~" << std::endl;


    std::cout << "Поиск пути с использованием алгоритма обхода в глубину..." << std::endl;
    clock_t startTimer4 = clock();
    auto path4 = dfs(graph, closestNode_start, closestNode_goal);
    clock_t endTimer4 = clock();
    double seconds4 = (double)(endTimer4 - startTimer4) / CLOCKS_PER_SEC;
    if (!path4.empty()) {
        std::cout << "Путь найден:" << std::endl;
        for (const auto& node : path4) {
            std::cout << "(" << node.lon << ", " << node.lat << ")\n";
        }
    }
    else {
        std::cout << "Алгоритм не смог найти путь." << std::endl;
    }
    std::cout << "Времени затрачено: " << seconds4 << " секунд\n" << "~~~" << std::endl;

    return 0;
}
