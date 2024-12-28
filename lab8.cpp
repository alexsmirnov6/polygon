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
    return std::sqrt(std::pow((a.lon - b.lon), 2) + std::pow((a.lat - b.lat), 2));
}

// Поиск ближайшей ноды
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

// Парсинг графа
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
            graph[target].push_back({ parent, weight });
        }
    }

    return graph;
}

std::vector<Node> aStar(const Graph& graph, const Node& start, const Node& goal) {
    std::unordered_map<Node, double, NodeHash> gScore; // ~48 байт
    std::unordered_map<Node, double, NodeHash> fScore; // ~48 байт
    std::unordered_map<Node, Node, NodeHash> cameFrom; // ~48 байт
    auto compare = [&](const Node& lhs, const Node& rhs) { return fScore[lhs] > fScore[rhs]; }; // 1 байт
    std::priority_queue<Node, std::vector<Node>, decltype(compare)> openSet(compare); // ~24 байта

    gScore[start] = 0;
    fScore[start] = heuristic(start, goal);
    openSet.push(start);

    while (!openSet.empty()) { // O(log N) — для каждого извлечения из очереди при условии, что очередь содержит N элементов
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

        if (graph.find(current) == graph.end()) { // O(1) — поиск в unordered_map
            continue;
        }

        for (const auto& edge : graph.at(current)) { // O(E) — для каждого соседа (E — количество рёбер)
            double tentativeGScore = gScore[current] + edge.weight;
            if (!gScore.count(edge.target) || tentativeGScore < gScore[edge.target]) { // O(1)
                cameFrom[edge.target] = current; // O(1)
                gScore[edge.target] = tentativeGScore; // O(1)
                fScore[edge.target] = gScore[edge.target] + heuristic(edge.target, goal); // O(1)
                openSet.push(edge.target); // O(log N) — вставка в priority_queue
            }
        }
    }

    return {}; // O(1)
}

std::vector<Node> bfs(const Graph& graph, const Node& start, const Node& goal) {
    std::queue<Node> q; // ~24 байта
    std::unordered_map<Node, Node, NodeHash> cameFrom; // ~48 байт
    std::unordered_map<Node, bool, NodeHash> visited; // ~48 байт

    q.push(start);
    visited[start] = true;

    while (!q.empty()) { // O(V) — обработка всех вершин в графе
        Node current = q.front();
        q.pop();

        if (current == goal) { // O(1) — проверка на достижение цели
            std::vector<Node> path;
            while (current != start) { // O(V) — построение пути
                path.push_back(current);
                current = cameFrom[current];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end()); // O(V)
            return path;
        }

        if (graph.find(current) != graph.end()) { // O(1) — поиск в unordered_map
            for (const auto& edge : graph.at(current)) { // O(E) — обработка всех рёбер
                if (!visited[edge.target]) { // O(1)
                    visited[edge.target] = true; // O(1)
                    cameFrom[edge.target] = current; // O(1)
                    q.push(edge.target); // O(1)
                }
            }
        }
    }

    return {}; // O(1)
}

std::vector<Node> dfsUtil(const Graph& graph, const Node& current, const Node& goal,
    std::unordered_map<Node, bool, NodeHash>& visited, // ~48 байт
    std::unordered_map<Node, Node, NodeHash>& cameFrom) { // ~48 байт
    visited[current] = true;

    if (current == goal) { // O(1)
        std::vector<Node> path;
        Node temp = current;
        while (temp != Node{ -1, -1 }) { // O(V)
            path.push_back(temp);
            temp = cameFrom[temp];
        }
        std::reverse(path.begin(), path.end()); // O(V)
        return path;
    }

    if (graph.find(current) != graph.end()) { // O(1)
        for (const auto& edge : graph.at(current)) { // O(E)
            if (!visited[edge.target]) { // O(1)
                cameFrom[edge.target] = current; // O(1)
                auto path = dfsUtil(graph, edge.target, goal, visited, cameFrom); // рекурсивный вызов
                if (!path.empty()) return path; // O(1)
            }
        }
    }

    return {}; // O(1)
}

std::vector<Node> dfs(const Graph& graph, const Node& start, const Node& goal) {
    std::unordered_map<Node, bool, NodeHash> visited; // ~48 байт
    std::unordered_map<Node, Node, NodeHash> cameFrom; // ~48 байт
    cameFrom[start] = Node{ -1, -1 };
    return dfsUtil(graph, start, goal, visited, cameFrom);
}

std::vector<Node> dijkstra(const Graph& graph, const Node& start, const Node& goal) {
    std::unordered_map<Node, double, NodeHash> dist; // ~48 байт
    std::unordered_map<Node, Node, NodeHash> cameFrom; // ~48 байт
    auto compare = [&](const Node& lhs, const Node& rhs) { return dist[lhs] > dist[rhs]; }; // 1 байт
    std::priority_queue<Node, std::vector<Node>, decltype(compare)> pq(compare); // ~24 байта

    for (const auto& pair : graph) { // O(V) — инициализация всех расстояний
        dist[pair.first] = std::numeric_limits<double>::infinity();
    }

    dist[start] = 0;
    pq.push(start);

    while (!pq.empty()) { // O(log V) — извлечение минимального элемента
        Node current = pq.top();
        pq.pop();

        if (current == goal) { // O(1)
            std::vector<Node> path;
            while (current != start) { // O(V) — построение пути
                path.push_back(current);
                current = cameFrom[current];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end()); // O(V)
            return path;
        }

        if (graph.find(current) != graph.end()) { // O(1)
            for (const auto& edge : graph.at(current)) { // O(E)
                double newDist = dist[current] + edge.weight;
                if (newDist < dist[edge.target]) { // O(1)
                    dist[edge.target] = newDist; // O(1)
                    cameFrom[edge.target] = current; // O(1)
                    pq.push(edge.target); // O(log V) — вставка в priority_queue
                }
            }
        }
    }

    return {}; // O(1)
}

void testPathfindingAlgorithms() {
    // Пример графа
    Graph graph;

    graph[Node{0, 0}] = { {Node{0, 1}, 1.0}, {Node{1, 0}, 1.0} };
    graph[Node{0, 1}] = { {Node{0, 0}, 1.0}, {Node{1, 1}, 1.0} };
    graph[Node{1, 0}] = { {Node{0, 0}, 1.0}, {Node{1, 1}, 1.0} };
    graph[Node{1, 1}] = { {Node{0, 1}, 1.0}, {Node{1, 0}, 1.0} };

    Node start = Node{0, 0};
    Node goal = Node{1, 1};

    // Применяем алгоритмы поиска пути
    std::vector<Node> aStarPath = aStar(graph, start, goal);
    std::vector<Node> bfsPath = bfs(graph, start, goal);
    std::vector<Node> dfsPath = dfs(graph, start, goal);
    std::vector<Node> dijkstraPath = dijkstra(graph, start, goal);

    // Проверяем, что все пути корректны
    assert(isValidPath(graph, aStarPath));
    assert(isValidPath(graph, bfsPath));
    assert(isValidPath(graph, dfsPath));
    assert(isValidPath(graph, dijkstraPath));

    std::cout << "Все алгоритмы прошли тесты!" << std::endl;
}

int main() {
    testPathfindingAlgorithms();
    Graph graph = parseGraph("spb_graph.txt");

    Node start{ 30.222172, 59.936539 }; // Рандомная локация
    Node goal{ 30.308108, 59.957238 }; // Кронва

    Node closestNode_start = findClosestNode(graph, start);
    Node closestNode_goal = findClosestNode(graph, goal);

    std::cout << "Поиск кратчайшего пути алгоритмом A*..." << std::endl;
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
        std::cout << "Путь не найден." << std::endl;
    }
    std::cout << "Затрачено " << seconds1 << " секунд\n" << "~~~" << std::endl;

    
    std::cout << "Поиск кратчайшего пути алгоритмом Дейкстры..." << std::endl;
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
        std::cout << "Путь не найден." << std::endl;
    }
    std::cout << "Затрачено " << seconds2 << " секунд\n" << "~~~" << std::endl;


    std::cout << "Поиск кратчайшего пути алгоритмом обзода в ширину..." << std::endl;
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
        std::cout << "Путь не найден." << std::endl;
    }
    std::cout << "Затрачено " << seconds3 << " секунд\n" << "~~~" << std::endl;


    std::cout << "Поиск кратчайшего пути алгоритмом обхода в глубину..." << std::endl;
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
        std::cout << "Путь не найден." << std::endl;
    }
    std::cout << "Затрачено " << seconds4 << " секунд\n" << "~~~" << std::endl;

    return 0;
}
