#include <iostream>
#include <vector>
#include <queue>
#include <random>
#include <ctime>
#include <limits>
#include <unordered_map>
#include <algorithm>
#include <functional>
#include <chrono>
#include <numeric>
#include <string>
#include <iomanip>
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/heap/binomial_heap.hpp>
#include <boost/heap/d_ary_heap.hpp>
#include <boost/heap/skew_heap.hpp>
#include <boost/heap/pairing_heap.hpp>

using namespace std;

class Node 
{
public:
    int id, layer;
    Node(int id = 0, int layer = 0) : id(id), layer(layer) {}
};

class Edge 
{
public:
    int from, to, length;
    Edge(int from, int to, int length) : from(from), to(to), length(length) {}
};

class Graph 
{
public:

    vector<vector<pair<int, int>>> adj_list;
    vector<Node> nodes;
    vector<Edge> edges;
    int node_num;
    int edge_num;

    Graph(int n, int m) // m is the number of nodes in a layer
    { 
        node_num = n;
        edge_num = 0;
        nodes = vector<Node>(n);
        nodes[0] = Node(0, 0);
        for (int i = 1; i < n - 1; ++i) 
            nodes[i] = Node(i, (i - 1) / m + 1);
        nodes[n - 1] = Node(n - 1, (n - 3) / m + 2);
        adj_list = vector<vector<pair<int, int>>>(node_num);
    }

    void insert_edge(int from, int to, int length) 
    {
        if (from < node_num && to < node_num && from >= 0 && to >= 0) 
        {
            edges.push_back(Edge(from, to, length));
            edge_num++;
        }
        else
            cout << "Error in bounds!" << endl;
    }

    void connect_layers(int max_length) // making each layer fully connected with the next layer with weights being random (seeded with current time)
    { 
        random_device rd;
        mt19937 gen(rd());
        uniform_int_distribution<> dist(1, max_length);
        for (int i = 0; i < nodes.size(); ++i) 
        {
            for (int j = i + 1; j < nodes.size(); ++j) 
            {
                if (nodes[j].layer == nodes[i].layer + 1) 
                    insert_edge(i, j, dist(gen));
            }
        }
        for (const auto& edge : edges)
            adj_list[edge.from].emplace_back(edge.to, edge.length);
    }

    void print_graph()
    {
        cout << "Nodes:" << endl;
        for (const auto& node : nodes)
            cout << "Node " << node.id << " (Layer " << node.layer << ")" << endl;
        cout << "Edges:" << endl;
        for (const auto& edge : edges)
            cout << "Edge from Node " << edge.from << " to Node " << edge.to << " with length " << edge.length << endl;
    }
};

void bellman_ford(const Graph& graph, int start, int goal) 
{
    vector<int> dist(graph.node_num, numeric_limits<int>::max());
    dist[start] = 0;

    for (int i = 0; i < graph.node_num - 1; ++i) 
    {
        for (const auto& edge : graph.edges) 
        {
            if (dist[edge.from] != numeric_limits<int>::max() && dist[edge.from] + edge.length < dist[edge.to]) 
                dist[edge.to] = dist[edge.from] + edge.length;
        }
    }
    cout << "Bellman-Ford distance from " << start << " to " << goal << ": " << dist[goal] << endl;
}

void dijkstra_brute_force_ve(const Graph& graph, int start, int goal) 
{
    vector<int> dist(graph.node_num, numeric_limits<int>::max());
    vector<bool> visited(graph.node_num, false);
    dist[start] = 0;

    for (int i = 0; i < graph.node_num; ++i) 
    {
        int u = -1;
        for (int j = 0; j < graph.node_num; ++j) 
        {
            if (!visited[j] && (u == -1 || dist[j] < dist[u])) {
                u = j;
            }
        }

        if (u == -1 || dist[u] == numeric_limits<int>::max()) 
            break;

        visited[u] = true;
        for (const auto& edge : graph.edges) 
        {
            if (edge.from == u && dist[u] + edge.length < dist[edge.to]) 
                dist[edge.to] = dist[u] + edge.length;
        }
    }
    cout << "Dijkstra (Brute Force VE) distance from " << start << " to " << goal << ": " << dist[goal] << endl;
}

void dijkstra_brute_force_vv(const Graph& graph, int start, int goal) 
{
    vector<int> dist(graph.node_num, numeric_limits<int>::max());
    vector<bool> visited(graph.node_num, false);
    dist[start] = 0;

    for (int i = 0; i < graph.node_num; ++i) 
    {
        int u = -1;

        // Find the unvisited node with the smallest distance
        for (int j = 0; j < graph.node_num; ++j) 
        {
            if (!visited[j] && (u == -1 || dist[j] < dist[u]))
                u = j;
        }

        if (u == -1 || dist[u] == numeric_limits<int>::max()) 
            break;

        visited[u] = true;

        // Relax all neighbors of u
        for (const auto& [v, length] : graph.adj_list[u]) 
        {
            if (dist[u] + length < dist[v]) 
                dist[v] = dist[u] + length;
        }
    }
    cout << "Dijkstra (Brute Force VV) distance from " << start << " to " << goal << ": " << dist[goal] << endl;
}

void dijkstra_binary_heap(const Graph& graph, int start, int goal) 
{
    using BinaryHeap = boost::heap::d_ary_heap<pair<int, int>, boost::heap::arity<2>, boost::heap::mutable_<true>, boost::heap::compare<greater<>>>;
    vector<int> dist(graph.node_num, numeric_limits<int>::max());
    vector<BinaryHeap::handle_type> handles(graph.node_num);
    BinaryHeap pq;

    dist[start] = 0;
    for (int i = 0; i < graph.node_num; ++i) 
        handles[i] = pq.push({ dist[i], i });

    while (!pq.empty()) 
    {
        int u = pq.top().second;
        pq.pop();
        if (u == goal) 
            break;
        for (const auto& [v, length] : graph.adj_list[u]) 
        {
            if (dist[u] + length < dist[v]) 
            {
                dist[v] = dist[u] + length;
                pq.update(handles[v], { dist[v], v });
            }
        }
    }
    cout << "Dijkstra (Binary Heap) distance from " << start << " to " << goal << ": " << dist[goal] << endl;
}

void dijkstra_ternary_heap(const Graph& graph, int start, int goal) 
{
    using TernaryHeap = boost::heap::d_ary_heap<pair<int, int>, boost::heap::arity<3>, boost::heap::mutable_<true>, boost::heap::compare<greater<>>>;
    vector<int> dist(graph.node_num, numeric_limits<int>::max());
    vector<TernaryHeap::handle_type> handles(graph.node_num);
    TernaryHeap pq;

    dist[start] = 0;
    for (int i = 0; i < graph.node_num; ++i) 
        handles[i] = pq.push({ dist[i], i });

    while (!pq.empty()) 
    {
        int u = pq.top().second;
        pq.pop();

        if (u == goal) 
            break;

        for (const auto& [v, length] : graph.adj_list[u]) 
        {
            if (dist[u] + length < dist[v]) 
            {
                dist[v] = dist[u] + length;
                pq.update(handles[v], { dist[v], v });
            }
        }
    }
    cout << "Dijkstra (Ternary Heap) distance from " << start << " to " << goal << ": " << dist[goal] << endl;
}

void dijkstra_quaternary_heap(const Graph& graph, int start, int goal) 
{
    using QuaternaryHeap = boost::heap::d_ary_heap<pair<int, int>, boost::heap::arity<4>, boost::heap::mutable_<true>, boost::heap::compare<greater<>>>;
    vector<int> dist(graph.node_num, numeric_limits<int>::max());
    vector<QuaternaryHeap::handle_type> handles(graph.node_num);
    QuaternaryHeap pq;

    dist[start] = 0;
    for (int i = 0; i < graph.node_num; ++i) 
        handles[i] = pq.push({ dist[i], i });

    while (!pq.empty()) 
    {
        int u = pq.top().second;
        pq.pop();

        if (u == goal) 
            break;

        for (const auto& [v, length] : graph.adj_list[u]) 
        {
            if (dist[u] + length < dist[v]) 
            {
                dist[v] = dist[u] + length;
                pq.update(handles[v], { dist[v], v });
            }
        }
    }
    cout << "Dijkstra (Quaternary Heap) distance from " << start << " to " << goal << ": " << dist[goal] << endl;
}

void dijkstra_denary_heap(const Graph& graph, int start, int goal) 
{
    using DenaryHeap = boost::heap::d_ary_heap<pair<int, int>, boost::heap::arity<10>, boost::heap::mutable_<true>, boost::heap::compare<greater<>>>;
    vector<int> dist(graph.node_num, numeric_limits<int>::max());
    vector<DenaryHeap::handle_type> handles(graph.node_num);
    DenaryHeap pq;

    dist[start] = 0;
    for (int i = 0; i < graph.node_num; ++i) 
        handles[i] = pq.push({ dist[i], i });

    while (!pq.empty()) 
    {
        int u = pq.top().second;
        pq.pop();

        if (u == goal) 
            break;

        for (const auto& [v, length] : graph.adj_list[u]) 
        {
            if (dist[u] + length < dist[v]) 
            {
                dist[v] = dist[u] + length;
                pq.update(handles[v], { dist[v], v });
            }
        }
    }
    cout << "Dijkstra (Denary Heap) distance from " << start << " to " << goal << ": " << dist[goal] << endl;
}

void dijkstra_binomial_heap(const Graph& graph, int start, int goal) 
{
    using BinHeap = boost::heap::binomial_heap<pair<int, int>, boost::heap::compare<greater<>>>;
    vector<int> dist(graph.node_num, numeric_limits<int>::max());
    vector<BinHeap::handle_type> handles(graph.node_num);
    BinHeap pq;

    dist[start] = 0;
    for (int i = 0; i < graph.node_num; ++i) 
        handles[i] = pq.push({ dist[i], i });

    while (!pq.empty()) 
    {
        int u = pq.top().second;
        pq.pop();

        if (u == goal) 
            break;

        for (const auto& [v, length] : graph.adj_list[u]) 
        {
            if (dist[u] + length < dist[v]) 
            {
                dist[v] = dist[u] + length;
                pq.update(handles[v], { dist[v], v });
            }
        }
    }
    cout << "Dijkstra (Binomial Heap) distance from " << start << " to " << goal << ": " << dist[goal] << endl;
}

void dijkstra_skew_heap(const Graph& graph, int start, int goal) 
{
    using SkewHeap = boost::heap::skew_heap<pair<int, int>, boost::heap::mutable_<true>, boost::heap::compare<greater<>>>;
    vector<int> dist(graph.node_num, numeric_limits<int>::max());
    vector<SkewHeap::handle_type> handles(graph.node_num);
    SkewHeap pq;

    dist[start] = 0;
    for (int i = 0; i < graph.node_num; ++i) 
        handles[i] = pq.push({ dist[i], i });

    while (!pq.empty()) 
    {
        int u = pq.top().second;
        pq.pop();

        if (u == goal) 
            break;

        for (const auto& [v, length] : graph.adj_list[u]) 
        {
            if (dist[u] + length < dist[v]) 
            {
                dist[v] = dist[u] + length;
                pq.update(handles[v], { dist[v], v });
            }
        }
    }
    cout << "Dijkstra (Skew Heap) distance from " << start << " to " << goal << ": " << dist[goal] << endl;
}

void dijkstra_pairing_heap(const Graph& graph, int start, int goal) 
{
    using PairingHeap = boost::heap::pairing_heap<pair<int, int>, boost::heap::compare<greater<>>>;
    vector<int> dist(graph.node_num, numeric_limits<int>::max());
    vector<PairingHeap::handle_type> handles(graph.node_num);
    PairingHeap pq;

    dist[start] = 0;
    for (int i = 0; i < graph.node_num; ++i) 
        handles[i] = pq.push({ dist[i], i });

    while (!pq.empty()) 
    {
        int u = pq.top().second;
        pq.pop();

        if (u == goal) break;

        for (const auto& [v, length] : graph.adj_list[u]) 
        {
            if (dist[u] + length < dist[v]) 
            {
                dist[v] = dist[u] + length;
                pq.update(handles[v], { dist[v], v });
            }
        }
    }
    cout << "Dijkstra (Pairing Heap) distance from " << start << " to " << goal << ": " << dist[goal] << endl;
}

void dijkstra_fibonacci_heap(const Graph& graph, int start, int goal) 
{
    using FibHeap = boost::heap::fibonacci_heap<pair<int, int>, boost::heap::compare<greater<>>>;
    vector<int> dist(graph.node_num, numeric_limits<int>::max());
    vector<FibHeap::handle_type> handles(graph.node_num);
    FibHeap pq;

    dist[start] = 0;
    for (int i = 0; i < graph.node_num; ++i) 
        handles[i] = pq.push({ dist[i], i });

    while (!pq.empty()) 
    {
        int u = pq.top().second;
        pq.pop();

        for (const auto& [v, length] : graph.adj_list[u]) 
        {
            if (dist[u] + length < dist[v]) 
            {
                dist[v] = dist[u] + length;
                pq.update(handles[v], { dist[v], v });
            }
        }
    }
    cout << "Dijkstra (Fibonacci Heap) distance from " << start << " to " << goal << ": " << dist[goal] << endl;
}

void a_star_brute_force_ve(const Graph& graph, int start, int goal, int max_length) 
{
    vector<int> dist(graph.node_num, numeric_limits<int>::max());
    vector<bool> visited(graph.node_num, false);
    vector<int> heuristic(graph.node_num);

    // Calculate heuristic
    for (const auto& node : graph.nodes) 
        heuristic[node.id] = ((max_length + 1) / 2) * abs(graph.nodes[goal].layer - node.layer);

    dist[start] = 0;

    for (int i = 0; i < graph.node_num; ++i) 
    {
        int u = -1;
        int min_f = numeric_limits<int>::max();

        // Find the unvisited node with the smallest f(u) = g(u) + h(u)
        for (int j = 0; j < graph.node_num; ++j) 
        {
            if (!visited[j] && dist[j] != numeric_limits<int>::max()) 
            {
                int f = dist[j] + heuristic[j];
                if (f < min_f) 
                {
                    min_f = f;
                    u = j;
                }
            }
        }

        if (u == -1 || u == goal) 
            break; 

        visited[u] = true;

        // Relax edges
        for (const auto& edge : graph.edges) 
        {
            if (edge.from == u) 
            {
                int v = edge.to;
                if (!visited[v] && dist[u] + edge.length < dist[v]) 
                    dist[v] = dist[u] + edge.length;
            }
        }
    }
    cout << "A* (Brute Force VE) distance from " << start << " to " << goal << ": " << dist[goal] << endl;
}

void a_star_brute_force_vv(const Graph& graph, int start, int goal, int max_length) 
{
    vector<int> dist(graph.node_num, numeric_limits<int>::max());
    vector<bool> visited(graph.node_num, false);
    vector<int> heuristic(graph.node_num);

    // Calculate heuristic
    for (const auto& node : graph.nodes) 
        heuristic[node.id] = ((max_length + 1) / 2) * abs(graph.nodes[goal].layer - node.layer);

    dist[start] = 0;

    for (int i = 0; i < graph.node_num; ++i) 
    {
        int u = -1;
        int min_f = numeric_limits<int>::max();

        // Find the unvisited node with the smallest f(u) = g(u) + h(u)
        for (int j = 0; j < graph.node_num; ++j) 
        {
            if (!visited[j] && dist[j] != numeric_limits<int>::max()) 
            {
                int f = dist[j] + heuristic[j];
                if (f < min_f) 
                {
                    min_f = f;
                    u = j;
                }
            }
        }

        if (u == -1 || u == goal) 
            break; 

        visited[u] = true;

        // Relax edges for all neighbors of `u`
        for (const auto& [v, length] : graph.adj_list[u]) 
        {
            if (!visited[v] && dist[u] + length < dist[v]) 
                dist[v] = dist[u] + length;
        }
    }
    cout << "A* (Brute Force VV) distance from " << start << " to " << goal << ": " << dist[goal] << endl;
}

void a_star_binary_heap(const Graph& graph, int start, int goal, int max_length) 
{
    using BinaryHeap = boost::heap::d_ary_heap<pair<int, int>, boost::heap::arity<2>, boost::heap::mutable_<true>, boost::heap::compare<greater<>>>;
    vector<int> dist(graph.node_num, numeric_limits<int>::max());
    vector<int> heuristic(graph.node_num, 0);
    vector<BinaryHeap::handle_type> handles(graph.node_num);
    BinaryHeap pq;

    // Calculate heuristic
    for (const auto& node : graph.nodes) 
        heuristic[node.id] = ((max_length + 1) / 2) * abs(graph.nodes[goal].layer - node.layer);

    dist[start] = 0;
    for (int i = 0; i < graph.node_num; ++i) 
        handles[i] = pq.push({ dist[i], i });

    while (!pq.empty()) 
    {
        int u = pq.top().second;
        pq.pop();

        if (u == goal) 
            break;

        for (const auto& [v, length] : graph.adj_list[u]) 
        {
            if (dist[u] + length < dist[v]) 
            {
                dist[v] = dist[u] + length;
                pq.update(handles[v], { dist[v] + heuristic[v], v });
            }
        }
    }
    cout << "A* (Binary Heap) distance from " << start << " to " << goal << ": " << dist[goal] << endl;
}

void a_star_ternary_heap(const Graph& graph, int start, int goal, int max_length) 
{
    using TernaryHeap = boost::heap::d_ary_heap<pair<int, int>, boost::heap::arity<2>, boost::heap::mutable_<true>, boost::heap::compare<greater<>>>;
    vector<int> dist(graph.node_num, numeric_limits<int>::max());
    vector<int> heuristic(graph.node_num, 0);
    vector<TernaryHeap::handle_type> handles(graph.node_num);
    TernaryHeap pq;

    // Calculate heuristic
    for (const auto& node : graph.nodes) 
        heuristic[node.id] = ((max_length + 1) / 2) * abs(graph.nodes[goal].layer - node.layer);

    dist[start] = 0;
    for (int i = 0; i < graph.node_num; ++i) 
        handles[i] = pq.push({ dist[i], i });

    while (!pq.empty()) 
    {
        int u = pq.top().second;
        pq.pop();

        if (u == goal) 
            break;

        for (const auto& [v, length] : graph.adj_list[u]) 
        {
            if (dist[u] + length < dist[v]) 
            {
                dist[v] = dist[u] + length;
                pq.update(handles[v], { dist[v] + heuristic[v], v });
            }
        }
    }
    cout << "A* (Ternary Heap) distance from " << start << " to " << goal << ": " << dist[goal] << endl;
}

void a_star_quaternary_heap(const Graph& graph, int start, int goal, int max_length) 
{
    using QuaternaryHeap = boost::heap::d_ary_heap<pair<int, int>, boost::heap::arity<4>, boost::heap::mutable_<true>, boost::heap::compare<greater<>>>;
    vector<int> dist(graph.node_num, numeric_limits<int>::max());
    vector<int> heuristic(graph.node_num, 0);
    vector<QuaternaryHeap::handle_type> handles(graph.node_num);
    QuaternaryHeap pq;

    // Calculate heuristic
    for (const auto& node : graph.nodes) 
        heuristic[node.id] = ((max_length + 1) / 2) * abs(graph.nodes[goal].layer - node.layer);

    dist[start] = 0;
    for (int i = 0; i < graph.node_num; ++i) 
        handles[i] = pq.push({ dist[i], i });

    while (!pq.empty()) 
    {
        int u = pq.top().second;
        pq.pop();

        if (u == goal) 
            break;

        for (const auto& [v, length] : graph.adj_list[u]) 
        {
            if (dist[u] + length < dist[v]) 
            {
                dist[v] = dist[u] + length;
                pq.update(handles[v], { dist[v] + heuristic[v], v });
            }
        }
    }
    cout << "A* (Quaternary Heap) distance from " << start << " to " << goal << ": " << dist[goal] << endl;
}

void a_star_denary_heap(const Graph& graph, int start, int goal, int max_length) 
{
    using DenaryHeap = boost::heap::d_ary_heap<pair<int, int>, boost::heap::arity<10>, boost::heap::mutable_<true>, boost::heap::compare<greater<>>>;
    vector<int> dist(graph.node_num, numeric_limits<int>::max());
    vector<int> heuristic(graph.node_num, 0);
    vector<DenaryHeap::handle_type> handles(graph.node_num);
    DenaryHeap pq;

    // Calculate heuristic
    for (const auto& node : graph.nodes) 
        heuristic[node.id] = ((max_length + 1) / 2) * abs(graph.nodes[goal].layer - node.layer);

    dist[start] = 0;
    for (int i = 0; i < graph.node_num; ++i) 
        handles[i] = pq.push({ dist[i], i });

    while (!pq.empty()) 
    {
        int u = pq.top().second;
        pq.pop();

        if (u == goal) 
            break;

        for (const auto& [v, length] : graph.adj_list[u]) 
        {
            if (dist[u] + length < dist[v]) 
            {
                dist[v] = dist[u] + length;
                pq.update(handles[v], { dist[v] + heuristic[v], v });
            }
        }
    }
    cout << "A* (Denary Heap) distance from " << start << " to " << goal << ": " << dist[goal] << endl;
}

void a_star_binomial_heap(const Graph& graph, int start, int goal, int max_length) 
{
    using BinHeap = boost::heap::binomial_heap<pair<int, int>, boost::heap::compare<greater<>>>;
    vector<int> dist(graph.node_num, numeric_limits<int>::max());
    vector<int> heuristic(graph.node_num, 0);
    vector<BinHeap::handle_type> handles(graph.node_num);
    BinHeap pq;

    // Calculate heuristic
    for (const auto& node : graph.nodes) 
        heuristic[node.id] = ((max_length+1)/2) * abs(graph.nodes[goal].layer - node.layer);

    dist[start] = 0;
    for (int i = 0; i < graph.node_num; ++i) 
        handles[i] = pq.push({ dist[i], i });

    while (!pq.empty()) 
    {
        int u = pq.top().second;
        pq.pop();

        if (u == goal) 
            break;

        for (const auto& [v, length] : graph.adj_list[u]) 
        {
            if (dist[u] + length < dist[v]) 
            {
                dist[v] = dist[u] + length;
                pq.update(handles[v], { dist[v] + heuristic[v], v });
            }
        }
    }
    cout << "A* (Binomial Heap) distance from " << start << " to " << goal << ": " << dist[goal] << endl;
}

void a_star_skew_heap(const Graph& graph, int start, int goal, int max_length) 
{
    using SkewHeap = boost::heap::skew_heap<pair<int, int>, boost::heap::mutable_<true>, boost::heap::compare<greater<>>>;
    vector<int> dist(graph.node_num, numeric_limits<int>::max());
    vector<int> heuristic(graph.node_num, 0);
    vector<SkewHeap::handle_type> handles(graph.node_num);
    SkewHeap pq;

    // Calculate heuristic
    for (const auto& node : graph.nodes) 
        heuristic[node.id] = ((max_length + 1) / 2) * abs(graph.nodes[goal].layer - node.layer);

    dist[start] = 0;
    for (int i = 0; i < graph.node_num; ++i) 
        handles[i] = pq.push({ dist[i], i });

    while (!pq.empty()) 
    {
        int u = pq.top().second;
        pq.pop();

        if (u == goal) 
            break;

        for (const auto& [v, length] : graph.adj_list[u]) 
        {
            if (dist[u] + length < dist[v]) 
            {
                dist[v] = dist[u] + length;
                pq.update(handles[v], { dist[v] + heuristic[v], v });
            }
        }
    }
    cout << "A* (Skew Heap) distance from " << start << " to " << goal << ": " << dist[goal] << endl;
}

void a_star_pairing_heap(const Graph& graph, int start, int goal, int max_length) 
{
    using PairingHeap = boost::heap::pairing_heap<pair<int, int>, boost::heap::compare<greater<>>>;
    vector<int> dist(graph.node_num, numeric_limits<int>::max());
    vector<int> heuristic(graph.node_num, 0);
    vector<PairingHeap::handle_type> handles(graph.node_num);
    PairingHeap pq;

    // Calculate heuristic
    for (const auto& node : graph.nodes) 
        heuristic[node.id] = ((max_length + 1) / 2) * abs(graph.nodes[goal].layer - node.layer);

    dist[start] = 0;
    for (int i = 0; i < graph.node_num; ++i) 
        handles[i] = pq.push({ dist[i], i });

    while (!pq.empty()) 
    {
        int u = pq.top().second;
        pq.pop();

        if (u == goal) 
            break;

        for (const auto& [v, length] : graph.adj_list[u]) 
        {
            if (dist[u] + length < dist[v]) 
            {
                dist[v] = dist[u] + length;
                pq.update(handles[v], { dist[v] + heuristic[v], v });
            }
        }
    }
    cout << "A* (Pairing Heap) distance from " << start << " to " << goal << ": " << dist[goal] << endl;
}

void a_star_fibonacci_heap(const Graph& graph, int start, int goal, int max_length) 
{
    using FibHeap = boost::heap::fibonacci_heap<pair<int, int>, boost::heap::compare<greater<>>>;
    vector<int> dist(graph.node_num, numeric_limits<int>::max());
    vector<int> heuristic(graph.node_num, 0);
    vector<FibHeap::handle_type> handles(graph.node_num);
    FibHeap pq;

    // Calculate heuristic
    for (const auto& node : graph.nodes) 
        heuristic[node.id] = ((max_length + 1) / 2) * abs(graph.nodes[goal].layer - node.layer);

    dist[start] = 0;
    for (int i = 0; i < graph.node_num; ++i) 
        handles[i] = pq.push({ dist[i], i });

    while (!pq.empty()) 
    {
        int u = pq.top().second;
        pq.pop();

        if (u == goal) 
            break;

        for (const auto& [v, length] : graph.adj_list[u]) 
        {
            if (dist[u] + length < dist[v]) 
            {
                dist[v] = dist[u] + length;
                pq.update(handles[v], { dist[v] + heuristic[v], v });
            }
        }
    }
    cout << "A* (Fibonacci Heap) distance from " << start << " to " << goal << ": " << dist[goal] << endl;
}

double monitor_time(const string& name, const function<void()>& func)
{
    auto start = chrono::high_resolution_clock::now();
    func();
    auto end = chrono::high_resolution_clock::now();
    chrono::duration<double, std::milli> elapsed = end - start;
    return elapsed.count();
}

int main() 
{
    int num_nodes = 10002;
    int nodes_per_layer = 50;
    int max_length = 1001;
    int num_iter = 10;
    Graph graph(num_nodes, nodes_per_layer);
    auto reset_graph = [&]() 
    {
        graph = Graph(num_nodes, nodes_per_layer);
        graph.connect_layers(max_length);
    };

    vector<pair<string, function<void()>>> functions = {
        /*{"Bellman Ford", [&]() { bellman_ford(graph, 0, num_nodes - 1); }},
        {"Dijkstra (Brute Force VE)", [&]() { dijkstra_brute_force_ve(graph, 0, num_nodes - 1); }},
        {"Dijkstra (Brute Force VV)", [&]() { dijkstra_brute_force_vv(graph, 0, num_nodes - 1); }},*/
        {"Dijkstra (Binary Heap)", [&]() { dijkstra_binary_heap(graph, 0, num_nodes - 1); }},
        {"Dijkstra (Ternary Heap)", [&]() { dijkstra_ternary_heap(graph, 0, num_nodes - 1); }},
        {"Dijkstra (Quaternary Heap)", [&]() { dijkstra_quaternary_heap(graph, 0, num_nodes - 1); }},
        {"Dijkstra (Denary Heap)", [&]() { dijkstra_denary_heap(graph, 0, num_nodes - 1); }},
        {"Dijkstra (Binomial Heap)", [&]() { dijkstra_binomial_heap(graph, 0, num_nodes - 1); }},
        {"Dijkstra (Skew Heap)", [&]() { dijkstra_skew_heap(graph, 0, num_nodes - 1); }},
        {"Dijkstra (Pairing Heap)", [&]() { dijkstra_pairing_heap(graph, 0, num_nodes - 1); }},
        {"Dijkstra (Fibonacci Heap)", [&]() { dijkstra_fibonacci_heap(graph, 0, num_nodes - 1); }},
        {"A* (Brute Force VE)", [&]() { a_star_brute_force_ve(graph, 0, num_nodes - 1, max_length); }},
        {"A* (Brute Force VV)", [&]() { a_star_brute_force_vv(graph, 0, num_nodes - 1, max_length); }},
        {"A* (Binary Heap)", [&]() { a_star_binary_heap(graph, 0, num_nodes - 1, max_length); }},
        {"A* (Ternary Heap)", [&]() { a_star_ternary_heap(graph, 0, num_nodes - 1, max_length); }},
        {"A* (Quaternary Heap)", [&]() { a_star_quaternary_heap(graph, 0, num_nodes - 1, max_length); }},
        {"A* (Denary Heap)", [&]() { a_star_denary_heap(graph, 0, num_nodes - 1, max_length); }},
        {"A* (Binomial Heap)", [&]() { a_star_binomial_heap(graph, 0, num_nodes - 1, max_length); }},
        {"A* (Skew Heap)", [&]() { a_star_skew_heap(graph, 0, num_nodes - 1, max_length); }},
        {"A* (Pairing Heap)", [&]() { a_star_pairing_heap(graph, 0, num_nodes - 1, max_length); }},
        {"A* (Fibonacci Heap)", [&]() { a_star_fibonacci_heap(graph, 0, num_nodes - 1, max_length); }}
    };
    vector<pair<string, double>> results;

    for (const auto& [name, func] : functions) 
    {
        vector<double> times;
        for (int i = 0; i < num_iter; ++i) 
        {
            reset_graph(); 
            double elapsed = monitor_time(name, func);
            times.push_back(elapsed);
        }
        double avg_time = accumulate(times.begin(), times.end(), 0.0) / times.size();
        results.emplace_back(name, avg_time);
    }
    cout << fixed << setprecision(6);
    cout << "Average execution times:\n";
    for (const auto& [name, avg_time] : results) 
        cout << name << ": " << avg_time << " milli-seconds\n";
    return 0;
}

