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

void bellman_ford(const Graph& graph, int start, int goal, bool print_result)
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
    if(print_result)
        cout << "Bellman-Ford distance from " << start << " to " << goal << ": " << dist[goal] << endl;
}

void dijkstra_brute_force_ve(const Graph& graph, int start, int goal, bool print_result)
{
    vector<int> dist(graph.node_num, numeric_limits<int>::max());
    vector<bool> visited(graph.node_num, false);
    dist[start] = 0;

    for (int i = 0; i < graph.node_num; ++i)
    {
        int u = -1;
        for (int j = 0; j < graph.node_num; ++j)
        {
            if (!visited[j] && (u == -1 || dist[j] < dist[u]))
            {
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
    if (print_result)
        cout << "Dijkstra (Brute Force VE) distance from " << start << " to " << goal << ": " << dist[goal] << endl;
}

void dijkstra_brute_force_vv(const Graph& graph, int start, int goal, bool print_result)
{
    vector<int> dist(graph.node_num, numeric_limits<int>::max());
    vector<bool> visited(graph.node_num, false);
    dist[start] = 0;

    for (int i = 0; i < graph.node_num; ++i)
    {
        int u = -1;
        for (int j = 0; j < graph.node_num; ++j)
        {
            if (!visited[j] && (u == -1 || dist[j] < dist[u]))
                u = j;
        }

        if (u == -1 || dist[u] == numeric_limits<int>::max())
            break;

        visited[u] = true;
        for (const auto& [v, length] : graph.adj_list[u])
        {
            if (dist[u] + length < dist[v])
                dist[v] = dist[u] + length;
        }
    }
    if (print_result)
        cout << "Dijkstra (Brute Force VV) distance from " << start << " to " << goal << ": " << dist[goal] << endl;
}

void dijkstra_binary_heap(const Graph& graph, int start, int goal, bool print_result)
{
    using BinaryHeap = boost::heap::d_ary_heap<pair<int, int>, boost::heap::arity<2>, boost::heap::mutable_<true>, boost::heap::compare<greater<>>>;
    vector<int> dist(graph.node_num, numeric_limits<int>::max());
    BinaryHeap pq;

    dist[start] = 0;
    pq.push({ dist[start], start });

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
                pq.push({ dist[v], v });
            }
        }
    }
    if (print_result)
        cout << "Dijkstra (Binary Heap) distance from " << start << " to " << goal << ": " << dist[goal] << endl;
}

void dijkstra_ternary_heap(const Graph& graph, int start, int goal, bool print_result)
{
    using TernaryHeap = boost::heap::d_ary_heap<pair<int, int>, boost::heap::arity<3>, boost::heap::mutable_<true>, boost::heap::compare<greater<>>>;
    vector<int> dist(graph.node_num, numeric_limits<int>::max());
    TernaryHeap pq;

    dist[start] = 0;
    pq.push({ dist[start], start });

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
                pq.push({ dist[v], v });
            }
        }
    }
    if (print_result)
        cout << "Dijkstra (Ternary Heap) distance from " << start << " to " << goal << ": " << dist[goal] << endl;
}

void dijkstra_quaternary_heap(const Graph& graph, int start, int goal, bool print_result)
{
    using QuaternaryHeap = boost::heap::d_ary_heap<pair<int, int>, boost::heap::arity<4>, boost::heap::mutable_<true>, boost::heap::compare<greater<>>>;
    vector<int> dist(graph.node_num, numeric_limits<int>::max());
    QuaternaryHeap pq;

    dist[start] = 0;
    pq.push({ dist[start], start });

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
                pq.push({ dist[v], v });
            }
        }
    }
    if (print_result)
        cout << "Dijkstra (Quaternary Heap) distance from " << start << " to " << goal << ": " << dist[goal] << endl;
}

void dijkstra_denary_heap(const Graph& graph, int start, int goal, bool print_result)
{
    using DenaryHeap = boost::heap::d_ary_heap<pair<int, int>, boost::heap::arity<10>, boost::heap::mutable_<true>, boost::heap::compare<greater<>>>;
    vector<int> dist(graph.node_num, numeric_limits<int>::max());
    DenaryHeap pq;

    dist[start] = 0;
    pq.push({ dist[start], start });

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
                pq.push({ dist[v], v });
            }
        }
    }
    if (print_result)
        cout << "Dijkstra (Denary Heap) distance from " << start << " to " << goal << ": " << dist[goal] << endl;
}

void dijkstra_binomial_heap(const Graph& graph, int start, int goal, bool print_result)
{
    using BinHeap = boost::heap::binomial_heap<pair<int, int>, boost::heap::compare<greater<>>>;
    vector<int> dist(graph.node_num, numeric_limits<int>::max());
    BinHeap pq;

    dist[start] = 0;
    pq.push({ dist[start], start });

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
                pq.push({ dist[v], v });
            }
        }
    }
    if (print_result)
        cout << "Dijkstra (Binomial Heap) distance from " << start << " to " << goal << ": " << dist[goal] << endl;
}

void dijkstra_pairing_heap(const Graph& graph, int start, int goal, bool print_result)
{
    using PairingHeap = boost::heap::pairing_heap<pair<int, int>, boost::heap::compare<greater<>>>;
    vector<int> dist(graph.node_num, numeric_limits<int>::max());
    PairingHeap pq;

    dist[start] = 0;
    pq.push({ dist[start], start });

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
                pq.push({ dist[v], v });
            }
        }
    }
    if (print_result)
        cout << "Dijkstra (Pairing Heap) distance from " << start << " to " << goal << ": " << dist[goal] << endl;
}

void dijkstra_skew_heap(const Graph& graph, int start, int goal, bool print_result)
{
    using SkewHeap = boost::heap::skew_heap<pair<int, int>, boost::heap::mutable_<true>, boost::heap::compare<greater<>>>;
    vector<int> dist(graph.node_num, numeric_limits<int>::max());
    SkewHeap pq;

    dist[start] = 0;
    pq.push({ dist[start], start });

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
                pq.push({ dist[v], v });
            }
        }
    }
    if (print_result)
        cout << "Dijkstra (Skew Heap) distance from " << start << " to " << goal << ": " << dist[goal] << endl;
}

void dijkstra_fibonacci_heap(const Graph& graph, int start, int goal, bool print_result)
{
    using FibHeap = boost::heap::fibonacci_heap<pair<int, int>, boost::heap::compare<greater<>>>;
    vector<int> dist(graph.node_num, numeric_limits<int>::max());
    FibHeap pq;

    dist[start] = 0;
    pq.push({ dist[start], start });

    while (!pq.empty())
    {
        int u = pq.top().second;
        pq.pop();

        for (const auto& [v, length] : graph.adj_list[u])
        {
            if (dist[u] + length < dist[v])
            {
                dist[v] = dist[u] + length;
                pq.push({ dist[v], v });
            }
        }
    }
    if (print_result)
        cout << "Dijkstra (Fibonacci Heap) distance from " << start << " to " << goal << ": " << dist[goal] << endl;
}

void a_star_brute_force_ve(const Graph& graph, int start, int goal, int max_length, bool print_result)
{
    vector<int> dist(graph.node_num, numeric_limits<int>::max());
    vector<bool> visited(graph.node_num, false);
    vector<int> heuristic(graph.node_num);

    for (const auto& node : graph.nodes)
        heuristic[node.id] = ((max_length + 1) / 2) * abs(graph.nodes[goal].layer - node.layer);

    dist[start] = 0;

    for (int i = 0; i < graph.node_num; ++i)
    {
        int u = -1;
        int min_f = numeric_limits<int>::max();
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
    if (print_result)
        cout << "A* (Brute Force VE) distance from " << start << " to " << goal << ": " << dist[goal] << endl;
}

void a_star_brute_force_vv(const Graph& graph, int start, int goal, int max_length, bool print_result)
{
    vector<int> dist(graph.node_num, numeric_limits<int>::max());
    vector<bool> visited(graph.node_num, false);
    vector<int> heuristic(graph.node_num);

    for (const auto& node : graph.nodes)
        heuristic[node.id] = ((max_length + 1) / 2) * abs(graph.nodes[goal].layer - node.layer);

    dist[start] = 0;

    for (int i = 0; i < graph.node_num; ++i)
    {
        int u = -1;
        int min_f = numeric_limits<int>::max();
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
        for (const auto& [v, length] : graph.adj_list[u])
        {
            if (!visited[v] && dist[u] + length < dist[v])
                dist[v] = dist[u] + length;
        }
    }
    if (print_result)
        cout << "A* (Brute Force VV) distance from " << start << " to " << goal << ": " << dist[goal] << endl;
}

void a_star_binary_heap(const Graph& graph, int start, int goal, int max_length, bool print_result)
{
    using BinaryHeap = boost::heap::d_ary_heap<pair<int, int>, boost::heap::arity<2>, boost::heap::mutable_<true>, boost::heap::compare<greater<>>>;
    vector<int> dist(graph.node_num, numeric_limits<int>::max());
    vector<int> heuristic(graph.node_num, 0);
    BinaryHeap pq;

    for (const auto& node : graph.nodes)
        heuristic[node.id] = ((max_length + 1) / 2) * abs(graph.nodes[goal].layer - node.layer);

    dist[start] = 0;
    pq.push({ dist[start], start });

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
                pq.push({ dist[v] + heuristic[v], v });
            }
        }
    }
    if (print_result)
        cout << "A* (Binary Heap) distance from " << start << " to " << goal << ": " << dist[goal] << endl;
}

void a_star_ternary_heap(const Graph& graph, int start, int goal, int max_length, bool print_result)
{
    using TernaryHeap = boost::heap::d_ary_heap<pair<int, int>, boost::heap::arity<2>, boost::heap::mutable_<true>, boost::heap::compare<greater<>>>;
    vector<int> dist(graph.node_num, numeric_limits<int>::max());
    vector<int> heuristic(graph.node_num, 0);
    TernaryHeap pq;

    for (const auto& node : graph.nodes)
        heuristic[node.id] = ((max_length + 1) / 2) * abs(graph.nodes[goal].layer - node.layer);

    dist[start] = 0;
    pq.push({ dist[start], start });

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
                pq.push({ dist[v] + heuristic[v], v });
            }
        }
    }
    if (print_result)
        cout << "A* (Ternary Heap) distance from " << start << " to " << goal << ": " << dist[goal] << endl;
}

void a_star_quaternary_heap(const Graph& graph, int start, int goal, int max_length, bool print_result)
{
    using QuaternaryHeap = boost::heap::d_ary_heap<pair<int, int>, boost::heap::arity<4>, boost::heap::mutable_<true>, boost::heap::compare<greater<>>>;
    vector<int> dist(graph.node_num, numeric_limits<int>::max());
    vector<int> heuristic(graph.node_num, 0);
    QuaternaryHeap pq;

    for (const auto& node : graph.nodes)
        heuristic[node.id] = ((max_length + 1) / 2) * abs(graph.nodes[goal].layer - node.layer);

    dist[start] = 0;
    pq.push({ dist[start], start });

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
                pq.push({ dist[v] + heuristic[v], v });
            }
        }
    }
    if (print_result)
        cout << "A* (Quaternary Heap) distance from " << start << " to " << goal << ": " << dist[goal] << endl;
}

void a_star_denary_heap(const Graph& graph, int start, int goal, int max_length, bool print_result)
{
    using DenaryHeap = boost::heap::d_ary_heap<pair<int, int>, boost::heap::arity<10>, boost::heap::mutable_<true>, boost::heap::compare<greater<>>>;
    vector<int> dist(graph.node_num, numeric_limits<int>::max());
    vector<int> heuristic(graph.node_num, 0);
    DenaryHeap pq;

    for (const auto& node : graph.nodes)
        heuristic[node.id] = ((max_length + 1) / 2) * abs(graph.nodes[goal].layer - node.layer);

    dist[start] = 0;
    pq.push({ dist[start], start });

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
                pq.push({ dist[v] + heuristic[v], v });
            }
        }
    }
    if (print_result)
        cout << "A* (Denary Heap) distance from " << start << " to " << goal << ": " << dist[goal] << endl;
}

void a_star_binomial_heap(const Graph& graph, int start, int goal, int max_length, bool print_result)
{
    using BinHeap = boost::heap::binomial_heap<pair<int, int>, boost::heap::compare<greater<>>>;
    vector<int> dist(graph.node_num, numeric_limits<int>::max());
    vector<int> heuristic(graph.node_num, 0);
    BinHeap pq;

    for (const auto& node : graph.nodes)
        heuristic[node.id] = ((max_length + 1) / 2) * abs(graph.nodes[goal].layer - node.layer);

    dist[start] = 0;
    pq.push({ dist[start], start });

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
                pq.push({ dist[v] + heuristic[v], v });
            }
        }
    }
    if (print_result)
        cout << "A* (Binomial Heap) distance from " << start << " to " << goal << ": " << dist[goal] << endl;
}

void a_star_pairing_heap(const Graph& graph, int start, int goal, int max_length, bool print_result)
{
    using PairingHeap = boost::heap::pairing_heap<pair<int, int>, boost::heap::compare<greater<>>>;
    vector<int> dist(graph.node_num, numeric_limits<int>::max());
    vector<int> heuristic(graph.node_num, 0);
    PairingHeap pq;

    for (const auto& node : graph.nodes)
        heuristic[node.id] = ((max_length + 1) / 2) * abs(graph.nodes[goal].layer - node.layer);

    dist[start] = 0;
    pq.push({ dist[start], start });

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
                pq.push({ dist[v] + heuristic[v], v });
            }
        }
    }
    if (print_result)
        cout << "A* (Pairing Heap) distance from " << start << " to " << goal << ": " << dist[goal] << endl;
}

void a_star_skew_heap(const Graph& graph, int start, int goal, int max_length, bool print_result)
{
    using SkewHeap = boost::heap::skew_heap<pair<int, int>, boost::heap::mutable_<true>, boost::heap::compare<greater<>>>;
    vector<int> dist(graph.node_num, numeric_limits<int>::max());
    vector<int> heuristic(graph.node_num, 0);
    SkewHeap pq;

    for (const auto& node : graph.nodes)
        heuristic[node.id] = ((max_length + 1) / 2) * abs(graph.nodes[goal].layer - node.layer);

    dist[start] = 0;
    pq.push({ dist[start], start });

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
                pq.push({ dist[v] + heuristic[v], v });
            }
        }
    }
    if (print_result)
        cout << "A* (Skew Heap) distance from " << start << " to " << goal << ": " << dist[goal] << endl;
}

void a_star_fibonacci_heap(const Graph& graph, int start, int goal, int max_length, bool print_result)
{
    using FibHeap = boost::heap::fibonacci_heap<pair<int, int>, boost::heap::compare<greater<>>>;
    vector<int> dist(graph.node_num, numeric_limits<int>::max());
    vector<int> heuristic(graph.node_num, 0);
    FibHeap pq;

    for (const auto& node : graph.nodes)
        heuristic[node.id] = ((max_length + 1) / 2) * abs(graph.nodes[goal].layer - node.layer);

    dist[start] = 0;
    pq.push({ dist[start], start });

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
                pq.push({ dist[v] + heuristic[v], v });
            }
        }
    }
    if (print_result)
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

void test_all(int num_nodes, int nodes_per_layer, int max_length, int num_iter, bool print_result)
{
    vector<Graph>graphs(num_iter, Graph(num_nodes, nodes_per_layer));
    for (auto& i : graphs)
        i.connect_layers(max_length);
    Graph graph(num_nodes, nodes_per_layer);
    vector<pair<string, function<void()>>> functions = {
        {"Bellman Ford", [&]() { bellman_ford(graph, 0, num_nodes - 1, print_result); }},
        {"Dijkstra (Brute Force VE)", [&]() { dijkstra_brute_force_ve(graph, 0, num_nodes - 1, print_result); }},
        {"Dijkstra (Brute Force VV)", [&]() { dijkstra_brute_force_vv(graph, 0, num_nodes - 1, print_result); }},
        {"Dijkstra (Binary Heap)", [&]() { dijkstra_binary_heap(graph, 0, num_nodes - 1, print_result); }},
        {"Dijkstra (Ternary Heap)", [&]() { dijkstra_ternary_heap(graph, 0, num_nodes - 1, print_result); }},
        {"Dijkstra (Quaternary Heap)", [&]() { dijkstra_quaternary_heap(graph, 0, num_nodes - 1, print_result); }},
        {"Dijkstra (Denary Heap)", [&]() { dijkstra_denary_heap(graph, 0, num_nodes - 1, print_result); }},
        {"Dijkstra (Binomial Heap)", [&]() { dijkstra_binomial_heap(graph, 0, num_nodes - 1, print_result); }},
        {"Dijkstra (Pairing Heap)", [&]() { dijkstra_pairing_heap(graph, 0, num_nodes - 1, print_result); }},
        {"Dijkstra (Skew Heap)", [&]() { dijkstra_skew_heap(graph, 0, num_nodes - 1, print_result); }},
        {"Dijkstra (Fibonacci Heap)", [&]() { dijkstra_fibonacci_heap(graph, 0, num_nodes - 1, print_result); }},
        {"A* (Brute Force VE)", [&]() { a_star_brute_force_ve(graph, 0, num_nodes - 1, max_length, print_result); }},
        {"A* (Brute Force VV)", [&]() { a_star_brute_force_vv(graph, 0, num_nodes - 1, max_length, print_result); }},
        {"A* (Binary Heap)", [&]() { a_star_binary_heap(graph, 0, num_nodes - 1, max_length, print_result); }},
        {"A* (Ternary Heap)", [&]() { a_star_ternary_heap(graph, 0, num_nodes - 1, max_length, print_result); }},
        {"A* (Quaternary Heap)", [&]() { a_star_quaternary_heap(graph, 0, num_nodes - 1, max_length, print_result); }},
        {"A* (Denary Heap)", [&]() { a_star_denary_heap(graph, 0, num_nodes - 1, max_length, print_result); }},
        {"A* (Binomial Heap)", [&]() { a_star_binomial_heap(graph, 0, num_nodes - 1, max_length, print_result); }},
        {"A* (Pairing Heap)", [&]() { a_star_pairing_heap(graph, 0, num_nodes - 1, max_length, print_result); }},
        {"A* (Skew Heap)", [&]() { a_star_skew_heap(graph, 0, num_nodes - 1, max_length, print_result); }},
        {"A* (Fibonacci Heap)", [&]() { a_star_fibonacci_heap(graph, 0, num_nodes - 1, max_length, print_result); }}
    };
    vector<pair<string, double>> results;
    for (const auto& [name, func] : functions)
    {
        vector<double> times;
        for (int i = 0; i < num_iter; ++i)
        {
            graph = graphs[i];
            double elapsed = monitor_time(name, func);
            times.push_back(elapsed);
        }
        double avg_time = accumulate(times.begin(), times.end(), 0.0) / times.size();
        results.emplace_back(name, avg_time);
    }
    cout << fixed << setprecision(6);
    cout << "Average execution times for num_nodes = " << num_nodes << ", nodes_per_layer = " << nodes_per_layer << endl << endl;
    for (const auto& [name, avg_time] : results)
        cout << name << ": " << avg_time << " milli-seconds" << endl;
    cout << endl;
}

void test_dijkstra(int num_nodes, int nodes_per_layer, int max_length, int num_iter, bool print_result)
{
    vector<Graph>graphs(num_iter, Graph(num_nodes, nodes_per_layer));
    for (auto& i : graphs)
        i.connect_layers(max_length);
    Graph graph(num_nodes, nodes_per_layer);
    vector<pair<string, function<void()>>> functions = {
        {"Dijkstra (Brute Force VE)", [&]() { dijkstra_brute_force_ve(graph, 0, num_nodes - 1, print_result); }},
        {"Dijkstra (Brute Force VV)", [&]() { dijkstra_brute_force_vv(graph, 0, num_nodes - 1, print_result); }},
        {"Dijkstra (Binary Heap)", [&]() { dijkstra_binary_heap(graph, 0, num_nodes - 1, print_result); }},
        {"Dijkstra (Ternary Heap)", [&]() { dijkstra_ternary_heap(graph, 0, num_nodes - 1, print_result); }},
        {"Dijkstra (Quaternary Heap)", [&]() { dijkstra_quaternary_heap(graph, 0, num_nodes - 1, print_result); }},
        {"Dijkstra (Denary Heap)", [&]() { dijkstra_denary_heap(graph, 0, num_nodes - 1, print_result); }},
        {"Dijkstra (Binomial Heap)", [&]() { dijkstra_binomial_heap(graph, 0, num_nodes - 1, print_result); }},
        {"Dijkstra (Pairing Heap)", [&]() { dijkstra_pairing_heap(graph, 0, num_nodes - 1, print_result); }},
        {"Dijkstra (Skew Heap)", [&]() { dijkstra_skew_heap(graph, 0, num_nodes - 1, print_result); }},
        {"Dijkstra (Fibonacci Heap)", [&]() { dijkstra_fibonacci_heap(graph, 0, num_nodes - 1, print_result); }}
    };
    vector<pair<string, double>> results;
    for (const auto& [name, func] : functions)
    {
        vector<double> times;
        for (int i = 0; i < num_iter; ++i)
        {
            graph = graphs[i];
            double elapsed = monitor_time(name, func);
            times.push_back(elapsed);
        }
        double avg_time = accumulate(times.begin(), times.end(), 0.0) / times.size();
        results.emplace_back(name, avg_time);
    }
    cout << fixed << setprecision(6);
    cout << "Average execution times for num_nodes = " << num_nodes << ", nodes_per_layer = " << nodes_per_layer << endl << endl;
    for (const auto& [name, avg_time] : results)
        cout << name << ": " << avg_time << " milli-seconds" << endl;
    cout << endl;
}

void test_a_star(int num_nodes, int nodes_per_layer, int max_length, int num_iter, bool print_result)
{
    vector<Graph>graphs(num_iter, Graph(num_nodes, nodes_per_layer));
    for (auto& i : graphs)
        i.connect_layers(max_length);
    Graph graph(num_nodes, nodes_per_layer);
    vector<pair<string, function<void()>>> functions = {
        {"A* (Brute Force VE)", [&]() { a_star_brute_force_ve(graph, 0, num_nodes - 1, max_length, print_result); }},
        {"A* (Brute Force VV)", [&]() { a_star_brute_force_vv(graph, 0, num_nodes - 1, max_length, print_result); }},
        {"A* (Binary Heap)", [&]() { a_star_binary_heap(graph, 0, num_nodes - 1, max_length, print_result); }},
        {"A* (Ternary Heap)", [&]() { a_star_ternary_heap(graph, 0, num_nodes - 1, max_length, print_result); }},
        {"A* (Quaternary Heap)", [&]() { a_star_quaternary_heap(graph, 0, num_nodes - 1, max_length, print_result); }},
        {"A* (Denary Heap)", [&]() { a_star_denary_heap(graph, 0, num_nodes - 1, max_length, print_result); }},
        {"A* (Binomial Heap)", [&]() { a_star_binomial_heap(graph, 0, num_nodes - 1, max_length, print_result); }},
        {"A* (Pairing Heap)", [&]() { a_star_pairing_heap(graph, 0, num_nodes - 1, max_length, print_result); }},
        {"A* (Skew Heap)", [&]() { a_star_skew_heap(graph, 0, num_nodes - 1, max_length, print_result); }},
        {"A* (Fibonacci Heap)", [&]() { a_star_fibonacci_heap(graph, 0, num_nodes - 1, max_length, print_result); }}
    };
    vector<pair<string, double>> results;
    for (const auto& [name, func] : functions)
    {
        vector<double> times;
        for (int i = 0; i < num_iter; ++i)
        {
            graph = graphs[i];
            double elapsed = monitor_time(name, func);
            times.push_back(elapsed);
        }
        double avg_time = accumulate(times.begin(), times.end(), 0.0) / times.size();
        results.emplace_back(name, avg_time);
    }
    cout << fixed << setprecision(6);
    cout << "Average execution times for num_nodes = " << num_nodes << ", nodes_per_layer = " << nodes_per_layer << endl << endl;
    for (const auto& [name, avg_time] : results)
        cout << name << ": " << avg_time << " milli-seconds" << endl;
    cout << endl;
}

void test_all_heap(int num_nodes, int nodes_per_layer, int max_length, int num_iter, bool print_result)
{
    vector<Graph>graphs(num_iter, Graph(num_nodes, nodes_per_layer));
    for (auto& i : graphs)
        i.connect_layers(max_length);
    Graph graph(num_nodes, nodes_per_layer);
    vector<pair<string, function<void()>>> functions = {
        {"Dijkstra (Binary Heap)", [&]() { dijkstra_binary_heap(graph, 0, num_nodes - 1, print_result); }},
        {"Dijkstra (Ternary Heap)", [&]() { dijkstra_ternary_heap(graph, 0, num_nodes - 1, print_result); }},
        {"Dijkstra (Quaternary Heap)", [&]() { dijkstra_quaternary_heap(graph, 0, num_nodes - 1, print_result); }},
        {"Dijkstra (Denary Heap)", [&]() { dijkstra_denary_heap(graph, 0, num_nodes - 1, print_result); }},
        {"Dijkstra (Binomial Heap)", [&]() { dijkstra_binomial_heap(graph, 0, num_nodes - 1, print_result); }},
        {"Dijkstra (Pairing Heap)", [&]() { dijkstra_pairing_heap(graph, 0, num_nodes - 1, print_result); }},
        {"Dijkstra (Skew Heap)", [&]() { dijkstra_skew_heap(graph, 0, num_nodes - 1, print_result); }},
        {"Dijkstra (Fibonacci Heap)", [&]() { dijkstra_fibonacci_heap(graph, 0, num_nodes - 1, print_result); }},
        {"A* (Binary Heap)", [&]() { a_star_binary_heap(graph, 0, num_nodes - 1, max_length, print_result); }},
        {"A* (Ternary Heap)", [&]() { a_star_ternary_heap(graph, 0, num_nodes - 1, max_length, print_result); }},
        {"A* (Quaternary Heap)", [&]() { a_star_quaternary_heap(graph, 0, num_nodes - 1, max_length, print_result); }},
        {"A* (Denary Heap)", [&]() { a_star_denary_heap(graph, 0, num_nodes - 1, max_length, print_result); }},
        {"A* (Binomial Heap)", [&]() { a_star_binomial_heap(graph, 0, num_nodes - 1, max_length, print_result); }},
        {"A* (Pairing Heap)", [&]() { a_star_pairing_heap(graph, 0, num_nodes - 1, max_length, print_result); }},
        {"A* (Skew Heap)", [&]() { a_star_skew_heap(graph, 0, num_nodes - 1, max_length, print_result); }},
        {"A* (Fibonacci Heap)", [&]() { a_star_fibonacci_heap(graph, 0, num_nodes - 1, max_length, print_result); }}
    };
    vector<pair<string, double>> results;
    for (const auto& [name, func] : functions)
    {
        vector<double> times;
        for (int i = 0; i < num_iter; ++i)
        {
            graph = graphs[i];
            double elapsed = monitor_time(name, func);
            times.push_back(elapsed);
        }
        double avg_time = accumulate(times.begin(), times.end(), 0.0) / times.size();
        results.emplace_back(name, avg_time);
    }
    cout << fixed << setprecision(6);
    cout << "Average execution times for num_nodes = " << num_nodes << ", nodes_per_layer = " << nodes_per_layer << endl << endl;
    for (const auto& [name, avg_time] : results)
        cout << name << ": " << avg_time << " milli-seconds" << endl;
    cout << endl;
}

void test_all_special_heap(int num_nodes, int nodes_per_layer, int max_length, int num_iter, bool print_result)
{
    vector<Graph>graphs(num_iter, Graph(num_nodes, nodes_per_layer));
    for (auto& i : graphs)
        i.connect_layers(max_length);
    Graph graph(num_nodes, nodes_per_layer);
    vector<pair<string, function<void()>>> functions = {
        {"Dijkstra (Binomial Heap)", [&]() { dijkstra_binomial_heap(graph, 0, num_nodes - 1, print_result); }},
        {"Dijkstra (Pairing Heap)", [&]() { dijkstra_pairing_heap(graph, 0, num_nodes - 1, print_result); }},
        {"Dijkstra (Skew Heap)", [&]() { dijkstra_skew_heap(graph, 0, num_nodes - 1, print_result); }},
        {"Dijkstra (Fibonacci Heap)", [&]() { dijkstra_fibonacci_heap(graph, 0, num_nodes - 1, print_result); }},
        {"A* (Binomial Heap)", [&]() { a_star_binomial_heap(graph, 0, num_nodes - 1, max_length, print_result); }},
        {"A* (Pairing Heap)", [&]() { a_star_pairing_heap(graph, 0, num_nodes - 1, max_length, print_result); }},
        {"A* (Skew Heap)", [&]() { a_star_skew_heap(graph, 0, num_nodes - 1, max_length, print_result); }},
        {"A* (Fibonacci Heap)", [&]() { a_star_fibonacci_heap(graph, 0, num_nodes - 1, max_length, print_result); }}
    };
    vector<pair<string, double>> results;
    for (const auto& [name, func] : functions)
    {
        vector<double> times;
        for (int i = 0; i < num_iter; ++i)
        {
            graph = graphs[i];
            double elapsed = monitor_time(name, func);
            times.push_back(elapsed);
        }
        double avg_time = accumulate(times.begin(), times.end(), 0.0) / times.size();
        results.emplace_back(name, avg_time);
    }
    cout << fixed << setprecision(6);
    cout << "Average execution times for num_nodes = " << num_nodes << ", nodes_per_layer = " << nodes_per_layer << endl << endl;
    for (const auto& [name, avg_time] : results)
        cout << name << ": " << avg_time << " milli-seconds" << endl;
    cout << endl;
}

void test_dijkstra_special_heap(int num_nodes, int nodes_per_layer, int max_length, int num_iter, bool print_result)
{
    vector<Graph>graphs(num_iter, Graph(num_nodes, nodes_per_layer));
    for (auto& i : graphs)
        i.connect_layers(max_length);
    Graph graph(num_nodes, nodes_per_layer);
    vector<pair<string, function<void()>>> functions = {
        {"Dijkstra (Binomial Heap)", [&]() { dijkstra_binomial_heap(graph, 0, num_nodes - 1, print_result); }},
        {"Dijkstra (Pairing Heap)", [&]() { dijkstra_pairing_heap(graph, 0, num_nodes - 1, print_result); }},
        {"Dijkstra (Skew Heap)", [&]() { dijkstra_skew_heap(graph, 0, num_nodes - 1, print_result); }},
        {"Dijkstra (Fibonacci Heap)", [&]() { dijkstra_fibonacci_heap(graph, 0, num_nodes - 1, print_result); }}
    };
    vector<pair<string, double>> results;
    for (const auto& [name, func] : functions)
    {
        vector<double> times;
        for (int i = 0; i < num_iter; ++i)
        {
            graph = graphs[i];
            double elapsed = monitor_time(name, func);
            times.push_back(elapsed);
        }
        double avg_time = accumulate(times.begin(), times.end(), 0.0) / times.size();
        results.emplace_back(name, avg_time);
    }
    cout << fixed << setprecision(6);
    cout << "Average execution times for num_nodes = " << num_nodes << ", nodes_per_layer = " << nodes_per_layer << endl << endl;
    for (const auto& [name, avg_time] : results)
        cout << name << ": " << avg_time << " milli-seconds" << endl;
    cout << endl;
}

void test_a_star_special_heap(int num_nodes, int nodes_per_layer, int max_length, int num_iter, bool print_result)
{
    vector<Graph>graphs(num_iter, Graph(num_nodes, nodes_per_layer));
    for (auto& i : graphs)
        i.connect_layers(max_length);
    Graph graph(num_nodes, nodes_per_layer);
    vector<pair<string, function<void()>>> functions = {
        {"A* (Binomial Heap)", [&]() { a_star_binomial_heap(graph, 0, num_nodes - 1, max_length, print_result); }},
        {"A* (Pairing Heap)", [&]() { a_star_pairing_heap(graph, 0, num_nodes - 1, max_length, print_result); }},
        {"A* (Skew Heap)", [&]() { a_star_skew_heap(graph, 0, num_nodes - 1, max_length, print_result); }},
        {"A* (Fibonacci Heap)", [&]() { a_star_fibonacci_heap(graph, 0, num_nodes - 1, max_length, print_result); }}
    };
    vector<pair<string, double>> results;
    for (const auto& [name, func] : functions)
    {
        vector<double> times;
        for (int i = 0; i < num_iter; ++i)
        {
            graph = graphs[i];
            double elapsed = monitor_time(name, func);
            times.push_back(elapsed);
        }
        double avg_time = accumulate(times.begin(), times.end(), 0.0) / times.size();
        results.emplace_back(name, avg_time);
    }
    cout << fixed << setprecision(6);
    cout << "Average execution times for num_nodes = " << num_nodes << ", nodes_per_layer = " << nodes_per_layer << endl << endl;
    for (const auto& [name, avg_time] : results)
        cout << name << ": " << avg_time << " milli-seconds" << endl;
    cout << endl;
}

int main()
{
    test_all(102, 10, 1001, 10, false);
    test_all(502, 20, 1001, 10, false);
    test_all(1002, 50, 1001, 10, false);
    test_all(5002, 100, 1001, 5, false);
    test_all_special_heap(10002, 200, 1001, 1, false);
    test_a_star_special_heap(50002, 500, 1001, 1, false);
    test_a_star_special_heap(100002, 500, 1001, 1, false);
    return 0;
}
