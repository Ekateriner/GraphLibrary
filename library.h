#pragma once

#include <vector>
#include <list>
#include <queue>
#include <stack>
#include <map>
#include <exception>

class CompareException : public std::exception {
    const char* what() const noexcept override {
        return "DFS and BFS iterators couldn't be compared.";
    }
};

class EndException : public std::exception {
    const char* what() const noexcept override {
        return "Undefined behavior. This iterator is end of DFS or BFS.";
    }
};


template <typename V, typename W>
class Graph {

    class VertIterator : std::iterator<std::input_iterator_tag, V> {
    public:
        VertIterator(const VertIterator &it) :
                pointer(it.pointer) {}

        bool operator!=(VertIterator const &other) const {
            return pointer != other.pointer;
        }

        bool operator==(VertIterator const &other) const {
            return pointer == other.pointer;
        }

        virtual VertIterator &operator++() {
            pointer++;
            return *this;
        }

        virtual typename VertIterator::reference operator*() const {
            return *pointer;
        }

    private:
        explicit VertIterator(V *_pointer) :
                pointer(_pointer) {}

        V *pointer;
    };

    class EdgeIterator : std::iterator<std::input_iterator_tag, std::pair<std::pair<V, V>, W>> {
    public:
        struct Edge {
            V from;
            V to;
            W weight;

            Edge() = default;

            Edge(V _from, V _to, W _weight = NULL) :
                    from(_from),
                    to(_to),
                    weight(_weight) {}
        };

        EdgeIterator(const EdgeIterator &it) :
                pointer(it.pointer) {}

        bool operator!=(EdgeIterator const &other) const {
            return pointer != other.pointer;
        }

        bool operator==(EdgeIterator const &other) const {
            return pointer == other.pointer;
        }

        typename EdgeIterator::reference operator*() const {
            return *pointer;
        }

        virtual EdgeIterator &operator++() = 0;

    private:
        explicit EdgeIterator(Edge *_pointer) :
                pointer(_pointer) {}

        explicit EdgeIterator(V _from, V _to, W _weight = NULL) :
                pointer(new Edge(_from, _to, _weight)) {}


        Edge *pointer;
    };
};

//======================================================================================================================

template <typename V, typename W>
class MatrixGraph : public Graph<V, W>{
public:
    MatrixGraph() = default;

    MatrixGraph(std::vector<V>& _vertices, std::vector<std::vector<std::pair<bool, W>>>& _edges):
        vertices(_vertices),
        matrix_edges(_edges) {}

    MatrixGraph(MatrixGraph<V, W>& other):
        vertices{other.vertices},
        matrix_edges(other.matrix_edges) {}

    MatrixGraph<V, W>& operator=(const MatrixGraph<V, W>& other) {
        vertices = other.vertices;
        matrix_edges = other.matrix_edges;
        return *this;
    }

    MatrixGraph<V, W>& operator=(MatrixGraph<V, W>&& other) noexcept {
        vertices = other.vertices;
        other.vertices.clear();
        matrix_edges = other.matrix_edges;
        other.matrix_edges.clear();
        return *this;
    }

    MatrixGraph<V, W>& operator=(MatrixGraph<V, W> other) noexcept {
        vertices = other.vertices;
        other.vertices.clear();
        matrix_edges = other.matrix_edges;
        other.matrix_edges.clear();
        return *this;
    }

    void add_vertex(V& new_vertex) {
        vertices.push_back(new_vertex);
        std::pair<bool, W> null_edge = std::make_pair(false, NULL);
        for(int i = 0; i < matrix_edges.size(); i++){
            matrix_edges[i].push_back(null_edge);
        }
        matrix_edges.push_back(std::vector<std::pair<bool, W>>(vertices.size(), null_edge));
    }

    void add_edge(int from, int to, W& weight=NULL, bool change = true) { //change if exist
        if (!change && matrix_edges[from][to].first)
            return ;
        matrix_edges[from][to] = std::make_pair(true, weight);
    }

    size_t vertices_count() {
        return vertices.size();
    }

    std::list<std::pair<int, W>>& next_vertices(int current_vertex){
        std::list<std::pair<int, W>> next_vertices;
        for (size_t to = 0; to < vertices_count(); to++) {
            if (matrix_edges[current_vertex][to].first)
                next_vertices.push_back(std::make_pair(to, matrix_edges[current_vertex][to].second));
        }
        return next_vertices;
    }

    //-----------------------iterators--------------------------------

    class MatrixVertIterator : public Graph<V, W>::VertIterator {
    private:
        explicit MatrixVertIterator(V* _pointer):
                pointer(_pointer) {}

        V* pointer;
    };

    MatrixVertIterator& vertex_begin() {
        return MatrixVertIterator(vertices.begin());
    }

    MatrixVertIterator& vertex_end() {
        return MatrixVertIterator(vertices.end());
    }

    class DFS_MatrixVertIterator : public Graph<V, W>::VertIterator {
    public:
        DFS_MatrixVertIterator(const DFS_MatrixVertIterator &it):
                pointer(it.pointer),
                index(it.index),
                stack(it.stack),
                visited(it.visited),
                graph(it.graph) {}

        bool operator!=(DFS_MatrixVertIterator const& other) const {
            throw CompareException();
        }

        bool operator==(DFS_MatrixVertIterator const& other) const {
            throw CompareException();
        }

        typename DFS_MatrixVertIterator::reference operator*() const {
            return *pointer;
        }

        DFS_MatrixVertIterator& operator++() override {
            std::list<std::pair<int, W>> next = graph->next_vertices(index);
            // add children in stack
            for (auto it = next.begin(); it != next.end(); it++) {
                if (!visited[it->first]) {
                    stack.push(it->first);
                }
            }

            // delete extra
            while(!stack.empty() && visited[stack.top()])
                stack.pop();

            // get next
            if(!stack.empty()) {
                index = stack.top();
                stack.pop();
                visited[index] = true;
                pointer = graph->vertices.begin() + index;
                return *this;
            }
            else {
                for(int i = 0; i < visited.size(); i++) {
                    if(!visited[i]) {
                        index = i;
                        visited[index] = true;
                        pointer = graph->vertices.begin() + index;
                        return *this;
                    }
                }

                throw EndException();
            }
        }

        bool is_end() {
            bool visited_all = true;
            for (auto && i : visited)
                visited_all = (visited_all and i);
            return visited_all;
        }

    private:
        explicit DFS_MatrixVertIterator(V* _pointer, int _index, MatrixGraph<V, W>* _graph):
                pointer(_pointer),
                index(_index),
                stack(std::stack<int>()),
                visited(std::vector<bool>(_graph->vertices_count(), false)),
                graph(_graph) {
            visited[_index] = true;
        }

        V* pointer;
        int index;
        std::stack<int> stack;
        std::vector<bool> visited;
        const MatrixGraph<V, W>* graph;
    };

    DFS_MatrixVertIterator& dfs_begin(int index = 0) {
        return DFS_MatrixVertIterator(vertices.begin() + index, index, this);
    }

    class BFS_MatrixVertIterator : public Graph<V, W>::VertIterator {
    public:
        BFS_MatrixVertIterator(const BFS_MatrixVertIterator &it):
                pointer(it.pointer),
                index(it.index),
                queue(it.queue),
                visited(it.visited),
                graph(it.graph) {}

        bool operator!=(BFS_MatrixVertIterator const& other) const {
            throw CompareException();
        }

        bool operator==(BFS_MatrixVertIterator const& other) const {
            throw CompareException();
        }

        typename BFS_MatrixVertIterator::reference operator*() const {
            return *pointer;
        }

        BFS_MatrixVertIterator& operator++() override {
            std::list<std::pair<int, W>> next = graph->next_vertices(index);
            // add children in queue
            for (auto it = next.begin(); it != next.end(); it++) {
                if (!visited[it->first]) {
                    queue.push(it->first);
                }
            }

            // delete extra
            while(!queue.empty() && visited[queue.front()])
                queue.pop();

            // get next
            if(!queue.empty()) {
                index = queue.front();
                queue.pop();
                visited[index] = true;
                pointer = graph->vertices.begin() + index;
                return *this;
            }
            else {
                for(int i = 0; i < visited.size(); i++) {
                    if(!visited[i]) {
                        index = i;
                        visited[index] = true;
                        pointer = graph->vertices.begin() + index;
                        return *this;
                    }
                }

                throw EndException();
            }
        }

        bool is_end() {
            bool visited_all = true;
            for (auto && i : visited)
                visited_all = (visited_all and i);

            return visited_all;
        }

    private:
        explicit BFS_MatrixVertIterator(V* _pointer, int _index, MatrixGraph<V, W>* _graph):
                pointer(_pointer),
                index(_index),
                queue(std::queue<int>()),
                visited(std::vector<bool>(_graph->vertices_count(), false)),
                graph(_graph) {
            visited[_index] = true;
        }

        V* pointer;
        int index;
        std::queue<int> queue;
        std::vector<bool> visited;
        const MatrixGraph<V, W>* graph;
    };

    BFS_MatrixVertIterator& bfs_begin(int index = 0) {
        return BFS_MatrixVertIterator(vertices.begin() + index, index, this);
    }

private:
    std::vector<V> vertices;
    std::vector<std::vector<std::pair<bool, W>>> matrix_edges;
};

//======================================================================================================================

template <typename V, typename W>
class ListGraph : public Graph<V, W>{
public:
    ListGraph() = default;

    ListGraph(std::vector<V>& _vertices, std::vector<std::vector<std::pair<bool, W>>>& _edges, bool _multi_edges = false):
        vertices(_vertices),
        list_edges(_edges) {}

    ListGraph(ListGraph<V, W>& other):
        vertices(other.vertices),
        list_edges(other.list_edges) {}

    ListGraph<V, W>& operator=(const ListGraph<V, W>& other){
        vertices = other.vertices;
        list_edges = other.list_edges;
        return *this;
    }

    ListGraph<V, W>& operator=(ListGraph<V, W>&& other)  noexcept{
        vertices = other.vertices;
        other.vertices.clear();
        list_edges = other.list_edges;
        other.list_edges.clear();
        return *this;
    }

    ListGraph<V, W>& operator=(ListGraph<V, W> other)  noexcept{
        vertices = other.vertices;
        other.vertices.clear();
        list_edges = other.list_edges;
        other.list_edges.clear();
        return *this;
    }

    /*explicit operator MatrixGraph<V, W>() const{
        size_t n = vertices.size();
        std::pair<bool, W> null_edge = std::make_pair(false, NULL);
        std::vector<std::vector<std::pair<bool, W>>> matrix_edges(n, std::vector<std::pair<bool, W>>(n, null_edge));
        MatrixGraph<V, W> graph(vertices, matrix_edges);
        for (int from = 0; from < n; from++){
            for (auto it = list_edges[from].begin(); it != list_edges[from].end(); it++){
                graph.add_edge(from, it->first, it->second);
            }
        }
        return graph;
    }*/

    void add_vertex(V& new_vertex) {
        vertices.push_back(new_vertex);
        list_edges.push_back(std::list<std::pair<int, W>>());
    }

    void add_edge(int from, int to, W& weight=NULL, bool change=true) { // change only in single edge case
        for(auto it=list_edges[from].begin(); it != list_edges[from].end(); it++) {
            if (it->first == to) {
                if (change)
                    list_edges.insert(it, std::make_pair(to, weight));
                return;
            }
        }
        list_edges[from].push_back(std::make_pair(to, weight));
    }

    size_t vertices_count() {
        return vertices.size();
    }

    std::list<std::pair<int, W>>& next_vertices(int current_vertex){
        return list_edges[current_vertex];
    }

    //-----------------------iterators--------------------------------

    class ListVertIterator : public Graph<V, W>::VertIterator {
    private:
        explicit ListVertIterator(V* _pointer):
                pointer(_pointer) {}

        V* pointer;
    };

    ListVertIterator& vertex_begin() {
        return ListVertIterator(vertices.begin());
    }

    ListVertIterator& vertex_end() {
        return ListVertIterator(vertices.end());
    }

    class DFS_ListVertIterator : public Graph<V, W>::VertIterator {
    public:
        DFS_ListVertIterator(const DFS_ListVertIterator &it):
                pointer(it.pointer),
                index(it.index),
                stack(it.stack),
                visited(it.visited),
                graph(it.graph) {}

        bool operator!=(DFS_ListVertIterator const& other) const {
            throw CompareException();
        }

        bool operator==(DFS_ListVertIterator const& other) const {
            throw CompareException();
        }

        typename DFS_ListVertIterator::reference operator*() const {
            return *pointer;
        }

        DFS_ListVertIterator& operator++() override {
            std::list<std::pair<int, W>> next = graph->next_vertices(index);
            // add children in stack
            for (auto it = next.begin(); it != next.end(); it++) {
                if (!visited[it->first]) {
                    stack.push(it->first);
                }
            }

            // delete extra
            while(!stack.empty() && visited[stack.top()])
                stack.pop();

            // get next
            if(!stack.empty()) {
                index = stack.top();
                stack.pop();
                visited[index] = true;
                pointer = graph->vertices.begin() + index;
                return *this;
            }
            else {
                for(int i = 0; i < visited.size(); i++) {
                    if(!visited[i]) {
                        index = i;
                        visited[index] = true;
                        pointer = graph->vertices.begin() + index;
                        return *this;
                    }
                }

                throw EndException();
            }
        }

        bool is_end() {
            bool visited_all = true;
            for (auto && i : visited)
                visited_all = (visited_all and i);

            return visited_all;
        }

    private:
        explicit DFS_ListVertIterator(V* _pointer, int _index, ListGraph<V, W>* _graph):
                pointer(_pointer),
                index(_index),
                stack(std::stack<int>()),
                visited(std::vector<bool>(_graph->vertices_count(), false)),
                graph(_graph) {
            visited[_index] = true;
        }

        V* pointer;
        int index;
        std::stack<int> stack;
        std::vector<bool> visited;
        const ListGraph<V, W>* graph;
    };

    DFS_ListVertIterator& dfs_begin(int index = 0) {
        return DFS_ListVertIterator(vertices.begin() + index, index, this);
    }

    class BFS_ListVertIterator : public Graph<V, W>::VertIterator {
    public:
        BFS_ListVertIterator(const BFS_ListVertIterator &it):
                pointer(it.pointer),
                index(it.index),
                queue(it.queue),
                visited(it.visited),
                graph(it.graph) {}

        bool operator!=(BFS_ListVertIterator const& other) const {
            throw CompareException();
        }

        bool operator==(BFS_ListVertIterator const& other) const {
            throw CompareException();
        }

        typename BFS_ListVertIterator::reference operator*() const {
            return *pointer;
        }

        BFS_ListVertIterator& operator++() override {
            std::list<std::pair<int, W>> next = graph->next_vertices(index);
            // add children in queue
            for (auto it = next.begin(); it != next.end(); it++) {
                if (!visited[it->first]) {
                    queue.push(it->first);
                }
            }

            // delete extra
            while(!queue.empty() && visited[queue.front()])
                queue.pop();

            // get next
            if(!queue.empty()) {
                index = queue.front();
                queue.pop();
                visited[index] = true;
                pointer = graph->vertices.begin() + index;
                return *this;
            }
            else {
                for(int i = 0; i < visited.size(); i++) {
                    if(!visited[i]) {
                        index = i;
                        visited[index] = true;
                        pointer = graph->vertices.begin() + index;
                        return *this;
                    }
                }

                throw EndException();
            }
        }

        bool is_end() {
            bool visited_all = true;
            for (auto && i : visited)
                visited_all = (visited_all and i);

            return visited_all;
        }

    private:
        explicit BFS_ListVertIterator(V* _pointer, int _index, ListGraph<V, W>* _graph):
                pointer(_pointer),
                index(_index),
                queue(std::queue<int>()),
                visited(std::vector<bool>(_graph->vertices_count(), false)),
                graph(_graph) {
            visited[_index] = true;
        }

        V* pointer;
        int index;
        std::queue<int> queue;
        std::vector<bool> visited;
        const ListGraph<V, W>* graph;
    };

    BFS_ListVertIterator& bfs_begin(int index = 0) {
        return BFS_ListVertIterator(vertices.begin() + index, index, this);
    }

private:
    std::vector<V> vertices;
    std::vector<std::list<std::pair<int, W>>> list_edges;
};

//======================================================================================================================

template <typename V, typename W>
class NodeGraph : public Graph<V, W>{
public:
    struct Vertex{
        V object;
        std::list<std::pair<Vertex*, W>> next_vertices;

        Vertex() = default;

        explicit Vertex(V& vertex):
            object(vertex),
            next_vertices(std::list<std::pair<Vertex*, W>>()) {}

        Vertex(V& vertex, std::list<std::pair<Vertex*, W>>& next):
            object(vertex),
            next_vertices(next) {}

        void add_edge(Vertex* to_vertex, W& weight=NULL, bool change = true) {
            for(auto it=next_vertices.begin(); it != next_vertices.end(); it++) {
                if (it->first == to_vertex) {
                    if (change)
                        next_vertices.insert(it, std::make_pair(to_vertex, weight));
                    return;
                }
            }
            next_vertices.push_back(std::make_pair(to_vertex, weight));
        }
    };

    NodeGraph() = default;

    NodeGraph(NodeGraph<V, W>& other):
        vertices(other.vertices) {}

    NodeGraph<V, W>& operator=(const NodeGraph<V, W>& other){
        vertices = other.vertices;
        return *this;
    }

    NodeGraph<V, W>& operator=(NodeGraph<V, W>&& other) noexcept{
        vertices = other.vertices;
        other.vertices.clear();
        return *this;
    }

    NodeGraph<V, W>& operator=(NodeGraph<V, W> other) noexcept{
        vertices = other.vertices;
        other.vertices.clear();
        return *this;
    }

    void add_vertex(Vertex* new_vertex) {
        vertices.push_back(new_vertex);
    }

    void add_edge(Vertex* from, Vertex* to, W& weight=NULL, bool change=true) {
        for(auto it=vertices.begin(); it != vertices.end(); it++) {
            if (*it == from) {
                (*it)->add_edge(to, weight, change);
                return;
            }
        }
    }

    size_t vertices_count() {
        return vertices.size();
    }

    std::list<std::pair<Vertex*, W>>& next_vertices(Vertex* current_vertex){
        return current_vertex->next_vertices;
    }

    std::list<std::pair<Vertex*, W>>& next_vertices(int current_vertex){
        return vertices[current_vertex]->next_vertices;
    }

    //-----------------------iterators--------------------------------

    class NodeVertIterator : public Graph<V, W>::VertIterator {
    public:
        typename NodeVertIterator::reference operator*() const {
            return pointer->object;
        }
    private:
        explicit NodeVertIterator(Vertex* _pointer):
                pointer(_pointer) {}

        Vertex* pointer;
    };

    NodeVertIterator& vertex_begin() {
        return NodeVertIterator(vertices.begin());
    }

    NodeVertIterator& vertex_end() {
        return NodeVertIterator(vertices.end());
    }

    class DFS_NodeVertIterator : public Graph<V, W>::VertIterator {
    public:
        DFS_NodeVertIterator(const DFS_NodeVertIterator &it):
                pointer(it.pointer),
                stack(it.stack),
                visited(it.visited),
                graph(it.graph) {}

        bool operator!=(DFS_NodeVertIterator const& other) const {
            throw CompareException();
        }

        bool operator==(DFS_NodeVertIterator const& other) const {
            throw CompareException();
        }

        typename DFS_NodeVertIterator::reference operator*() const {
            return pointer->object;
        }

        DFS_NodeVertIterator& operator++() override {
            std::list<std::pair<Vertex*, W>> next = graph->next_vertices(pointer);
            // add children in stack
            for (auto it = next.begin(); it != next.end(); it++) {
                if (!visited[it->first]) {
                    stack.push(it->first);
                }
            }

            // delete extra
            while(!stack.empty() && visited[stack.top()])
                stack.pop();

            // get next
            if(!stack.empty()) {
                pointer = stack.top();
                stack.pop();
                visited[pointer] = true;
                return *this;
            }
            else {
                for(auto it = visited.begin(); it != visited.end(); it++) {
                    if(!it->second) {
                        pointer = it->first;
                        visited[pointer] = true;
                        return *this;
                    }
                }

                throw EndException();
            }
        }

        bool is_end() {
            bool visited_all = true;
            for (auto it = visited.begin(); it != visited.end(); it++)
                visited_all = (visited_all and it->second);

            return visited_all;
        }

    private:
        explicit DFS_NodeVertIterator(Vertex* _pointer, NodeGraph<V, W>* _graph):
                pointer(_pointer),
                stack(std::stack<Vertex*>()),
                visited(std::map<Vertex*, bool>()),
                graph(_graph) {
            for (auto it = _graph->vertices.begin(); it != _graph->vertices.end(); it++)
                visited[*it] = false;
            visited[_pointer] = true;
        }

        Vertex* pointer;
        std::stack<Vertex*> stack;
        std::map<Vertex*, bool> visited;
        const NodeGraph<V, W>* graph;
    };

    DFS_NodeVertIterator& dfs_begin(int index = 0) {
        return DFS_NodeVertIterator(vertices.begin() + index, this);
    }

    DFS_NodeVertIterator& dfs_begin(Vertex* start) {
        return DFS_NodeVertIterator(start, this);
    }

    class BFS_NodeVertIterator : public Graph<V, W>::VertIterator {
    public:
        BFS_NodeVertIterator(const BFS_NodeVertIterator &it):
                pointer(it.pointer),
                queue(it.queue),
                visited(it.visited),
                graph(it.graph) {}

        bool operator!=(BFS_NodeVertIterator const& other) const {
            throw CompareException();
        }

        bool operator==(BFS_NodeVertIterator const& other) const {
            throw CompareException();
        }

        typename BFS_NodeVertIterator::reference operator*() const {
            return pointer->object;
        }

        BFS_NodeVertIterator& operator++() override {
            std::list<std::pair<Vertex*, W>> next = graph->next_vertices(pointer);
            // add children in queue
            for (auto it = next.begin(); it != next.end(); it++) {
                if (!visited[it->first]) {
                    queue.push(it->first);
                }
            }

            // delete extra
            while(!queue.empty() && visited[queue.front()])
                queue.pop();

            // get next
            if(!queue.empty()) {
                pointer = queue.front();
                queue.pop();
                visited[pointer] = true;
                return *this;
            }
            else {
                for(auto it = visited.begin(); it != visited.end(); it++) {
                    if(!it->second) {
                        pointer = it->first;
                        visited[pointer] = true;
                        return *this;
                    }
                }

                throw EndException();
            }
        }

        bool is_end() {
            bool visited_all = true;
            for (auto it = visited.begin(); it != visited.end(); it++)
                visited_all = (visited_all and it->second);

            return visited_all;
        }

    private:
        explicit BFS_NodeVertIterator(Vertex* _pointer, NodeGraph<V, W>* _graph):
        pointer(_pointer),
        queue(std::queue<Vertex*>()),
        visited(std::map<Vertex*, bool>()),
        graph(_graph) {
            for (auto it = _graph->vertices.begin(); it != _graph->vertices.end(); it++)
                visited[*it] = false;
            visited[_pointer] = true;
        }

        Vertex* pointer;
        std::queue<Vertex*> queue;
        std::map<Vertex*, bool> visited;
        const NodeGraph<V, W>* graph;
    };

    BFS_NodeVertIterator& bfs_begin(int index = 0) {
        return BFS_NodeVertIterator(vertices.begin() + index, this);
    }

    BFS_NodeVertIterator& bfs_begin(Vertex* start) {
        return BFS_NodeVertIterator(start, this);
    }

private:
    std::vector<Vertex*> vertices;
};

//======================================================================================================================

template <typename V, typename W>
class SetGraph : public Graph<V, W>{
public:
    SetGraph() = default;

    SetGraph(SetGraph<V, W>& other):
        vertices(other.vertices),
        set_edges(other.set_edges) {}

    SetGraph<V, W>& operator=(const SetGraph<V, W>& other){
        vertices = other.vertices;
        set_edges = other.set_edges;
        return *this;
    }


    SetGraph<V, W>& operator=(SetGraph<V, W>&& other) noexcept{
        vertices = other.vertices;
        other.vertices.clear();
        set_edges = other.set_edges;
        other.set_edges.clear();
        return *this;
    }


    SetGraph<V, W>& operator=(SetGraph<V, W> other) noexcept{
        vertices = other.vertices;
        other.vertices.clear();
        set_edges = other.set_edges;
        other.set_edges.clear();
        return *this;
    }

    void add_vertex(V& new_vertex) {
        vertices.push_back(new_vertex);
    }

    void add_edge(int from, int to, W& weight=NULL) {
        set_edges.push_back(std::make_pair(std::make_pair(from, to), weight));
    }

    size_t vertices_count() {
        return vertices.size();
    }

    std::list<std::pair<int, W>>& next_vertices(int current_vertex){
        std::list<std::pair<int, W>> next_vertices;
        for (auto edge = set_edges.begin(); edge < set_edges.end(); edge++) {
            if (edge->first.first == current_vertex)
                next_vertices.push_back(std::make_pair(edge->first.second, edge->second));
        }
        return next_vertices;
    }

    //-----------------------iterators--------------------------------

    class SetVertIterator : public Graph<V, W>::VertIterator {
    private:
        explicit SetVertIterator(V* _pointer):
                pointer(_pointer) {}

        V* pointer;
    };

    SetVertIterator& vertex_begin() {
        return SetVertIterator(vertices.begin());
    }

    SetVertIterator& vertex_end() {
        return ListVertIterator(vertices.end());
    }

    class DFS_SetVertIterator : public Graph<V, W>::VertIterator {
    public:
        DFS_SetVertIterator(const DFS_SetVertIterator &it):
                pointer(it.pointer),
                index(it.index),
                stack(it.stack),
                visited(it.visited),
                graph(it.graph) {}

        bool operator!=(DFS_SetVertIterator const& other) const {
            throw CompareException();
        }

        bool operator==(DFS_SetVertIterator const& other) const {
            throw CompareException();
        }

        typename DFS_SetVertIterator::reference operator*() const {
            return *pointer;
        }

        DFS_SetVertIterator& operator++() override {
            std::list<std::pair<int, W>> next = graph->next_vertices(index);
            // add children in stack
            for (auto it = next.begin(); it != next.end(); it++) {
                if (!visited[it->first]) {
                    stack.push(it->first);
                }
            }

            // delete extra
            while(!stack.empty() && visited[stack.top()])
                stack.pop();

            // get next
            if(!stack.empty()) {
                index = stack.top();
                stack.pop();
                visited[index] = true;
                pointer = graph->vertices.begin() + index;
                return *this;
            }
            else {
                for(int i = 0; i < visited.size(); i++) {
                    if(!visited[i]) {
                        index = i;
                        visited[index] = true;
                        pointer = graph->vertices.begin() + index;
                        return *this;
                    }
                }

                throw EndException();
            }
        }

        bool is_end() {
            bool visited_all = true;
            for (auto && i : visited)
                visited_all = (visited_all and i);

            return visited_all;
        }

    private:
        explicit DFS_SetVertIterator(V* _pointer, int _index, SetGraph<V, W>* _graph):
                pointer(_pointer),
                index(_index),
                stack(std::stack<int>()),
                visited(std::vector<bool>(_graph->vertices_count(), false)),
                graph(_graph) {
            visited[_index] = true;
        }

        V* pointer;
        int index;
        std::stack<int> stack;
        std::vector<bool> visited;
        const SetGraph<V, W>* graph;
    };

    DFS_SetVertIterator& dfs_begin(int index = 0) {
        return DFS_SetVertIterator(vertices.begin() + index, index, this);
    }

    class BFS_SetVertIterator : public Graph<V, W>::VertIterator {
    public:
        BFS_SetVertIterator(const BFS_SetVertIterator &it):
                pointer(it.pointer),
                index(it.index),
                queue(it.queue),
                visited(it.visited),
                graph(it.graph) {}

        bool operator!=(BFS_SetVertIterator const& other) const {
            throw CompareException();
        }

        bool operator==(BFS_SetVertIterator const& other) const {
            throw CompareException();
        }

        typename BFS_SetVertIterator::reference operator*() const {
            return *pointer;
        }

        BFS_SetVertIterator& operator++() override {
            std::list<std::pair<int, W>> next = graph->next_vertices(index);
            // add children in queue
            for (auto it = next.begin(); it != next.end(); it++) {
                if (!visited[it->first]) {
                    queue.push(it->first);
                }
            }

            // delete extra
            while(!queue.empty() && visited[queue.front()])
                queue.pop();

            // get next
            if(!queue.empty()) {
                index = queue.front();
                queue.pop();
                visited[index] = true;
                pointer = graph->vertices.begin() + index;
                return *this;
            }
            else {
                for(int i = 0; i < visited.size(); i++) {
                    if(!visited[i]) {
                        index = i;
                        visited[index] = true;
                        pointer = graph->vertices.begin() + index;
                        return *this;
                    }
                }

                throw EndException();
            }
        }

        bool is_end() {
            bool visited_all = true;
            for (auto && i : visited)
                visited_all = (visited_all and i);

            return visited_all;
        }

    private:
        explicit BFS_SetVertIterator(V* _pointer, int _index, SetGraph<V, W>* _graph):
                pointer(_pointer),
                index(_index),
                queue(std::queue<int>()),
                visited(std::vector<bool>(_graph->vertices_count(), false)),
                graph(_graph) {
            visited[_index] = true;
        }

        V* pointer;
        int index;
        std::queue<int> queue;
        std::vector<bool> visited;
        const SetGraph<V, W>* graph;
    };

    BFS_SetVertIterator& bfs_begin(int index = 0) {
        return BFS_SetVertIterator(vertices.begin() + index, index, this);
    }

private:
    std::vector<V> vertices;
    std::list<std::pair<std::pair<int, int>, W>> set_edges;
};

//======================================================================================================================

/*class Convsation {
    explicit operator MatrixGraph<V, W>() const{
        size_t n = vertices.size();
        std::pair<bool, W> null_edge = std::make_pair(false, NULL);
        std::vector<std::vector<std::pair<bool, W>>> matrix_edges(n, std::vector<std::pair<bool, W>>(n, null_edge));
        MatrixGraph<V, W> graph(vertices, matrix_edges);
        for (int from = 0; from < n; from++){
            for (auto it = list_edges[from].begin(); it != list_edges[from].end(); it++){
                graph.add_edge(from, it->first, it->second);
            }
        }
        return graph;
    }
};*/