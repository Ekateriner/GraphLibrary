#include "library.h"

class Nulltype {} nulltype;

class UniqueException : public std::exception {
    const char* what() const noexcept override {
        return "Base component already exists";
    }
};

class Component {
public:
    void execute() {};
};

class Linker {
public:
    void base_component(Component& new_com) {
        if (components.vertices_count() > 0)
            throw UniqueException();
        components.add_vertex(new_com);
    }

    void add_component(int parent, Component new_com) {
        components.add_vertex(new_com);
        components.add_edge(parent, int(components.vertices_count()) - 1, nulltype);
    }

    void execute(int index) {
        for (auto & it : components.next_vertices(index)) {
            execute(it.first);
        }
        components[index].execute();
    }

private:
    ListGraph<Component, Nulltype> components;
};

class PermissionException : public std::exception {
    const char* what() const noexcept override {
        return "You don't have enough permission";
    }
};

class Handler {
public:
    void handle(int request = 0) {}
    static bool can_handle(int request = 0) { return request > 0; }
};

class ChainHandler {
public:
    void SetNext(int children, int parent) {
        chain.delete_children_edges(children); // delete previous structure
        chain.add_edge(children, parent, nulltype);
    }

    void add_handle(Handler& new_one) {
        chain.add_vertex(new_one);
    }

    void handle(int handler_index = 0) {
        if (!chain[handler_index].can_handle())
            throw PermissionException();

        int next_handler = chain.next_vertices(handler_index).front().first;
        if (chain[next_handler].can_handle())
            handle(next_handler);
        else
            chain[handler_index].handle();
    }

private:
    ListGraph<Handler, Nulltype> chain; // т.к. у меня нет обратного обхода то ребра идут от ребенка к родителю
};

class Element {};

class Visitor{
public:
    virtual void visit(Element some_element) = 0;
};

class Pattern{
public:
private:
    
};