#include <glm/glm.hpp>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <memory>
#include <set>

class HalfEdge;

class Vertex;

class Face;

class Edge;

class HalfEdge {
public:
    glm::vec2 tex_coord{};
    std::shared_ptr<HalfEdge> next;
    std::shared_ptr<HalfEdge> twin;
    std::shared_ptr<Face> face;
    std::shared_ptr<Vertex> vertex;
    std::shared_ptr<Edge> edge;
    bool exists;
    [[maybe_unused]] int id;

    explicit HalfEdge(int _id) : id(_id), exists(true) {}
};

class Vertex {
public:
    glm::vec3 pos{};
    std::shared_ptr<HalfEdge> he;
    glm::vec4 qem_coff{};
    [[maybe_unused]] float qem_coff_extra{};
    bool exists{};
    int id;

    // Check if the vertex is on the boundary
    bool boundary = false;
    // Store neighboring boundary vertices
    std::set<std::shared_ptr<Vertex>> neighboring_boundary_vertices;
    // New QEM matrix
    glm::mat4 qem_matrix{};

    explicit Vertex(int _id) : id(_id) {}

    Vertex(glm::vec3 _pos, int _id) : pos(_pos), id(_id), exists(true) {}

    [[nodiscard]] std::vector<std::shared_ptr<Vertex>> neighbor_vertices() const;

    [[nodiscard]] std::vector<std::shared_ptr<HalfEdge>> neighbor_half_edges() const;

    void compute_qem_coeff();
};

class Face {
public:
    std::shared_ptr<HalfEdge> he;
    bool exists;
    [[maybe_unused]] int id;

    explicit Face(int _id) : id(_id), exists(true) {}

    Face(glm::vec3 _color, int _id) : id(_id), exists(true) {}

    [[nodiscard]] std::vector<std::shared_ptr<Vertex>> vertices() const;
};

class Edge {
public:
    std::shared_ptr<HalfEdge> he;
    glm::vec3 verts_contract_pos{};
    double qem{};
    bool visited{}, exists;
    [[maybe_unused]] int id;

    Edge(std::shared_ptr<HalfEdge> _he, int _id) : he(std::move(_he)), id(_id), exists(true) {};

    void compute_contraction(int total);

    void edge_contraction();

    // Extra processing for boundary edges
    glm::mat4 calculate_bcq();
};

class Cmp {
public:
    bool operator()(const std::shared_ptr<Edge> &a, const std::shared_ptr<Edge> &b) {
        return a->qem > b->qem;
    }
};
