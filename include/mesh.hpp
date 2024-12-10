#include <half_edge.hpp>
#include <glm/glm.hpp>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <set>
#include <memory>

using namespace glm;

class Mesh {
private:
    void remove_invalid_components();

    static void parse_face(const std::string &face, int &vertexIndex, int &texIndex);

public:
    std::vector<vec3> display_vertices;
    std::vector<std::pair<ivec3, ivec3>> display_faces;
    std::vector<vec2> display_vt;
    std::vector<std::shared_ptr<Vertex>> vertices;
    std::vector<std::shared_ptr<HalfEdge>> half_edges;
    std::vector<std::shared_ptr<Face>> faces;
    std::vector<std::shared_ptr<Edge>> edges;

    Mesh() = default;

    ~Mesh() = default;

    bool load_obj(const std::string &filepath);

    [[nodiscard]] bool save_obj(const std::string &filepath) const;

    void convert_obj_format_to_mesh();

    void convert_mesh_to_obj_format();

    void simplify(float ratio);

    void print_mesh_info() const;

    int verify();
};
