#include <include/mesh.hpp>
#include <fstream>
#include <sstream>
#include <string>
#include <queue>
#include <algorithm>
#include <map>
#include <array>

bool Mesh::load_obj(const std::string &filepath) {
    if (!this->display_vertices.empty()) {
        this->display_vertices.clear();
        this->display_faces.clear();
        this->display_vt.clear();
    }
    if (filepath.substr(filepath.size() - 4, 4) != ".obj") {
        std::cerr << "Only obj file is supported." << std::endl;
        return false;
    }
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filepath << std::endl;
        return false;
    }
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string prefix;
        iss >> prefix;
        if (prefix == "v") {
            glm::vec3 vertex;
            iss >> vertex.x >> vertex.y >> vertex.z;
            this->display_vertices.push_back(vertex);
        } else if (prefix == "vt") {
            glm::vec2 texCoord;
            iss >> texCoord.x >> texCoord.y;
            this->display_vt.push_back(texCoord);
        } else if (prefix == "f") {
            std::string vertex1, vertex2, vertex3;
            glm::ivec3 vertIndices, texIndices;
            iss >> vertex1 >> vertex2 >> vertex3;
            parse_face(vertex1, vertIndices[0], texIndices[0]);
            parse_face(vertex2, vertIndices[1], texIndices[1]);
            parse_face(vertex3, vertIndices[2], texIndices[2]);
            vertIndices -= 1;
            texIndices -= 1;
            display_faces.emplace_back(vertIndices, texIndices);
        }
    }
    return true;
}

void Mesh::parse_face(const std::string &face, int &vertexIndex, int &texIndex) {
    std::string face_copy = face;
    size_t pos = face_copy.find("//");
    while (pos != std::string::npos) {
        face_copy.replace(pos, 2, "/-1/");
        pos = face_copy.find("//", pos + 3);
    }
    std::replace(face_copy.begin(), face_copy.end(), '/', ' ');
    std::istringstream iss(face_copy);
    texIndex = -1;
    iss >> vertexIndex;
    if (!(iss >> texIndex)) {
        texIndex = -1;
    }
}

void Mesh::convert_obj_format_to_mesh() {
    if (!this->vertices.empty()) {
        this->vertices.clear();
        this->faces.clear();
        this->half_edges.clear();
        this->edges.clear();
    }
    std::map<std::pair<int, int>, std::shared_ptr<HalfEdge>> edge_map;
    for (int i = 0; i < display_vertices.size(); ++i) {
        vertices.push_back(std::make_shared<Vertex>(display_vertices[i], i));
    }
    for (int i = 0; i < display_faces.size(); ++i) {
        auto &vertex_indices = display_faces[i].first;
        auto &tex_indices = display_faces[i].second;

        auto face = std::make_shared<Face>(i);
        std::array<std::shared_ptr<HalfEdge>, 3> halfEdges;
        for (int j = 0; j < 3; ++j) {
            halfEdges[j] = std::make_shared<HalfEdge>(half_edges.size());
            halfEdges[j]->vertex = vertices[vertex_indices[j]];
            halfEdges[j]->face = face;

            // Bind texture coordinate indices with half-edges
            if (tex_indices[j] >= 0) { // Ensure texture indices are valid
                halfEdges[j]->tex_coord = display_vt[tex_indices[j]];
            }
            half_edges.push_back(halfEdges[j]);
            if (!halfEdges[j]->vertex->he) {
                halfEdges[j]->vertex->he = halfEdges[j];
            }
        }
        for (int j = 0; j < 3; ++j) {
            halfEdges[j]->next = halfEdges[(j + 1) % 3];
            auto &v1 = halfEdges[j]->vertex->id;
            auto &v2 = halfEdges[(j + 1) % 3]->vertex->id;
            auto key = std::make_pair(std::min(v1, v2), std::max(v1, v2));
            if (edge_map.count(key)) {
                halfEdges[j]->twin = edge_map[key];
                edge_map[key]->twin = halfEdges[j];
                halfEdges[j]->edge = edge_map[key]->edge;
            } else {
                edge_map[key] = halfEdges[j];
                auto edge = std::make_shared<Edge>(halfEdges[j], edges.size());
                halfEdges[j]->edge = edge;
                edges.push_back(edge);
            }
        }
        face->he = halfEdges[0];
        faces.push_back(face);
    }
    // Mark valid points
    for (const auto &edge: edges) {
        if (!edge->he->twin) {
            auto vertex1 = edge->he->vertex;
            auto vertex2 = edge->he->next->vertex;
            vertex1->boundary = true;
            vertex2->boundary = true;
            vertex1->neighboring_boundary_vertices.insert(vertex2);
            vertex2->neighboring_boundary_vertices.insert(vertex1);
        }
    }
    std::cout << "====== Mesh Information ======" << std::endl;
    this->print_mesh_info();
}

struct Vec2Comparator {
    bool operator()(const glm::vec2 &a, const glm::vec2 &b) const {
        return (a.x < b.x) || (a.x == b.x && a.y < b.y);
    }
};

void Mesh::convert_mesh_to_obj_format() {
    if (!display_vertices.empty() || !display_vt.empty()) {
        display_vertices.clear();
        display_faces.clear();
        display_vt.clear();
    }
    // Store vertices and texture coordinates to their indices
    std::map<std::shared_ptr<Vertex>, int> vertexIndices;
    // Use a custom comparator to manage the uniqueness of texture coordinates
    std::map<glm::vec2, int, Vec2Comparator> texCoordIndices;
    int vertexIndex = 0, texIndex = 0;
    // Traverse all vertices, assign indices
    for (const auto &vertex: vertices) {
        vertexIndices[vertex] = vertexIndex++;
        display_vertices.push_back(vertex->pos);
    }
    // Traverse all faces
    for (const auto &face: faces) {
        std::shared_ptr<HalfEdge> start = face->he;
        std::shared_ptr<HalfEdge> he = start;
        glm::ivec3 vertexIdx, texIdx;
        int idx = 0;
        // Traverse each half-edge of the face, collect vertex and texture coordinates
        do {
            if (texCoordIndices.find(he->tex_coord) == texCoordIndices.end()) {
                texCoordIndices[he->tex_coord] = texIndex++;
                display_vt.push_back(he->tex_coord);
            }
            vertexIdx[idx] = vertexIndices[he->vertex];
            texIdx[idx] = texCoordIndices[he->tex_coord];
            idx++;
            he = he->next;
        } while (he != start); // Loop until returning to the starting half-edge
        display_faces.emplace_back(vertexIdx, texIdx);
    }
}

bool Mesh::save_obj(const std::string &filepath) const {
    if (filepath.substr(filepath.size() - 4, 4) != ".obj") {
        std::cerr << "Only obj file is supported." << std::endl;
        return false;
    }
    std::ofstream out_file(filepath);
    if (!out_file.is_open()) {
        std::cerr << "Failed to open file: " << filepath << std::endl;
        return false;
    }
    size_t last_slash_idx = filepath.find_last_of("\\/"); 
    size_t last_dot_idx = filepath.rfind('.');            
    std::string base_name = filepath.substr(last_slash_idx + 1, last_dot_idx - last_slash_idx - 1);
    std::string mtl_filename = base_name + ".mtl";
    out_file << "mtllib " << mtl_filename << "\n";
    out_file << "usemtl material_0\n";
    // Write vertices
    for (const auto &vertex: display_vertices) {
        out_file << "v " << vertex[0] << " " << vertex[1] << " " << vertex[2] << "\n";
    }
    // Write texture coordinates
    for (const auto &texCoord: display_vt) {
        out_file << "vt " << texCoord[0] << " " << texCoord[1] << "\n";
    }
    // Write faces
    for (const auto &face: display_faces) {
        out_file << "f ";
        for (int i = 0; i < 3; ++i) {
            out_file << face.first[i] + 1 << "/" << face.second[i] + 1 << " ";
        }
        out_file << "\n";
    }
    out_file.close();
    return true;
}

void Mesh::remove_invalid_components() {
    this->vertices.erase(
            std::remove_if(this->vertices.begin(), this->vertices.end(),
                           [](std::shared_ptr<Vertex> v) { return !v->exists; }),
            this->vertices.end());
    this->faces.erase(
            std::remove_if(this->faces.begin(), this->faces.end(), [](std::shared_ptr<Face> f) { return !f->exists; }),
            this->faces.end());
    this->half_edges.erase(
            std::remove_if(this->half_edges.begin(), this->half_edges.end(),
                           [](std::shared_ptr<HalfEdge> he) { return !he->exists; }),
            this->half_edges.end());
    this->edges.erase(
            std::remove_if(this->edges.begin(), this->edges.end(), [](std::shared_ptr<Edge> e) { return !e->exists; }),
            this->edges.end());
}

int Mesh::verify() {
    int rvalue = 0;
    for (auto &v: this->vertices) {
        if (v->exists) {
            if (!v->he->exists) {
                rvalue |= 1 << 0;
            }
        }
    }
    for (auto &f: this->faces) {
        if (f->exists) {
            if (!f->he->exists) {
                rvalue |= 1 << 1;
            }
            if (f->he->next->next->next != f->he) {
                rvalue |= 1 << 2;
            }
        }
    }
    for (auto &he: this->half_edges) {
        if (he->exists) {
            if (!he->vertex->exists) {
                rvalue |= 1 << 3;
            }
            if (!he->edge->exists) {
                rvalue |= 1 << 4;
            }
            if (!he->face->exists) {
                rvalue |= 1 << 5;
            }
            if (he->twin && he->twin->twin != he) {
                rvalue |= 1 << 6;
            }
        }
    }
    for (auto &e: this->edges) {
        if (e->exists) {
            if (!e->he->exists) {
                rvalue |= 1 << 7;
            }
        }
    }

    return rvalue;
}

void Mesh::simplify(const float ratio) {
    int total = faces.size();
    for (const auto &vertex: this->vertices) {
        // Compute pem for non-boundary points or boundary points with two boundary point neighbors
        if (!vertex->boundary || vertex->neighboring_boundary_vertices.size() == 2) {
            vertex->compute_qem_coeff();
        }
    }

    std::priority_queue<std::shared_ptr<Edge>, std::vector<std::shared_ptr<Edge>>, Cmp> cost_min_heap;

    for (const auto &edge: this->edges) {
        if (!edge->he->vertex->boundary && edge->he->twin && !edge->he->next->vertex->boundary) {
            // Only edges that meet the conditions are added to the heap
            edge->compute_contraction(total);
            cost_min_heap.push(edge);
        }
        if (edge->he->vertex->boundary && !edge->he->twin && edge->he->next->vertex->boundary) {
            if (edge->he->vertex->neighboring_boundary_vertices.size() == 2 &&
                edge->he->next->vertex->neighboring_boundary_vertices.size() == 2) {
                edge->compute_contraction(total);
                cost_min_heap.push(edge);
            }
        }
    }

    auto num_face_preserve = std::round(this->faces.size() * ratio);
    int delete_faces = 0;

    while (this->faces.size() - delete_faces > num_face_preserve) {

        if (!cost_min_heap.empty()) {
            std::shared_ptr<Edge> contract_edge_candid = cost_min_heap.top();
            if (contract_edge_candid->exists) {
                auto v1 = contract_edge_candid->he->vertex;
                auto v2 = contract_edge_candid->he->next->vertex;
                int common_neighbors = 0;
                for (auto &n1: v1->neighbor_vertices()) {
                    for (auto &n2: v2->neighbor_vertices()) {
                        if (n1 == n2) {
                            common_neighbors++;
                        }
                    }
                }
                int manifold;
                if (!contract_edge_candid->he->twin) {
                    manifold = 0;
                } else if (contract_edge_candid->he->twin->twin != contract_edge_candid->he) {
                    manifold = -1;
                } else {
                    manifold = 1;
                }

                if (!contract_edge_candid->visited) {
                    // Legality checks should be here
                    if (manifold == 1 && common_neighbors == 2) {
                        contract_edge_candid->edge_contraction();
                        // Mark edges needing updates
                        for (auto &adj_he: contract_edge_candid->he->vertex->neighbor_half_edges()) {
                            adj_he->edge->visited = true;
                        }
                        cost_min_heap.pop();
                        delete_faces += 2;
                    } else if (manifold == 0 && common_neighbors == 1) {
                        auto new_vert = contract_edge_candid->he->next->vertex;
                        contract_edge_candid->edge_contraction();

                        // Mark edges needing updates
                        for (auto &adj_he: new_vert->neighbor_half_edges()) {
                            if (!adj_he->twin) {
                                adj_he->edge->visited = true;
                            }
                            if (!adj_he->next->next->twin) {
                                adj_he->next->next->edge->visited = true;
                            }
                        }
                        cost_min_heap.pop();
                        delete_faces += 1;
                    } else {
                        cost_min_heap.pop();
                    }
                } else {
                    cost_min_heap.pop();
                    contract_edge_candid->visited = false;
                    contract_edge_candid->compute_contraction(total);
                    cost_min_heap.push(contract_edge_candid);
                }
            } else {
                cost_min_heap.pop();
            }
        } else {
            break;
        }
    }

    // Remove invalid components
    this->remove_invalid_components();

    std::cout << std::endl
              << "====== Mesh Simplification with Ratio " << ratio << " ======" << std::endl;
    this->print_mesh_info();
}

void Mesh::print_mesh_info() const {
    std::cout << "number of faces: " << this->faces.size() << std::endl;
    std::cout << "number of vertices: " << this->vertices.size() << std::endl;
    std::cout << "number of half edges: " << this->half_edges.size() << std::endl;
    std::cout << "number of edges: " << this->edges.size() << std::endl;
}
