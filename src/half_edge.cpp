#include <include/half_edge.hpp>
#include <glm/glm.hpp>

std::vector<std::shared_ptr<Vertex>> Vertex::neighbor_vertices() const {
    std::vector<std::shared_ptr<Vertex>> neighborhood;
    auto start_he = this->he;
    auto current_he = start_he;
    if (this->boundary) {
        // Checked externally
        if (this->neighboring_boundary_vertices.size() == 2) {
            while (current_he->twin) {
                // Start half-edge added
                neighborhood.push_back(current_he->twin->vertex);
                current_he = current_he->twin->next;
            }
            neighborhood.push_back(current_he->next->vertex);
            current_he = start_he;
            while (current_he->next->next->twin) {
                // Start half-edge not added
                current_he = current_he->next->next->twin;
                neighborhood.push_back(current_he->twin->vertex);
            }
            neighborhood.push_back(current_he->next->next->vertex);
        }
    } else {
        do {
            neighborhood.push_back(current_he->twin->vertex);
            current_he = current_he->twin->next;
        } while (current_he != start_he && current_he);
    }
    return neighborhood;
}

std::vector<std::shared_ptr<HalfEdge>> Vertex::neighbor_half_edges() const {
    std::vector<std::shared_ptr<HalfEdge>> neighborhood;
    auto start_he = this->he; 
    auto current_he = start_he;
    if (this->boundary) {
        // Checked externally
        if (this->neighboring_boundary_vertices.size() == 2) {
            while (current_he->twin) {
                // Start half-edge added
                neighborhood.push_back(current_he);
                current_he = current_he->twin->next;
            }
            neighborhood.push_back(current_he);
            current_he = start_he;
            while (current_he->next->next->twin) {
                // Start half-edge not added
                current_he = current_he->next->next->twin;
                neighborhood.push_back(current_he);
            }
        }
    } else {
        do {
            neighborhood.push_back(current_he);
            current_he = current_he->twin->next;
        } while (current_he != start_he);
    }

    return neighborhood;
}

void Vertex::compute_qem_coeff() {
    auto Q = glm::mat4(0.0f);
    auto hes = this->neighbor_half_edges();
    std::set<std::shared_ptr<Face>> processed_faces;
    for (auto &he: hes) {
        auto face = he->face;
        if (!face || processed_faces.find(face) != processed_faces.end()) {
            continue;
        }
        processed_faces.insert(face);
        auto verts = face->vertices();
        if (verts.size() < 3) {
            continue;
        }

        if (!he->next->next->twin) {
            verts.clear();
            auto it = he->vertex->neighboring_boundary_vertices.begin();
            verts.push_back(he->vertex);
            verts.push_back(*it);
            ++it;
            verts.push_back(*it);
        }

        glm::vec3 v1 = verts[0]->pos;
        glm::vec3 v2 = verts[1]->pos;
        glm::vec3 v3 = verts[2]->pos;
        glm::vec3 normal = glm::normalize(glm::cross(v2 - v1, v3 - v1));
        double d = -glm::dot(normal, v1);
        glm::mat4 faceQ = glm::mat4(
                normal.x * normal.x, normal.x * normal.y, normal.x * normal.z, normal.x * d,
                normal.y * normal.x, normal.y * normal.y, normal.y * normal.z, normal.y * d,
                normal.z * normal.x, normal.z * normal.y, normal.z * normal.z, normal.z * d,
                d * normal.x, d * normal.y, d * normal.z, d * d);
        Q += faceQ;
    }
    this->qem_matrix = Q;
}

double compute_curvature(const std::shared_ptr<Vertex> &vertex) {
    if (vertex->neighboring_boundary_vertices.size() < 2) {
        return 0.0; // Curvature calculation not performed if insufficient neighboring vertices
    }

    auto it = vertex->neighboring_boundary_vertices.begin();
    std::shared_ptr<Vertex> v2 = *it++;
    std::shared_ptr<Vertex> v3 = *it;

    // Compute first and second derivatives
    double x_prime = v3->pos.x - v2->pos.x;
    double x_double_prime = v3->pos.x - 2.0 * vertex->pos.x + v2->pos.x;
    double y_prime = v3->pos.y - v2->pos.y;
    double y_double_prime = v3->pos.y - 2.0 * vertex->pos.y + v2->pos.y;
    double z_prime = v3->pos.z - v2->pos.z;
    double z_double_prime = v3->pos.z - 2.0 * vertex->pos.z + v2->pos.z;

    // Calculate curvature
    double numerator = std::sqrt(
            std::pow(z_double_prime * y_prime - y_double_prime * z_prime, 2) +
            std::pow(x_double_prime * z_prime - z_double_prime * x_prime, 2) +
            std::pow(y_double_prime * x_prime - x_double_prime * y_prime, 2));
    double denominator = std::pow(x_prime * x_prime + y_prime * y_prime + z_prime * z_prime, 1.5);

    return (denominator != 0) ? (numerator / denominator) : 0.0;
}

std::vector<std::shared_ptr<Vertex>> Face::vertices() const {
    std::vector<std::shared_ptr<Vertex>> member_vertices;
    auto start_edge = this->he;
    if (!start_edge) {
        return member_vertices;
    }
    auto current_edge = start_edge;
    do {
        member_vertices.push_back(current_edge->vertex);
        current_edge = current_edge->next;
    } while (current_edge != start_edge);
    return member_vertices;
}

void Edge::compute_contraction(int total) {
    auto v1 = this->he->vertex;
    // Switch from twin to next to avoid null pointers
    auto v2 = this->he->next->vertex;
    bool boundry_edge = (v1->neighboring_boundary_vertices.size() == 2 &&
                         v2->neighboring_boundary_vertices.size() == 2 &&
                         v1->boundary && v2->boundary);
    // Add boundary contraction quality (BCQ)
    double Wb = 2 + 1.0f / (1.0f + exp(-total / 250));
    if (boundry_edge) {
        glm::mat4 Qbcp1 = calculate_bcq();
        glm::mat4 Qbcp2 = calculate_bcq();
        for (int i = 0; i < 4; ++i) {
            Qbcp1[i] *= Wb * compute_curvature(v1);
            Qbcp2[i] *= Wb * compute_curvature(v2);
        }
        v1->qem_matrix += Qbcp1;
        v2->qem_matrix += Qbcp2;
    }

    glm::mat4 Q_combined = v1->qem_matrix + v2->qem_matrix;
    auto Q3 = glm::mat3(Q_combined);
    auto new_pos = glm::vec3(0.0f);
    if (glm::determinant(Q3) != 0) {
        new_pos = -glm::inverse(Q3) * glm::vec3(Q_combined[3][0], Q_combined[3][1], Q_combined[3][2]);
    } else {
        new_pos = (v1->pos + v2->pos) * 0.5f;
    }
    this->verts_contract_pos = new_pos;
    glm::vec4 v_homogeneous = glm::vec4(new_pos, 1.0f);
    this->qem = glm::dot(v_homogeneous, Q_combined * v_homogeneous);
    if (boundry_edge) {
        this->qem += Wb;
    }
}

glm::mat4 Edge::calculate_bcq() {
    std::vector<std::shared_ptr<Vertex>> vertices = he->face->vertices();
    glm::vec3 v1 = vertices[0]->pos;
    glm::vec3 v2 = vertices[1]->pos;
    glm::vec3 v3 = vertices[2]->pos;
    glm::vec3 face_normal = glm::normalize(glm::cross(v2 - v1, v3 - v1));
    float d = -glm::dot(face_normal, v1);
    glm::mat4 Qbcp;
    Qbcp[0] = glm::vec4(face_normal.x * face_normal.x, face_normal.x * face_normal.y, face_normal.x * face_normal.z,
                        face_normal.x * d);
    Qbcp[1] = glm::vec4(face_normal.y * face_normal.x, face_normal.y * face_normal.y, face_normal.y * face_normal.z,
                        face_normal.y * d);
    Qbcp[2] = glm::vec4(face_normal.z * face_normal.x, face_normal.z * face_normal.y, face_normal.z * face_normal.z,
                        face_normal.z * d);
    Qbcp[3] = glm::vec4(d * face_normal.x, d * face_normal.y, d * face_normal.z, d * d);
    return Qbcp;
}

void boundry_edge_contraction(Edge *edge) {
    auto v1 = edge->he->vertex;
    auto v2 = edge->he->next->vertex;
    auto he_1 = edge->he;
    auto he_2 = edge->he->next;
    auto he_3 = edge->he->next->next;

    // Texture part
    glm::vec2 v1_tex_coord = v1->he->tex_coord;
    glm::vec2 v2_tex_coord = v2->he->tex_coord;
    bool v1_only_one_tex = true;
    bool v2_only_one_tex = true;
    for (auto &he: v1->neighbor_half_edges()) {
        if (he->tex_coord != v1_tex_coord) {
            v1_only_one_tex = false;
        }
    }
    for (auto &he: v2->neighbor_half_edges()) {
        if (he->tex_coord != v2_tex_coord) {
            v2_only_one_tex = false;
        }
    }
    glm::vec2 newTexCoord;
    if (v1_only_one_tex && !v2_only_one_tex) {
        if (v1->pos == v2->pos) {
            newTexCoord = he_2->tex_coord;
        } else {
            glm::vec3 edgeVector = v1->pos - v2->pos;
            glm::vec3 newPointVector = edge->verts_contract_pos - v2->pos;
            float projectionLengthRatio = glm::dot(newPointVector, edgeVector) / glm::dot(edgeVector, edgeVector);
            newTexCoord = he_2->tex_coord * (1.0f - projectionLengthRatio) + v1_tex_coord * projectionLengthRatio;
        }
        for (auto &he: v1->neighbor_half_edges()) {
            he->tex_coord = newTexCoord;
        }
        he_2->tex_coord = he_3->twin->tex_coord;
    }
    if (!v1_only_one_tex && v2_only_one_tex) {
        if (v1->pos == v2->pos) {
            newTexCoord = he_1->tex_coord;
        } else {
            glm::vec3 edgeVector = v1->pos - v2->pos;
            glm::vec3 newPointVector = edge->verts_contract_pos - v2->pos;
            float projectionLengthRatio = glm::dot(newPointVector, edgeVector) / glm::dot(edgeVector, edgeVector);
            newTexCoord = v2_tex_coord * (1.0f - projectionLengthRatio) + he_1->tex_coord * projectionLengthRatio;
        }
        for (auto &he: v2->neighbor_half_edges()) {
            he->tex_coord = newTexCoord;
        }
        he_2->tex_coord = he_3->twin->tex_coord;
    }

    he_1->exists = false;
    he_3->exists = false;
    he_3->twin->exists = false;
    he_1->edge->exists = false;
    he_3->edge->exists = false;
    he_1->face->exists = false;
    v1->exists = false;
    he_2->twin->vertex->he = he_2->twin;

    if (v1->neighbor_half_edges().size() == 2) {
        he_3->twin->next->next->next = he_2;
        he_2->next = he_3->twin->next;
        he_2->face = he_3->twin->face;
        he_2->face->he = he_2;
    } else {
        for (auto &he: v1->neighbor_half_edges()) {
            if (he != he_1 && he != he_3->twin) {
                he->vertex = v2;
                if (he->twin == he_3->twin->next->next) {
                    he->twin->next = he_2;
                    he_2->next = he_3->twin->next;
                    he->twin->face->he = he->twin;
                    he_2->face = he->twin->face;
                }
            }
        }
    }
    v2->compute_qem_coeff();
    std::shared_ptr<Vertex> v1_neighbor_non_v2 = nullptr;
    for (auto &v: v1->neighboring_boundary_vertices) {
        if (v != v2) {
            v1_neighbor_non_v2 = v;
            break;
        }
    }
    if (v1_neighbor_non_v2) {
        v2->neighboring_boundary_vertices.erase(v1);
        v2->neighboring_boundary_vertices.insert(v1_neighbor_non_v2);
    }
    if (v1_only_one_tex && v2_only_one_tex) {
        if (v1->pos == v2->pos) {
            newTexCoord = v1_tex_coord;
        } else {
            glm::vec3 edgeVector = v1->pos - v2->pos;
            glm::vec3 newPointVector = edge->verts_contract_pos - v2->pos;
            float projectionLengthRatio = glm::dot(newPointVector, edgeVector) / glm::dot(edgeVector, edgeVector);
            newTexCoord = v2_tex_coord * (1.0f - projectionLengthRatio) + v1_tex_coord * projectionLengthRatio;
        }
        for (auto &he: v2->neighbor_half_edges()) {
            he->tex_coord = newTexCoord;
        }
    } else {
        he_2->tex_coord = he_3->twin->tex_coord;
    }
    v2->pos = edge->verts_contract_pos;
    for (auto &v: v2->neighbor_vertices()) {
        v->compute_qem_coeff();
    }
}

void Edge::edge_contraction() {
    auto v1 = this->he->vertex;
    // Switch from twin to next to avoid null pointers
    auto v2 = this->he->next->vertex;

    if (v1->boundary && v2->boundary) {
        boundry_edge_contraction(this);
        return;
    }

    // Check if both vertices have only one texture coordinate
    glm::vec2 v1_tex_coord = v1->he->tex_coord;
    glm::vec2 v2_tex_coord = v2->he->tex_coord;
    bool v1_only_one_tex = true;
    bool v2_only_one_tex = true;
    for (auto &he: v1->neighbor_half_edges()) {
        if (he->tex_coord != v1_tex_coord) {
            v1_only_one_tex = false;
        }
    }
    for (auto &he: v2->neighbor_half_edges()) {
        if (he->tex_coord != v2_tex_coord) {
            v2_only_one_tex = false;
        }
    }

    auto v1_up = this->he->next->next;
    auto v1_down = this->he->twin->next;
    auto v1_mid = this->he;
    auto v1_up_prev = this->he->next->twin->next->next;
    auto v1_down_next = this->he->twin->next->next->twin->next;
    auto v2_up = this->he->next;
    auto v2_mid = this->he->twin;
    auto v2_down = this->he->twin->next->next;

    bool dificult = false;
    glm::vec2 newTexCoord;
    if (!v1_only_one_tex && v2_only_one_tex) {
        if (v1_mid->tex_coord == v1_down->tex_coord) {
            // Interpolation can be tried here
            if (v1->pos == v2->pos) {
                newTexCoord = v1_mid->tex_coord;
            } else {
                glm::vec3 edgeVector = v2->pos - v1->pos;
                glm::vec3 newPointVector = this->verts_contract_pos - v1->pos;
                float projectionLengthRatio = glm::dot(newPointVector, edgeVector) / glm::dot(edgeVector, edgeVector);
                newTexCoord = v1_mid->tex_coord * (1.0f - projectionLengthRatio) + v2_tex_coord * projectionLengthRatio;
            }
            for (auto &he: v2->neighbor_half_edges()) {
                he->tex_coord = newTexCoord;
            }
            v1_up->tex_coord = v2_up->twin->tex_coord;
            v1_down->tex_coord = v2_down->twin->tex_coord;
        } else {
            dificult = true;
        }
    }

    if (v1_only_one_tex && !v2_only_one_tex) {
        if (v2_up->tex_coord == v2_mid->tex_coord) {
            if (v1->pos == v2->pos) {
                newTexCoord = v2_up->tex_coord;
            } else {
                glm::vec3 edgeVector = v2->pos - v1->pos;
                glm::vec3 newPointVector = this->verts_contract_pos - v1->pos;
                float projectionLengthRatio = glm::dot(newPointVector, edgeVector) / glm::dot(edgeVector, edgeVector);
                newTexCoord = v1_tex_coord * (1.0f - projectionLengthRatio) + v2_up->tex_coord * projectionLengthRatio;
            }
            for (auto &he: v1->neighbor_half_edges()) {
                he->tex_coord = newTexCoord;
            }
            v1_up->tex_coord = v2_up->twin->tex_coord;
            v1_down->tex_coord = v2_down->twin->tex_coord;
        } else {
            dificult = true;
        }
    }

    v2_mid->exists = false;
    v2_mid->twin->exists = false;
    v2_up->exists = false;
    v2_up->twin->exists = false;
    v2_down->exists = false;
    v2_down->twin->exists = false;
    v2_up->edge->exists = false;
    v2_mid->edge->exists = false;
    v2_down->edge->exists = false;
    v1_mid->face->exists = false;
    v2_mid->face->exists = false;
    v2->exists = false;

    v1->he = v1_down;
    v1_up->vertex->he = v1_up;
    v1_down->twin->vertex->he = v1_down->twin;

    if (v2->neighbor_half_edges().size() == 3) {
        v1_up_prev->next = v1_up;
        v1_up->next = v1_down;
        v1_down->next = v1_up_prev;
        v1_up_prev->face->he = v1_up_prev;
        v1_up->face = v1_up_prev->face;
        v1_down->face = v1_up_prev->face;
    } else {
        for (auto &he: v2->neighbor_half_edges()) {
            if (he != v2_up && he != v2_mid && he != v2_down->twin) {
                he->vertex = v1;
                if (he->next == v1_up_prev) {
                    v1_up_prev->next = v1_up;
                    v1_up->next = he;
                    he->face->he = he;
                    v1_up->face = he->face;
                }
                if (he->twin == v1_down_next->next) {
                    he->twin->next = v1_down;
                    v1_down->next = v1_down_next;
                    he->twin->face->he = he->twin;
                    v1_down->face = he->twin->face;
                }
            }
        }
    }
    v1->compute_qem_coeff();

    if (v1_only_one_tex && v2_only_one_tex) {
        if (v1->pos == v2->pos) {
            newTexCoord = v1_tex_coord;
        } else {
            glm::vec3 edgeVector = v2->pos - v1->pos;
            glm::vec3 newPointVector = this->verts_contract_pos - v1->pos;
            float projectionLengthRatio = glm::dot(newPointVector, edgeVector) / glm::dot(edgeVector, edgeVector);
            newTexCoord = v1_tex_coord * (1.0f - projectionLengthRatio) + v2_tex_coord * projectionLengthRatio;
        }
        for (auto &he: v1->neighbor_half_edges()) {
            he->tex_coord = newTexCoord;
        }
    } else {
        dificult = true;
    }

    if (dificult) {
        v1_up->tex_coord = v2_up->twin->tex_coord;
        v1_down->tex_coord = v2_down->twin->tex_coord;
    }

    v1->pos = this->verts_contract_pos;

    for (auto &v: v1->neighbor_vertices()) {
        v->compute_qem_coeff();
    }
}
