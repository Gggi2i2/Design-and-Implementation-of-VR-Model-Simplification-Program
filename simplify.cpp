#include <iostream>
#include <mutex>
#include <include/mesh.hpp>

int main(int argc, const char **argv) {
    std::string file_path;
    float ratio = 0.1;
    if (argc >= 3) {
        file_path = std::string(argv[1]);
        ratio = std::stof(argv[2]);
    } else {
        std::cout << "Please Input your model path as an argument";
        return 0;
    }
    size_t pos = file_path.size() - 4;
    std::string save_file_path = file_path.substr(0, pos) + "_simplify.obj";

    Mesh mesh;
    mesh.load_obj(file_path);
    mesh.convert_obj_format_to_mesh();

    mesh.simplify(ratio);

    std::cout << std::endl << "====== Verification ======" << std::endl;
    int result;
    result = mesh.verify();

    if (result) {
        std::cout << "ERROR, CODE " << std::oct << result << std::endl;
    } else {
        std::cout << "Verification Pass" << std::endl;
    }

    mesh.convert_mesh_to_obj_format();
    mesh.save_obj(save_file_path);

    return 0;
}
