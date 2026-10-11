#include "dog_map/high_column_filter.hpp"
#include <iostream>

// Dependency-free probe of the production classifier used by OccMap::PubRes.
int main(int argc, char **argv) {
    if (argc != 6) return 3;
    try {
        const HighColumnFilter filter(std::string(argv[1]) == "true",
                                      std::stod(argv[4]), std::stod(argv[5]));
        std::cout << (filter.ignores(std::stoi(argv[2]), std::stod(argv[3]))
                          ? "true" : "false") << '\n';
    } catch (const std::invalid_argument &error) {
        std::cerr << error.what() << '\n';
        return 2;
    }
}
