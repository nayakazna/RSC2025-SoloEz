#include "map_state.hpp"
#include <iostream>
#include <sstream>

MapState::MapState(int width, int height)
    : width_(width), height_(height) {
    grid_data_ = new int32_t[width * height]; // Alokasi array 1D
    std::fill(grid_data_, grid_data_ + (width * height), KOSONG); // Set default ke 0
}

MapState::~MapState() {
    delete[] grid_data_; // Dealokasi klo udah gak dipake
}

int MapState::getGridValue(int x, int y) const {
    return grid_data_[y * width_ + x];
}

void MapState::setGridValue(int x, int y, int value) {
    grid_data_[y * width_ + x] = value;
}

void MapState::setObstacle(int x, int y) {
    setGridValue(x, y, OBSTACLE);
}

void MapState::setVictim(int x, int y) {
    setGridValue(x, y, VICTIM);
}

int MapState::getWidth() const { return width_; }
int MapState::getHeight() const { return height_; }

std::string MapState::GridToArray() const {
    std::stringstream ss;
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            ss << getGridValue(x, y) << " ";
        }
        ss << "\n";
    }
    return ss.str();
}
