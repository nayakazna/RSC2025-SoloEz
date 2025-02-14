#ifndef MAP_STATE_HPP
#define MAP_STATE_HPP

#include <string>
#include <vector>
#include <cstdint>

#define KOSONG 0
#define OBSTACLE -1
#define VICTIM -2

class MapState {
public:
    MapState(int width, int height);
    ~MapState(); // Destruktor buat dealokasi memori

    int getGridValue(int x, int y) const;
    void setGridValue(int x, int y, int value);

    // Setter buat rintangan (obstacle) dan korban (victim)
    void setObstacle(int x, int y);
    void setVictim(int x, int y);

    // Getter buat grid data, lebar, dan tinggi
    std::vector<int32_t> getGridData() const;
    int getWidth() const;
    int getHeight() const;

    // Konversi grid data ke string berbentuk matriks
    std::string GridToArray() const;

private:
    int width_, height_;
    int32_t* grid_data_;
};

#endif