#pragma once
#include <vector>
#include <cstdint>
#include <string>
#include <stdexcept>

namespace re{ //robot_emblem

enum class Tile : int8_t { Free=0, Wall=1, High=2 };

struct MapGrid {
  int w{0}, h{0};
  std::vector<Tile> tiles; // row-major: idx = y*w + x

  bool inBounds(int x,int y) const { return x>=0 && y>=0 && x<w && y<h; }

//   Tile at(int x,int y) const {
//     if(!inBounds(x,y)) throw std::out_of_range("MapGrid::at out of bounds");
//     return tiles[y*w + x];
//   }

//   bool passable(int x,int y) const {
//     auto t = at(x,y);
//     return t != Tile::Wall;
//   }
// };
};
}