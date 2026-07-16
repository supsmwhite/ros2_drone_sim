#pragma once

namespace drone_planning
{

struct GridIndex
{
  int x{0};
  int y{0};
  int z{0};
};

inline bool operator==(const GridIndex & lhs, const GridIndex & rhs)
{
  return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}

inline bool operator!=(const GridIndex & lhs, const GridIndex & rhs)
{
  return !(lhs == rhs);
}

}  // namespace drone_planning
