#pragma once

struct GridCell
{
    int x = 0;
    int z = 0;

    bool operator==(const GridCell& other) const
    {
        return x == other.x && z == other.z;
    }

    bool operator!=(const GridCell& other) const
    {
        return !(*this == other);
    }
};