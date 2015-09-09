#ifndef vadstena_libs_vts_basetypes_hpp_included_
#define vadstena_libs_vts_basetypes_hpp_included_

#include <string>

#include "math/geometry_core.hpp"

#include "../storage/lod.hpp"
#include "../storage/range.hpp"

namespace vadstena { namespace vts {

using storage::Lod;
using storage::Range;
using storage::LodRange;

/** Tile identifier (index in 3D space): LOD + tile index from upper-left corner
 *  in tile grid.
 */
struct TileId {
    Lod lod;
    unsigned int x;
    unsigned int y;

    bool operator<(const TileId &tid) const;
    bool operator==(const TileId &tid) const;
    bool operator!=(const TileId &tid) const { return !operator==(tid); }

    TileId(Lod lod = 0, unsigned int x = 0, unsigned int y = 0)
        : lod(lod), x(x), y(y)
    {}
};

typedef std::array<TileId, 4> Children;

enum class CreateMode {
    failIfExists //!< creation fails if tile set/storage already exists
    , overwrite  //!< existing tile set/storage is replace with new one
};

// inline stuff

inline bool TileId::operator<(const TileId &tileId) const
{
    if (lod < tileId.lod) { return true; }
    else if (tileId.lod < lod) { return false; }

    if (x < tileId.x) { return true; }
    else if (tileId.x < x) { return false; }

    return y < tileId.y;
}

inline bool TileId::operator==(const TileId &tid) const
{
    return ((lod == tid.lod)
            && (x == tid.x)
            && (y == tid.y));
}

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_basetypes_hpp_included_
