//
// Created by unity on 4/20/22.
//

#ifndef VTS_TOOLS_MESHIO_EXT_H
#define VTS_TOOLS_MESHIO_EXT_H

#include <vector>
#include <numeric>

#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/gzip.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/binaryio.hpp"
#include "utility/getenv.hpp"

#include "math/math.hpp"

#include "geometry/forsyth.hpp"
#include "geometry/parse-obj.hpp"

#include "../storage/error.hpp"
#include "../registry/referenceframe.hpp"

#include "mesh.hpp"
#include "atlas.hpp"

namespace vtslibs { namespace vts { namespace detail {
    // helpers normalized bbox
    const math::Extents3 normBbox(-1.0, -1.0, -1.0, +1.0, +1.0, +1.0);

    void loadSubmeshVersion_VERSION_WITH_NORMAL(std::istream &in, SubMesh &sm, std::uint8_t flags
            , const math::Extents3 &bbox);

    inline void loadSubmeshVersion_VERSION_WITH_NORMAL(std::istream &in, NormalizedSubMesh &sm
            , std::uint8_t flags
            , const math::Extents3 &bbox)
    {
        loadSubmeshVersion_VERSION_WITH_NORMAL(in, sm.submesh, flags, normBbox);
        sm.extents = bbox;
    }

    inline bool isShort(std::size_t size) {
        return size <= std::numeric_limits<std::uint16_t>::max();
    }

    struct SubMeshFlag { enum : std::uint8_t {
            internalTexture = 0x1
            , externalTexture = 0x2
            /*, reserved = 0x4 */
            , textureMode = 0x8
        }; };

} } } // namespace vtslibs::vts::detail



#endif //VTS_TOOLS_MESHIO_EXT_H
