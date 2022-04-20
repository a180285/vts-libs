//
// Created by unity on 4/20/22.
//

#include "meshio_ext.hpp"


namespace fs = boost::filesystem;
namespace bio = boost::iostreams;
namespace bin = utility::binaryio;

namespace vtslibs { namespace vts { namespace detail {

void loadSubmeshVersion_VERSION_WITH_NORMAL(std::istream &in, SubMesh &sm, std::uint8_t flags
    , const math::Extents3 &bbox)
{
    // helper functions
    auto loadVertexComponent([&in](double o, double s) -> double
                             {
                                 std::uint16_t v;
                                 bin::read(in, v);
                                 return o + ((v * s) / std::numeric_limits<std::uint16_t>::max());
                             });

    auto loadTexCoord([&in]() -> double
                      {
                          std::uint16_t v;
                          bin::read(in, v);
                          return (double(v) / std::numeric_limits<std::uint16_t>::max());
                      });

    auto loadNormal([&in]() -> double {
        std::uint32_t v;
        bin::read(in, v);
        return (double(v) / std::numeric_limits<std::uint32_t>::max()) * 2 - 1;
    });

    math::Point3d bbsize(bbox.ur - bbox.ll);

    std::uint32_t vertexCount;
    bin::read(in, vertexCount);
    bool isShortVertex(isShort(vertexCount));
    sm.vertices.resize(vertexCount);

    if (flags & SubMeshFlag::externalTexture) {
        sm.etc.resize(vertexCount);
    }

    // load all vertex components
    auto ietc(sm.etc.begin());
    for (auto &vertex : sm.vertices) {
        vertex(0) = loadVertexComponent(bbox.ll(0), bbsize(0));
        vertex(1) = loadVertexComponent(bbox.ll(1), bbsize(1));
        vertex(2) = loadVertexComponent(bbox.ll(2), bbsize(2));

        if (flags & SubMeshFlag::externalTexture) {
            (*ietc)(0) = loadTexCoord();
            (*ietc)(1) = loadTexCoord();
            ++ietc;
        }
    }

    // load normal
    std::uint32_t normalCount;
    bin::read(in, normalCount);
    bool isShortNormalIndex =  (isShort(normalCount));
    sm.normals.resize(normalCount);
    for (auto &normal : sm.normals) {
        normal[0] = loadNormal();
        normal[1] = loadNormal();
        normal[2] = loadNormal();
    }

    bool isShortTc = true;
    // load (internal) texture coordinates
    if (flags & SubMeshFlag::internalTexture) {
        std::uint32_t tcCount;
        bin::read(in, tcCount);
        isShortTc = isShort(tcCount);
        sm.tc.resize(tcCount);
        for (auto &tc : sm.tc) {
            tc(0) = loadTexCoord();
            tc(1) = loadTexCoord();
        }
    }

    // load faces
    std::uint32_t faceCount;
    bin::read(in, faceCount);
    sm.faces.resize(faceCount);

    if (flags & SubMeshFlag::internalTexture) {
        sm.facesTc.resize(faceCount);
    }

    if (isShortVertex) {
        for (auto &face : sm.faces) {
            std::uint16_t index;
            bin::read(in, index); face(0) = index;
            bin::read(in, index); face(1) = index;
            bin::read(in, index); face(2) = index;
        }
    } else {
        for (auto &face: sm.faces) {
            std::uint32_t index;
            bin::read(in, index); face(0) = index;
            bin::read(in, index); face(1) = index;
            bin::read(in, index); face(2) = index;
        }
    }

    if (flags & SubMeshFlag::internalTexture) {
        if (isShortTc) {
            for (auto &faceTc : sm.facesTc) {
                std::uint16_t index;
                bin::read(in, index); (faceTc)(0) = index;
                bin::read(in, index); (faceTc)(1) = index;
                bin::read(in, index); (faceTc)(2) = index;
            }
        } else {
            for (auto &faceTc : sm.facesTc) {
                std::uint32_t index;
                bin::read(in, index); (faceTc)(0) = index;
                bin::read(in, index); (faceTc)(1) = index;
                bin::read(in, index); (faceTc)(2) = index;
            }
        }
    }

    // Normal face
    if (isShortNormalIndex) {
        for (auto &normalIndex : sm.normalIndexes) {
            std::uint16_t index;
            bin::read(in, index); (normalIndex)(0) = index;
            bin::read(in, index); (normalIndex)(1) = index;
            bin::read(in, index); (normalIndex)(2) = index;
        }
    } else {
        for (auto &normalIndex : sm.normalIndexes) {
            std::uint32_t index;
            bin::read(in, index); (normalIndex)(0) = index;
            bin::read(in, index); (normalIndex)(1) = index;
            bin::read(in, index); (normalIndex)(2) = index;
        }
    }

    // load json string
    std::uint32_t jsonStrCount;
    bin::read(in, jsonStrCount);
    if (jsonStrCount > 0) {
        sm.jsonStr.resize(jsonStrCount);
        bin::read(in, &sm.jsonStr[0], sm.jsonStr.size());
    }
}



} } } // namespace vtslibs::vts::detail