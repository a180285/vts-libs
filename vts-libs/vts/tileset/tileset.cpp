#include "../tileset.hpp"
#include "./detail.hpp"
#include "./driver.hpp"

namespace fs = boost::filesystem;

namespace vadstena { namespace vts {

StaticProperties TileSet::getProperties() const
{
    return {};
}

TileSet::TileSet(const std::shared_ptr<Driver> &driver)
{
    (void) driver;
}

TileSet::TileSet(const std::shared_ptr<Driver> &driver
                 , const StaticProperties &properties)
{
    (void) driver;
    (void) properties;
}

TileSet::~TileSet()
{
}

Mesh TileSet::mesh(const TileId &tileId) const
{
    (void) tileId;
    return {};
}

void TileSet::mesh(const TileId &tileId, const Mesh &mesh) const
{
    (void) tileId;
    (void) mesh;
}

Atlas TileSet::atlas(const TileId &tileId) const
{
    (void) tileId;
    return {};
}

void TileSet::atlas(const TileId &tileId, const Atlas &atlas) const
{
    (void) tileId;
    (void) atlas;
}

MetaNode TileSet::metaNode(const TileId &tileId) const
{
    (void) tileId;
    return {};
}

void TileSet::metaNode(const TileId &tileId, const MetaNode node) const
{
    (void) tileId;
    (void) node;
}

bool TileSet::exists(const TileId &tileId) const
{
    (void) tileId;
    return false;
}

void TileSet::flush()
{
}

void TileSet::begin(utility::Runnable *runnable)
{
    (void) runnable;
}

void TileSet::commit()
{
}

void TileSet::rollback()
{
}

void TileSet::watch(utility::Runnable *runnable)
{
    (void) runnable;
}

bool TileSet::inTx() const
{
    return false;
}

bool TileSet::empty() const
{
    return true;
}

void TileSet::drop()
{
}

LodRange TileSet::lodRange() const
{
    return {};
}

struct TileSet::Factory
{
    static TileSet create(const fs::path &path
                          , const StaticProperties &properties
                          , CreateMode mode)
    {
        (void) path; (void) mode;
        std::shared_ptr<Driver> driver;
        return TileSet(driver, properties);
    }

    static TileSet open(const fs::path &path)
    {
        (void) path;
        std::shared_ptr<Driver> driver;
        return TileSet(driver);
    }
};

TileSet createTileSet(const boost::filesystem::path &path
                      , const StaticProperties &properties
                      , CreateMode mode)
{
    return TileSet::Factory::create(path, properties, mode);
}

TileSet openTileSet(const boost::filesystem::path &path)
{
    return TileSet::Factory::open(path);
}

TileSet::Detail::Detail(const Driver::pointer &driver)
    : readOnly(true), driver(driver), propertiesChanged(false)
    , metadataChanged(false)
{
    loadConfig();
    // load tile index only if there are any tiles
    // if (properties.hasData) {
    //     loadTileIndex();
    // } else {
    //     tileIndex = {};
    // }
}

TileSet::Detail::Detail(const Driver::pointer &driver
                        , const StaticProperties &properties)
    : readOnly(false), driver(driver), propertiesChanged(false)
    , metadataChanged(false)
{
    (void) properties;
}

void TileSet::Detail::loadConfig()
{
}

void TileSet::Detail::saveConfig()
{
}

} } // namespace vadstena::vts