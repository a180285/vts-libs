#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/crc.hpp>

#include <boost/multi_index_container.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/identity.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/mem_fun.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/time.hpp"

#include "./cache.hpp"
#include "../../io.hpp"
#include "../../openfiles.hpp"

namespace vadstena { namespace tilestorage { namespace tilardriver {

namespace {

Tilar::ContentTypes tileContentTypes({ "", "image/jpeg" });
Tilar::ContentTypes metatileContentTypes;

std::uint32_t calculateHash(const std::string &data)
{
    boost::crc_32_type crc;
    crc.process_bytes(data.data(), data.size());
    return crc.checksum();
}

fs::path dir(const fs::path &filename)
{
    const auto hash(calculateHash(filename.string()));
    return str(boost::format("%02x") % ((hash >> 24) & 0xff));
}

int fileType(TileFile type) {
    switch (type) {
    case TileFile::meta: return 0;
    case TileFile::mesh: return 0;
    case TileFile::atlas: return 1;
    }
    throw;
}

typedef decltype(utility::usecFromEpoch()) Time;

} // namespace

struct Cache::Archives
{
    struct Record {
        Record(Index index, Tilar &&tilar)
            : index(index), lastHit(utility::usecFromEpoch())
            , tilar_(std::move(tilar))
        {}

        /** Allow non-const access to underlying tilar using const_cast.
         *
         *  Reason: multi-index-container's interator is always const because
         *          non-const iterator could modify keys with disastrous
         *          consequences. But tilar is not a key, therefore we can
         *          safely modify it as we wish.
         */
        Tilar& tilar() const {
            return const_cast<Tilar&>(tilar_);
        }

        void hit() { lastHit = utility::usecFromEpoch(); }

        void infinity() { lastHit = ~(Time(0)); }

        Index index;
        Time lastHit;

    private:
        Tilar tilar_;
    };

    struct IndexIdx {};
    struct LastHitIdx {};

    typedef boost::multi_index_container<
        Record
        , boost::multi_index::indexed_by<
              boost::multi_index::ordered_unique
              <boost::multi_index::tag<IndexIdx>
               , BOOST_MULTI_INDEX_MEMBER
               (Record, decltype(Record::index), index)>
              , boost::multi_index::ordered_non_unique
              <boost::multi_index::tag<LastHitIdx>
               , BOOST_MULTI_INDEX_MEMBER
               (Record, decltype(Record::lastHit), lastHit)>
              >
        > Map;

    const fs::path root;
    const std::string extension;
    const Tilar::Options options;
    const bool readOnly;
    Map map;
    const Tilar::ContentTypes &contentTypes;

    Archives(const fs::path &root, const std::string &extension
             , bool readOnly, int filesPerTile, const Options &options
             , const Tilar::ContentTypes &contentTypes);

    Tilar& open(const Index &archive);

    fs::path filePath(const Index &index) const;

    void commitChanges() {
        for (auto &record : map) { record.tilar().commit(); }
    }

    void discardChanges() {
        for (auto &record : map) { record.tilar().rollback(); }
    }

private:
    void houseKeeping(const Index *keep = nullptr);

    template <typename Idx, typename Iterator>
    inline void hit(Idx &idx, Iterator iterator) {
        idx.modify(iterator, [](Record &r) { r.hit(); });
    }

    template <typename Idx, typename Iterator>
    inline void infinity(Idx &idx, Iterator iterator) {
        idx.modify(iterator, [](Record &r) { r.infinity(); });
    }
};

Cache::Archives::Archives(const fs::path &root, const std::string &extension
                          , bool readOnly, int filesPerTile
                          , const Options &options
                          , const Tilar::ContentTypes &contentTypes)
    : root(root), extension(extension)
    , options(options.binaryOrder, filesPerTile, options.uuid)
    , readOnly(readOnly), contentTypes(contentTypes)
{}

fs::path Cache::Archives::filePath(const Index &index) const
{
    const auto filename(str(boost::format("%s-%07d-%07d.%s")
                            % index.lod % index.easting % index.northing
                            % extension));
    const auto parent(root / dir(filename));
    create_directories(parent);
    return parent / filename;
}

Cache::Cache(const fs::path &root, const Options &options
             , bool readOnly)
    : root_(root), options_(options), readOnly_(readOnly)
    , tiles_(new Archives
             (root, "tiles", readOnly, 2, options
              , tileContentTypes))
    , metatiles_(new Archives
                 (root, "metatiles", readOnly, 1, options
                  , metatileContentTypes))
{}

namespace {

Tilar tilar(const fs::path &path, const Tilar::Options &options
            , bool readOnly)
{
    if (readOnly) {
        // read-only
        return Tilar::open(path, options
                           , Tilar::OpenMode::readOnly);
    }
    return Tilar::create(path, options
                         , Tilar::CreateMode::appendOrTruncate);
}

} // namespace

Cache::~Cache() {}

void Cache::Archives::houseKeeping(const Index *keep)
{
    if (!OpenFiles::critical()) { return; }

    auto &idx(map.get<LastHitIdx>());

    LOG(info1)
        << "Critical number of open files reached (current count is "
        << OpenFiles::count() << ") trying to drop some of "
        << idx.size() << " files owned by this driver.";

    int toDrop(5);
    for (auto iidx(idx.begin()), eidx(idx.end()); toDrop && (iidx != eidx); ) {
        // skip keep file
        if (keep && (iidx->index == *keep)) {
            LOG(debug) << "File " << iidx->tilar().path()
                       << " must be kept in the cache.";
            ++iidx;
            continue;
        }

        auto &file(iidx->tilar());

        switch (file.state()) {
        case Tilar::State::detached:
            LOG(debug)
                << "File " << file.path() << '/' << iidx->lastHit
                << " is already detached.";
            return;

        case Tilar::State::detaching:
            LOG(debug) << "File " << file.path() << '/' << iidx->lastHit
                       << " is being detached.";
            return;

        case Tilar::State::pristine:
            // no changes -> we are free to remove the file
            LOG(debug) << "Removing pristine tilar file "
                       << file.path() << '/' << iidx->lastHit
                       << " from the cache.";

            iidx = idx.erase(iidx);
            --toDrop;
            break;

        case Tilar::State::changed:
            // file is changed -> ask to detach
            LOG(debug)
                << "Asking to detach changed file "
                << file.path() << '/' << iidx->lastHit << ".";

            // ask for file detachment
            file.detach();

            // hit -> move to the future, next time this is function is called
            // we will not encounter this detached file again
            {
                auto old(iidx);
                ++iidx;
                infinity(idx, old);
            }

            // mark one more dropped file
            --toDrop;
            break;
        }
    }
}

Tilar& Cache::Archives::open(const Index &archive)
{
    auto fmap(map.find(archive));
    if (fmap != map.end()) {
        hit(map, fmap);
        // housekeeping, we want to keep found file
        houseKeeping(&fmap->index);
        return fmap->tilar();
    }

    // housekeeping before open
    houseKeeping();

    auto path(filePath(archive));
    auto file(tilar(path, options, readOnly));
    file.setContentTypes(contentTypes);

    return map.insert
        (Record(archive, std::move(file))).first->tilar();
}

IStream::pointer Cache::input(const TileId tileId, TileFile type)
{
    auto index(options_.index(tileId, fileType(type)));
    return getArchives(type).open(index.archive).input(index.file);
}

OStream::pointer Cache::output(const TileId tileId, TileFile type)
{
    auto index(options_.index(tileId, fileType(type)));
    return getArchives(type).open(index.archive).output(index.file);
}

void Cache::remove(const TileId tileId, TileFile type)
{
    auto index(options_.index(tileId, fileType(type)));
    try {
        return getArchives(type).open(index.archive).remove(index.file);
    } catch (const std::exception &e) {
        // ignore, this fails when the file cannot be opened
    }
}

std::size_t Cache::size(const TileId tileId, TileFile type)
{
    auto index(options_.index(tileId, fileType(type)));
    return getArchives(type).open(index.archive).size(index.file);
}

FileStat Cache::stat(const TileId tileId, TileFile type)
{
    auto index(options_.index(tileId, fileType(type)));
    return getArchives(type).open(index.archive).stat(index.file);
}

Driver::Resources Cache::resources()
{
    return { tiles_->map.size() + metatiles_->map.size(), 0 };
}

void Cache::commit()
{
    if (readOnly_) { return; }
    tiles_->commitChanges();
    metatiles_->commitChanges();
}

void Cache::rollback()
{
    if (readOnly_) { return; }
    tiles_->discardChanges();
    metatiles_->discardChanges();
}

} } } // namespace vadstena::tilestorage::tilardriver
