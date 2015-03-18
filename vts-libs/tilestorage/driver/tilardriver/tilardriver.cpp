#include <stdexcept>
#include <limits>
#include <type_traits>
#include <fstream>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/utility/in_place_factory.hpp>
#include <boost/uuid/string_generator.hpp>
#include <boost/uuid/random_generator.hpp>
#include <boost/uuid/uuid_io.hpp>

#include "utility/streams.hpp"
#include "utility/path.hpp"

#include "../tilardriver.hpp"
#include "../../io.hpp"
#include "../../error.hpp"
#include "../../config.hpp"
#include "../fstreams.hpp"

namespace vadstena { namespace tilestorage {

namespace fs = boost::filesystem;

namespace {
    const std::string KeyBinaryOrder("binaryOrder");
    const std::uint8_t DefaultBinaryOrder(5);

    const std::string KeyUUID("uuid");

    const std::string ConfigName("mapConfig.json");
    const std::string TileIndexName("index.bin");
    const std::string TransactionRoot("tx");

    const std::string filePath(File type)
    {
        switch (type) {
        case File::config: return ConfigName;
        case File::tileIndex: return TileIndexName;
        }
        throw "unknown file type";
    }

    template <typename T, class Enable = void>
    T getOption(const DriverProperties::Options&, const std::string &
                , const boost::optional<T>& = boost::none);

    template
    <typename T
     , class = typename std::enable_if<std::is_unsigned<T>::value>::type>
    T getOption(const DriverProperties::Options &options
                , const std::string &key
                , const boost::optional<T> &defaultValue = boost::none)
    {
        auto foptions(options.find(key));
        if (foptions == options.end()) {
            if (!defaultValue) {
                LOGTHROW(err2, std::runtime_error)
                    << "Option <" << key << "> not found "
                    "and no default value has been provided.";
            }
            return *defaultValue;
        }

        std::uint64_t value{};
        try {
            value = boost::any_cast<std::uint64_t>(foptions->second);
        } catch (const boost::bad_any_cast&) {
            LOGTHROW(err2, std::logic_error)
                << "Options value <" << key << "> is not an unsigned integer.";
        }

        if (value > std::numeric_limits<T>::max()) {
            LOGTHROW(err2, std::logic_error)
                << "Options value <" << key << "> is doesn't fit into "
                "requested type.";
        }
        return static_cast<T>(value);
    }

    template<>
    boost::uuids::uuid getOption<boost::uuids::uuid, void>
    (const DriverProperties::Options &options, const std::string &key
     , const boost::optional<boost::uuids::uuid> &defaultValue)
    {
        auto foptions(options.find(key));
        if (foptions == options.end()) {
            if (!defaultValue) {
                LOGTHROW(err2, std::runtime_error)
                    << "Option <" << key << "> not found "
                    "and no default value has been provided.";
            }
            return *defaultValue;
        }

        try {
            return boost::uuids::string_generator()
                (boost::any_cast<std::string>(foptions->second));
        } catch (const boost::bad_any_cast&) {
            LOGTHROW(err2, std::logic_error)
                << "Options value <" << key << "> is not a string.";
        }

        throw;
    }

    boost::uuids::uuid generateUuid() {
        // generate random uuid
        return boost::uuids::random_generator()();
    }
} // namespace

namespace tilardriver {

long mask(int order)
{
    long value(0);
    for (long bit(1l); order; --order, bit <<= 1) {
        value |= bit;
    }
    return value;
}

Options::Options(const Driver::CreateProperties &properties)
    : baseTileSize(properties->baseTileSize)
    , alignment(properties->alignment)
    , binaryOrder(getOption<decltype(binaryOrder)>
                 (properties->driver.options, KeyBinaryOrder))
    , uuid(getOption<boost::uuids::uuid>
           (properties->driver.options, KeyUUID))
    , tileMask(mask(binaryOrder))
{}

Options::Options(const Driver::CreateProperties &properties, bool)
    : baseTileSize(properties->baseTileSize)
    , alignment(properties->alignment)
    , binaryOrder(getOption<decltype(binaryOrder)>
                 (properties->driver.options, KeyBinaryOrder
                  , DefaultBinaryOrder))
    , uuid(properties.cloned
           ? generateUuid()
           : getOption<boost::uuids::uuid>
           (properties->driver.options, KeyUUID, generateUuid()))
    , tileMask(mask(binaryOrder))
{}

} // namespace tilardriver

TilarDriver::TilarDriver(const boost::filesystem::path &root
                         , CreateMode mode, const CreateProperties &properties)
    : Driver(false)
    , root_(absolute(root)), tmp_(root_ / TransactionRoot)
    , options_(properties, true)
    , cache_(root_, options_, false)
{
    if (!create_directories(root_)) {
        // directory already exists -> fail if mode says so
        if (mode == CreateMode::failIfExists) {
            LOGTHROW(err2, TileSetAlreadyExists)
                << "Tile set at " << root_ << " already exists.";
        }
    }
}

TilarDriver::TilarDriver(const boost::filesystem::path &root
                         , OpenMode mode)
    : Driver(mode == OpenMode::readOnly)
    , root_(absolute(root)), tmp_(root_ / TransactionRoot)
    , options_(tilestorage::loadConfig(root_ / filePath(File::config)))
    , cache_(root_, options_, (mode == OpenMode::readOnly))
{
}

TilarDriver::~TilarDriver()
{
    if (tx_) {
        LOG(warn3) << "Active transaction on driver close; rolling back.";
        try {
            rollback_impl();
        } catch (const std::exception &e) {
            LOG(warn3)
                << "Error while trying to destroy active transaction on "
                "driver close: <" << e.what() << ">.";
        }
    }
}

OStream::pointer TilarDriver::output_impl(File type)
{
    const auto name(filePath(type));
    if (!tx_) {
        // no transaction -> plain file
        const auto path(root_ / name);
        LOG(info1) << "Saving to " << path << ".";
        return std::make_shared<FileOStream>(type, path);
    }

    const auto path(tmp_ / name);
    LOG(info1) << "Saving to " << path << ".";

    const auto tmpPath(utility::addExtension(path, ".tmp"));
    return std::make_shared<FileOStream>
        (type, tmpPath, [this, path, tmpPath, name](bool success)
    {
        if (!success) {
            // failed -> remove
            LOG(warn2)
                << "Removing failed file " << tmpPath << ".";
            fs::remove(tmpPath);
            return;
        }

        // OK -> move file to destination
        LOG(info1)
            << "Moving file " << tmpPath << " to " << path << ".";
        rename(tmpPath, path);

        // remember file in transaction
        tx_->files.insert(name);
});
}

IStream::pointer TilarDriver::input_impl(File type) const
{
    const auto name(filePath(type));

    auto path(root_ / name);
    if (tx_) {
        // we have active transaction: is file part of the tx?
        auto ffiles(tx_->files.find(name));
        if (ffiles != tx_->files.end()) {
            // yes -> tmp file
            path = tmp_ / name;
        }
    }

    LOG(info1) << "Loading from " << path << ".";
    return std::make_shared<FileIStream>(type, path);
}

OStream::pointer TilarDriver::output_impl(const TileId tileId, TileFile type)
{
    return cache_.output(tileId, type);
}

IStream::pointer TilarDriver::input_impl(const TileId tileId, TileFile type)
    const
{
    return cache_.input(tileId, type);
}

FileStat TilarDriver::stat_impl(File type) const
{
    // TODO: add tx support
    const auto name(filePath(type));
    const auto path(root_ / name);
    LOG(info1) << "Statting " << path << ".";
    return FileStat::stat(path);
}

FileStat TilarDriver::stat_impl(const TileId tileId, TileFile type) const
{
    return cache_.stat(tileId, type);
}

void TilarDriver::remove_impl(const TileId tileId, TileFile type)
{
    return cache_.remove(tileId, type);
}

void TilarDriver::begin_impl()
{
    if (tx_) {
        LOGTHROW(err2, PendingTransaction)
            << "Pending transaction.";
    }

    // remove whole tmp directory
    remove_all(tmp_);
    // create fresh tmp directory
    create_directories(tmp_);

    // begin tx
    tx_ = boost::in_place();
}

void TilarDriver::commit_impl()
{
    if (!tx_) {
        LOGTHROW(err2, PendingTransaction)
            << "No pending transaction.";
    }

    for (const auto &file : tx_->files) {
        // move file
        rename(tmp_ / file, root_ / file);
    }

    // remove whole tmp directory
    remove_all(tmp_);
    cache_.commit();

    // no tx at all
    tx_ = boost::none;
}

void TilarDriver::rollback_impl()
{
    if (!tx_) {
        LOGTHROW(err2, PendingTransaction)
            << "No pending transaction.";
    }

    // remove whole tmp directory
    remove_all(tmp_);
    cache_.rollback();

    // no tx at all
    tx_ = boost::none;
}

void TilarDriver::flush_impl()
{
    cache_.commit();
}

void TilarDriver::drop_impl()
{
    if (tx_) {
        LOGTHROW(err2, PendingTransaction)
            << "Cannot drop tile set inside an active transaction.";
    }

    // remove whole tmp directory
    remove_all(root_);
}

void TilarDriver::update_impl() {
    if (tx_) {
        LOGTHROW(err2, PendingTransaction)
            << "Cannot update tile set inside an active transaction.";
    }
}

DriverProperties TilarDriver::properties_impl() const
{
    DriverProperties dp;
    dp.type = Factory::staticType();
    dp.options[KeyBinaryOrder]
        = boost::any(std::uint64_t(options_.binaryOrder));
    dp.options[KeyUUID]
        = boost::any(to_string(options_.uuid));
    return dp;
}

std::string TilarDriver::detectType_impl(const std::string &location)
{
    try {
        // try load config
        return tilestorage::loadConfig
            (fs::path(location) / filePath(File::config)).driver.type;
    } catch (const std::exception&) {}
    return {};
}

const std::string TilarDriver::help
("Filesystem-based storage driver with deep directory structure. "
 "Tiles and metatiles are stored inside Tile Archives.");

} } // namespace vadstena::tilestorage
