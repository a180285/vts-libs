/**
 * Copyright (c) 2017 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * *  Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * \file vts/storage/storageview.cpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Tile set storage view access.
 */

#include <memory>
#include <string>
#include <exception>
#include <algorithm>
#include <iterator>

#include <boost/optional.hpp>
#include <boost/noncopyable.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/range/adaptor/reversed.hpp>

#include "utility/streams.hpp"
#include "utility/guarded-call.hpp"
#include "utility/streams.hpp"

#include "../../storage/error.hpp"
#include "../storageview.hpp"
#include "../../vts.hpp"
#include "./detail.hpp"
#include "../tileset/detail.hpp"

#include "./config.hpp"

namespace fs = boost::filesystem;

namespace utility {

inline fs::path cleanAbsolute(fs::path in)
{
    in = absolute(in);
    fs::path p;
    for (auto i(in.begin()), e(in.end()); i != e; ++i) {
        const auto &part(*i);
        if (part == ".") {
        } else if (part == "..") {
            p = p.parent_path();
        } else {
            p /= part;
        }
    }
    return p;
}

fs::path relpath(const fs::path &src, const fs::path &dst)
{
    auto s(cleanAbsolute(src));
    auto d(cleanAbsolute(dst));

    auto is(s.begin()), es(s.end());
    auto id(d.begin()), ed(d.end());

    for (; (is != es) && (id != ed); ++is, ++id) {
        if (*is != *id) { break; }
    }

    fs::path p;
    // append ".." for each source element
    for (; is != es; ++is) {
        p /= "..";
    }
    // copy whole destination
    for (; id != ed; ++id) {
        p /= *id;
    }

    return p;
}

} // namespace utility

namespace vtslibs { namespace vts {

StorageView::StorageView(const fs::path &path)
    : detail_(std::make_shared<Detail>(path))
{
}

StorageView::StorageView(const fs::path &path, const Properties &properties
                         , Storage &storage)
    : detail_(std::make_shared<Detail>(path, properties, storage))
{}

StorageView::~StorageView()
{
    // no-op
}

StorageView::Detail::~Detail()
{
}

StorageView::Detail::Detail(const fs::path &root)
    : configPath(root)
    , properties(storageview::loadConfig(root))
    , configStat(FileStat::stat(configPath))
    , lastModified(configStat.lastModified)
    , storage(properties.storagePath, OpenMode::readOnly)
{}

StorageView::Detail::Detail(const fs::path &root, const Properties &properties
                            , Storage &storage)
    : configPath(root)
    , properties(properties)
    , configStat(FileStat::stat(configPath))
    , lastModified(configStat.lastModified)
    , storage(storage)
{}

void StorageView::Detail::loadConfig()
{
    properties = loadConfig(configPath);
}

StorageView::Properties StorageView::Detail::loadConfig(const fs::path &path)
{
    try {
        // load config
        return storageview::loadConfig(path);
    } catch (const std::exception &e) {
        LOGTHROW(err1, vtslibs::storage::Error)
            << "Unable to read config: <" << e.what() << ">.";
    }
    throw;
}

MapConfig StorageView::mapConfig() const
{
    return detail().mapConfig();
}

MapConfig StorageView::mapConfig(const fs::path &configPath)

{
    return Detail::mapConfig(configPath, Detail::loadConfig(configPath));
}

MapConfig StorageView::Detail::mapConfig() const
{
    return mapConfig(configPath, properties);
}

MapConfig
StorageView::Detail::mapConfig(const fs::path &configPath
                               , const StorageView::Properties &properties)
{
    // NB: view behaves like a directory although it is a single file
    return Storage::mapConfig
        (properties.storagePath, properties.extra
         , properties.tilesets, properties.freeLayerTilesets
         , utility::relpath(configPath, properties.storagePath));
}

Glue::IdSet StorageView::pendingGlues() const
{
    return detail().storage.pendingGlues(&detail().properties.tilesets);
}

bool StorageView::check(const fs::path &root)
{
    return storageview::checkConfig(root);
}

bool StorageView::externallyChanged() const
{
    return detail().externallyChanged();
}

vtslibs::storage::Resources StorageView::resources() const
{
    return {};
}

bool StorageView::Detail::externallyChanged() const
{
    return (configStat.changed(FileStat::stat(configPath))
            || storage.externallyChanged());
}

std::time_t StorageView::lastModified() const
{
    return detail().lastModified;
}

const Storage& StorageView::storage() const
{
    return detail().storage;
}

const TilesetIdSet& StorageView::tilesets() const
{
    return detail().properties.tilesets;
}

fs::path StorageView::storagePath() const
{
    return detail().properties.storagePath;
}

StorageView openStorageView(const fs::path &path)
{
    return { path };
}

TileSet StorageView::clone(const fs::path &tilesetPath
                           , const CloneOptions &createOptions)
{
    return storage().clone(tilesetPath, createOptions, &tilesets());
}

void StorageView::relocate(const fs::path &root, const RelocateOptions &ro
                           , const std::string &prefix)
{
    if (ro.dryRun) {
        LOG(info3) << prefix << "Simulating relocation of storageview "
                   << root << ".";
    } else {
        LOG(info3) << prefix << "Relocating " << root << ".";
    }

    auto config(storageview::loadConfig(root));

    auto res(ro.apply(config.storagePath.string()));
    if (!ro.dryRun && res.replacement) {
        config.storagePath = *res.replacement;

        // TODO: save config
        storageview::saveConfig(root, config);
    }

    Storage::relocate(res.follow, ro, prefix + "    ");
}

void StorageView::reencode(const fs::path &root, const ReencodeOptions &ro
                           , const std::string &prefix)
{
    if (ro.dryRun) {
        LOG(info3) << prefix << "Simulating reencode of storageview "
                   << root << ".";
    } else {
        LOG(info3) << prefix << "Relocating " << root << ".";
    }

    auto config(storageview::loadConfig(root));

    Storage::reencode(config.storagePath, ro, prefix + "    ");
}

void openStorageView(const fs::path &path
                     , const StorageViewOpenCallback::pointer &callback)
{
    struct StorageCallback : StorageOpenCallback {
        StorageCallback(const fs::path &path
                        , const StorageView::Properties &properties
                        , const StorageViewOpenCallback::pointer &callback)
            : path(path), properties(properties), callback(callback)
        {}

        virtual void done(Storage &storage) {
            // storage open, create storage view and notify interested party
            try {
                StorageView storageView(path, properties, storage);
                callback->done(storageView);
            } catch (...) {
                callback->error(std::current_exception());
            }
        }

        virtual void error(const std::exception_ptr &exc) {
            callback->error(exc);
        }

        const fs::path path;
        const StorageView::Properties properties;
        const StorageViewOpenCallback::pointer callback;
    };

    try {
        // try to load properties
        const auto properties(StorageView::Detail::loadConfig(path));

        callback->openStorage(properties.storagePath
                              , std::make_shared<StorageCallback>
                              (path, properties, callback));
    } catch (...) {
        callback->error(std::current_exception());
    }
}

} } // namespace vtslibs::vts

