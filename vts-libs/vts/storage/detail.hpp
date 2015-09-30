/**
 * \file vts/storage/detail.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Storage access (internals).
 */

#ifndef vadstena_libs_vts_storage_detail_hpp_included_
#define vadstena_libs_vts_storage_detail_hpp_included_

#include <boost/filesystem/path.hpp>

#include "../storage.hpp"

namespace vadstena { namespace vts {

struct Storage::Properties : StorageProperties {
    /** Data version/revision. Should be increment anytime the data change.
     *  Used in template URL's to push through caches.
     */
    unsigned int revision;

    Properties() : revision(0) {}
};

struct Storage::Detail
{
    bool readOnly;

    Detail(const boost::filesystem::path &root
           , const StorageProperties &properties
           , CreateMode mode);

    ~Detail();

    boost::filesystem::path root;

    Properties properties;
};

inline void Storage::DetailDeleter::operator()(Detail *d) { delete d; }

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_storage_detail_hpp_included_