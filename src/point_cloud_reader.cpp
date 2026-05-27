#include "point_cloud_reader.h"

#include <cstdint>
#include <stdexcept>
#include <string>

// laz-perf — header-only structs + compiled reader
#include <lazperf/lazperf.hpp>
#include <lazperf/readers.hpp>
#include <lazperf/las.hpp>

#include <godot_cpp/classes/file_access.hpp>
#include <godot_cpp/classes/time.hpp>
#include <godot_cpp/core/error_macros.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

using namespace godot;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Returns the byte-offset of the RGB record within a point buffer for formats
/// that carry colour data.  Returns -1 for formats without RGB.
static inline int rgb_offset_for_format(int fmt) {
    switch (fmt) {
        case 2:  return 20; // point10 (20)
        case 3:  return 28; // point10 (20) + gpstime (8)
        case 5:  return 28; // point10 (20) + gpstime (8) — rgb before wave
        case 7:  return 30; // point14 (30)
        case 8:  return 30; // point14 (30)
        case 10: return 30; // point14 (30)
        default: return -1;
    }
}

/// Returns true for LAS 1.4 extended point formats (6–10).
static inline bool is_fmt14(int fmt) { return fmt >= 6; }

// ---------------------------------------------------------------------------
// PointCloudReader
// ---------------------------------------------------------------------------

PointCloudReader::PointCloudReader() {}

int PointCloudReader::load(const String &p_path) {
    // ---- 1. Read the whole file via Godot FileAccess -----------------------
    // This is the only platform-safe approach (works on Android assets and
    // Emscripten's virtual FS, unlike std::ifstream).
    Ref<FileAccess> fa = FileAccess::open(p_path, FileAccess::READ);
    if (!fa.is_valid()) {
        ERR_PRINT("PointCloudReader: cannot open file: " + p_path);
        return (int)ERR_FILE_NOT_FOUND;
    }

    uint64_t file_size = fa->get_length();
    if (file_size == 0) {
        ERR_PRINT("PointCloudReader: file is empty: " + p_path);
        return (int)ERR_FILE_CORRUPT;
    }

    const uint64_t t0 = Time::get_singleton()->get_ticks_usec();
    PackedByteArray raw = fa->get_buffer((int64_t)file_size);
    fa->close();

    if ((uint64_t)raw.size() != file_size) {
        ERR_PRINT("PointCloudReader: short read from file: " + p_path);
        return (int)ERR_FILE_CORRUPT;
    }

    // ---- 2. Parse with laz-perf --------------------------------------------
    try {
        // mem_file accepts a non-const char*, but does not modify the buffer.
        char *buf_ptr = reinterpret_cast<char *>(raw.ptrw());
        lazperf::reader::mem_file mf(buf_ptr, (size_t)file_size);

        const lazperf::header14 &hdr = mf.header();
        uint64_t count = mf.pointCount();
        if (count == 0) {
            // Valid but empty file — clear and return OK
            _point_count     = 0;
            _has_color       = false;
            _source_path     = p_path;
            _point_format_id = (int)hdr.point_format_id;
            _load_time_ms    = 0.0f;
            return (int)OK;
        }

        const double sx = hdr.scale.x,  sy = hdr.scale.y,  sz = hdr.scale.z;
        const double ox = hdr.offset.x, oy = hdr.offset.y, oz = hdr.offset.z;
        const int    fmt = hdr.point_format_id;
        const int    rgb_off = rgb_offset_for_format(fmt);
        const bool   fmt_has_rgb = (rgb_off >= 0);
        const bool   fmt_is14    = is_fmt14(fmt);

        // Centroid in LAS world space (double precision) used to centre
        // float positions and avoid catastrophic cancellation for distant coords.
        const double cx = (hdr.minx + hdr.maxx) * 0.5;
        const double cy = (hdr.miny + hdr.maxy) * 0.5;
        const double cz = (hdr.minz + hdr.maxz) * 0.5;

        // Store the world-space centroid in Godot Y-up convention:
        //   Godot X = LAS X (easting)   Godot Y = LAS Z (elevation)
        //   Godot Z = -LAS Y (northing negated: Godot +Z is toward viewer / south)
        _center          = Vector3((float)cx, (float)cz, -(float)cy);
        _has_color       = fmt_has_rgb;
        _source_path     = p_path;
        _point_format_id = fmt;

        // ---- 3. Pre-allocate output arrays ----------------------------------
        _positions.resize((int64_t)count);
        _intensities.resize((int64_t)count);
        _classifications.resize((int64_t)count);
        _return_numbers.resize((int64_t)count);
        if (fmt_has_rgb) {
            _colors.resize((int64_t)count);
        } else {
            _colors.resize(0);
        }

        Vector3 *pos_w   = _positions.ptrw();
        float   *int_w   = _intensities.ptrw();
        uint8_t *cls_w   = _classifications.ptrw();
        uint8_t *ret_w   = _return_numbers.ptrw();
        Color   *col_w   = fmt_has_rgb ? _colors.ptrw() : nullptr;

        // ---- 4. Read and parse points ----------------------------------------
        // 128 bytes is safe for all formats (largest is format 10 at 67 bytes).
        char pbuf[128];

        for (uint64_t i = 0; i < count; i++) {
            mf.readPoint(pbuf);

            // Update progress every 50 000 points so callers can poll.
            if ((i & 0xFFFF) == 0) {
                _load_progress = (int)((i * 100) / count);
            }

            if (!fmt_is14) {
                // Formats 0–5: 20-byte point10 base
                lazperf::las::point10 p(pbuf);

                // Convert to Godot Y-up, centred for float precision
                pos_w[i] = Vector3(
                        (float)(p.x * sx + ox - cx),    // Godot X = LAS X
                        (float)(p.z * sz + oz - cz),    // Godot Y = LAS Z (elevation)
                        -(float)(p.y * sy + oy - cy));  // Godot Z = -LAS Y (northing negated)

                int_w[i] = (float)p.intensity / 65535.0f;
                cls_w[i] = p.classification;
                ret_w[i] = p.return_number;

                if (fmt_has_rgb) {
                    lazperf::las::rgb rgb_val(pbuf + rgb_off);
                    col_w[i] = Color(
                            rgb_val.r / 65535.0f,
                            rgb_val.g / 65535.0f,
                            rgb_val.b / 65535.0f,
                            1.0f);
                }
            } else {
                // Formats 6–10: 30-byte point14 base (LAS 1.4)
                lazperf::las::point14 p(pbuf);

                pos_w[i] = Vector3(
                        (float)(p.x() * sx + ox - cx),
                        (float)(p.z() * sz + oz - cz),
                        -(float)(p.y() * sy + oy - cy));

                int_w[i] = (float)p.intensity() / 65535.0f;
                cls_w[i] = p.classification();
                ret_w[i] = (uint8_t)p.returnNum();

                if (fmt_has_rgb) {
                    lazperf::las::rgb rgb_val(pbuf + rgb_off); // rgb14 shares rgb layout
                    col_w[i] = Color(
                            rgb_val.r / 65535.0f,
                            rgb_val.g / 65535.0f,
                            rgb_val.b / 65535.0f,
                            1.0f);
                }
            }
        }

        // ---- 5. Compute bounds (centroid-relative, Godot Y-up) --------------
        // Bounds: negating LAS Y swaps min/max on the Godot Z axis.
        Vector3 gmin = Vector3(
                (float)(hdr.minx - cx),
                (float)(hdr.minz - cz),
                -(float)(hdr.maxy - cy));  // negate: LAS Y max → Godot Z min
        Vector3 gmax = Vector3(
                (float)(hdr.maxx - cx),
                (float)(hdr.maxz - cz),
                -(float)(hdr.miny - cy)); // negate: LAS Y min → Godot Z max
        _bounds      = AABB(gmin, gmax - gmin);
        _point_count = (int64_t)count;
        _load_time_ms = (float)((Time::get_singleton()->get_ticks_usec() - t0) / 1000.0);

        // ---- 6. CRS WKT from LASF_Projection VLR (record 2112) ---------------
        // LAS 1.4 spec §2.7: user_id "LASF_Projection", record_id 2112 carries
        // an OGC WKT coordinate reference system string.
        // We read it verbatim — no reprojection is done. Both the point cloud
        // and the IFCGeoreference must share the same projected CRS.
        _crs_wkt = String();
        try {
            std::vector<char> wkt_raw = mf.vlrData("LASF_Projection", 2112);
            if (!wkt_raw.empty()) {
                // The VLR payload is a null-terminated (or padded) ASCII/UTF-8 string.
                std::string wkt_str(wkt_raw.begin(), wkt_raw.end());
                // Trim to the first null character.
                auto null_pos = wkt_str.find('\0');
                if (null_pos != std::string::npos)
                    wkt_str.resize(null_pos);
                _crs_wkt = String(wkt_str.c_str());
            }
        } catch (...) {
            // vlrData throws if the VLR is not found in some implementations;
            // silently leave _crs_wkt empty.
        }
    _load_progress = 100;

    } catch (const std::exception &e) {
        ERR_PRINT(String("PointCloudReader: parse error — ") + e.what());
        _load_progress = 0;
        // Reset state
        _positions.resize(0);
        _colors.resize(0);
        _intensities.resize(0);
        _classifications.resize(0);
        _return_numbers.resize(0);
        _point_count = 0;
        _has_color   = false;
        return (int)ERR_PARSE_ERROR;
    }

    return (int)OK;
}

// ---- accessors --------------------------------------------------------------

PackedVector3Array PointCloudReader::get_positions() const     { return _positions; }
PackedColorArray   PointCloudReader::get_colors() const        { return _colors; }
PackedFloat32Array PointCloudReader::get_intensities() const   { return _intensities; }
PackedByteArray    PointCloudReader::get_classifications() const { return _classifications; }
PackedByteArray    PointCloudReader::get_return_numbers() const { return _return_numbers; }

int64_t PointCloudReader::get_point_count() const  { return _point_count; }
AABB    PointCloudReader::get_bounds() const        { return _bounds; }
Vector3 PointCloudReader::get_center() const        { return _center; }
bool    PointCloudReader::has_color() const         { return _has_color; }
String  PointCloudReader::get_source_path() const   { return _source_path; }
String  PointCloudReader::get_crs_wkt() const       { return _crs_wkt; }
int     PointCloudReader::get_point_format_id() const { return _point_format_id; }
float   PointCloudReader::get_load_time_ms() const  { return _load_time_ms; }
int     PointCloudReader::get_load_progress() const { return _load_progress; }

// ---- bindings ---------------------------------------------------------------

void PointCloudReader::_bind_methods() {
    ClassDB::bind_method(D_METHOD("load", "path"), &PointCloudReader::load);

    ClassDB::bind_method(D_METHOD("get_positions"),       &PointCloudReader::get_positions);
    ClassDB::bind_method(D_METHOD("get_colors"),          &PointCloudReader::get_colors);
    ClassDB::bind_method(D_METHOD("get_intensities"),     &PointCloudReader::get_intensities);
    ClassDB::bind_method(D_METHOD("get_classifications"), &PointCloudReader::get_classifications);
    ClassDB::bind_method(D_METHOD("get_return_numbers"),  &PointCloudReader::get_return_numbers);

    ClassDB::bind_method(D_METHOD("get_point_count"),     &PointCloudReader::get_point_count);
    ClassDB::bind_method(D_METHOD("get_bounds"),          &PointCloudReader::get_bounds);
    ClassDB::bind_method(D_METHOD("get_center"),          &PointCloudReader::get_center);
    ClassDB::bind_method(D_METHOD("has_color"),           &PointCloudReader::has_color);
    ClassDB::bind_method(D_METHOD("get_source_path"),     &PointCloudReader::get_source_path);
    ClassDB::bind_method(D_METHOD("get_crs_wkt"),         &PointCloudReader::get_crs_wkt);
    ClassDB::bind_method(D_METHOD("get_point_format_id"), &PointCloudReader::get_point_format_id);
    ClassDB::bind_method(D_METHOD("get_load_time_ms"),    &PointCloudReader::get_load_time_ms);
    ClassDB::bind_method(D_METHOD("get_load_progress"),   &PointCloudReader::get_load_progress);
}
