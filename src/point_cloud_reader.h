#pragma once

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/packed_color_array.hpp>
#include <godot_cpp/variant/packed_float32_array.hpp>
#include <godot_cpp/variant/packed_byte_array.hpp>
#include <godot_cpp/variant/aabb.hpp>
#include <godot_cpp/variant/vector3.hpp>

using namespace godot;

/// Loads a LAS or LAZ point cloud file and exposes the raw point attributes.
///
/// Usage (GDScript):
///   var reader = PointCloudReader.new()
///   var err = reader.load("res://cloud.laz")
///   if err == OK:
///       var positions = reader.get_positions()   # PackedVector3Array
///       var colors    = reader.get_colors()       # PackedColorArray  (empty if no RGB)
///       var intensity = reader.get_intensities()  # PackedFloat32Array (0..1)
///
/// Coordinate convention (Godot Y-up):
///   Godot X = LAS X  (Easting)
///   Godot Y = LAS Z  (Elevation — becomes the up axis)
///   Godot Z = LAS Y  (Northing)
///
/// Positions are stored centroid-relative for float precision. The world-space
/// centroid is available via get_center(). PointCloudNode handles repositioning.
class PointCloudReader : public RefCounted {
    GDCLASS(PointCloudReader, RefCounted)

public:
    PointCloudReader();
    ~PointCloudReader() override = default;

    /// Load a .las or .laz file. Returns an Error code (OK on success).
    /// Uses Godot's FileAccess to read the raw bytes, then decompresses via
    /// laz-perf's memory reader — works on all platforms including Web/Android.
    int load(const String &p_path);

    // ---- raw attribute access -----------------------------------------------
    PackedVector3Array get_positions() const;
    PackedColorArray   get_colors() const;       ///< Empty if the file has no RGB
    PackedFloat32Array get_intensities() const;  ///< Normalised 0–1
    PackedByteArray    get_classifications() const;
    PackedByteArray    get_return_numbers() const;

    // ---- metadata -----------------------------------------------------------
    int64_t  get_point_count() const;
    AABB     get_bounds() const;     ///< In Godot space, centroid-relative
    Vector3  get_center() const;     ///< World-space centroid (use to reposition node)
    bool     has_color() const;      ///< True if the file contains RGB channels
    String   get_source_path() const;
    int      get_point_format_id() const;  ///< LAS point record format (0–10)
    float    get_load_time_ms() const;     ///< Wall-clock ms spent in load()
    int      get_load_progress() const;    ///< 0–100 while loading, 100 when done, 0 when idle

    /// WKT CRS string read from the LAS "LASF_Projection" VLR (record 2112).
    /// Empty when the file contains no CRS VLR (e.g. ungeoreferenced scan data).
    /// NOTE: no reprojection is performed — the point cloud coordinates are
    /// always used as-is. When applying IFCGeoreference, the user must ensure
    /// this CRS matches the IFC ProjectedCRS.
    String   get_crs_wkt() const;

protected:
    static void _bind_methods();

private:
    PackedVector3Array _positions;
    PackedColorArray   _colors;
    PackedFloat32Array _intensities;
    PackedByteArray    _classifications;
    PackedByteArray    _return_numbers;

    int64_t _point_count = 0;
    AABB    _bounds;
    Vector3 _center;
    bool    _has_color = false;
    String  _source_path;
    String  _crs_wkt;             ///< WKT CRS from LASF_Projection VLR 2112 (empty if absent)
    int     _point_format_id  = -1;
    float   _load_time_ms     = 0.0f;
    int     _load_progress    = 0;   ///< updated during load(); safe to read from another thread
};
