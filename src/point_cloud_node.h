#pragma once

#include "point_cloud_reader.h"

#include <vector>

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/shader.hpp>
#include <godot_cpp/variant/node_path.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/transform3d.hpp>

using namespace godot;

/// A Node3D that loads a LAS/LAZ point cloud and visualises it as a set of
/// spatially-bucketed ArrayMesh(PRIMITIVE_POINTS) children, enabling Godot's
/// built-in frustum culling per chunk.
///
/// Key properties (editable in the Inspector):
///   file_path              — path to the .las/.laz file (supports res:// and absolute)
///   point_size             — rendered point radius in pixels (default 2.0)
///   color_mode             — AUTO / RGB / INTENSITY / CLASSIFICATION
///   max_points_per_chunk   — target bucket size (default 100 000)
///   auto_center            — translate the node to the cloud's world centroid
///
/// Loading:
///   load()        — synchronous (blocks main thread; fine for small clouds)
///   load_async()  — dispatches parsing to WorkerThreadPool; emits signals
///
/// Signals:
///   loaded(point_count: int)   — fired on the main thread after mesh is built
///   load_failed(error: String) — fired if the file could not be parsed
///   load_progress(ratio: float)— fired periodically during async loading (0..1)
class PointCloudNode : public Node3D {
    GDCLASS(PointCloudNode, Node3D)

public:
    enum ColorMode {
        COLOR_MODE_AUTO           = 0, ///< Use RGB if available, else Intensity
        COLOR_MODE_RGB            = 1,
        COLOR_MODE_INTENSITY      = 2,
        COLOR_MODE_CLASSIFICATION = 3,
    };

    PointCloudNode();
    ~PointCloudNode() override = default;

    // ---- property setters / getters ----------------------------------------
    void    set_file_path(const String &p_path);
    String  get_file_path() const;

    void    set_point_size(float p_size);
    float   get_point_size() const;

    void      set_color_mode(ColorMode p_mode);
    ColorMode get_color_mode() const;

    void    set_max_points_per_chunk(int p_count);
    int     get_max_points_per_chunk() const;

    void    set_auto_center(bool p_enable);
    bool    get_auto_center() const;

    /// Path to an IFCGeoreference node in the scene tree.
    /// When set, load() / load_async() use the georef transform instead of
    /// auto-centering to the cloud's own centroid.
    void     set_georef_node_path(const NodePath &p_path);
    NodePath get_georef_node_path() const;

    void    set_show_chunk_bounds(bool p_show);
    bool    get_show_chunk_bounds() const;

    void  set_visibility_range_begin(float p_distance);
    float get_visibility_range_begin() const;

    void  set_visibility_range_end(float p_distance);
    float get_visibility_range_end() const;

    void  set_smooth_points_enabled(bool p_enabled);
    bool  get_smooth_points_enabled() const;

    void  set_smooth_edge_softness(float p_value);
    float get_smooth_edge_softness() const;

    void set_auto_load_on_ready(bool p_enabled);
    bool get_auto_load_on_ready() const;

    // ---- loading -----------------------------------------------------------
    void load();        ///< Synchronous load + mesh build (blocks main thread)
    void load_async();  ///< Async load via WorkerThreadPool

    // ---- data access -------------------------------------------------------
    Ref<PointCloudReader> get_reader() const;
    int    get_load_progress() const; ///< 0–100 during async load; 100 when idle
    float  get_build_time_ms() const;
    int    get_chunk_count() const;
    Array  get_chunk_aabbs() const;

    // ---- clipping ----------------------------------------------------------
    /// Apply an axis-aligned clip box in the node’s local coordinate space.
    /// @param clip_transform   Transform of the clip box in node-local space.
    /// @param half_extents     Half-sizes along each local axis of the clip box.
    void apply_clip(Transform3D clip_transform, Vector3 half_extents);
    void reset_clip();          ///< Remove clip and rebuild full mesh
    bool get_has_clip() const;  ///< Whether a clip is currently applied

protected:
    static void _bind_methods();
    void _notification(int p_what);

private:
    // ---- internal helpers --------------------------------------------------
    void _build_mesh();
    void _clear_chunks();
    void _update_chunk_bound_visuals();
    void _apply_render_settings_to_chunks();
    Ref<Material> _create_point_material();
    void _load_worker();          // runs on worker thread
    void _on_load_done();         // runs on main thread (deferred)
    void _on_load_failed(const String &p_error); // runs on main thread (deferred)

    // ---- state -------------------------------------------------------------
    String                _file_path;
    float                 _point_size           = 2.0f;
    ColorMode             _color_mode           = COLOR_MODE_AUTO;
    int                   _max_points_per_chunk = 100000;
    bool                  _auto_center          = true;
    NodePath              _georef_node_path;     ///< optional IFCGeoreference node
    bool                  _is_loading           = false;
    bool                  _show_chunk_bounds    = false;
    bool                  _has_clip             = false;
    bool                  _smooth_points_enabled = true;
    bool                  _auto_load_on_ready    = true;

    float                 _visibility_range_begin = 0.0f;
    float                 _visibility_range_end   = 0.0f;
    float                 _smooth_edge_softness   = 0.20f;

    float                 _build_time_ms        = 0.0f;
    int                   _chunk_count          = 0;
    Array                 _chunk_aabbs;

    std::vector<uint8_t>  _clip_mask;   ///< 1 = keep, 0 = discard; size == point_count when active

    Ref<PointCloudReader> _reader;
    Ref<Shader>           _smooth_point_shader;
};

VARIANT_ENUM_CAST(PointCloudNode::ColorMode)
