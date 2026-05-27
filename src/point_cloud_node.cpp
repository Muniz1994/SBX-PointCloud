#include "point_cloud_node.h"

#include <cmath>
#include <vector>

#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/base_material3d.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/classes/time.hpp>
#include <godot_cpp/classes/worker_thread_pool.hpp>
#include <godot_cpp/core/error_macros.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

using namespace godot;

// ---------------------------------------------------------------------------
// Standard LAS classification colour palette (classes 0–15+)
// https://www.asprs.org/wp-content/uploads/2019/07/LAS_1_4_r15.pdf §2.3
// ---------------------------------------------------------------------------
static const Color CLASSIFICATION_COLORS[] = {
    Color(0.50f, 0.50f, 0.50f), // 0  Created / Never classified
    Color(0.70f, 0.70f, 0.70f), // 1  Unclassified
    Color(0.60f, 0.40f, 0.20f), // 2  Ground
    Color(0.55f, 0.80f, 0.30f), // 3  Low vegetation
    Color(0.25f, 0.65f, 0.10f), // 4  Medium vegetation
    Color(0.00f, 0.45f, 0.05f), // 5  High vegetation
    Color(0.90f, 0.75f, 0.10f), // 6  Building
    Color(0.80f, 0.10f, 0.10f), // 7  Low point / Noise
    Color(0.30f, 0.30f, 0.90f), // 8  Reserved (LAS 1.4 model key)
    Color(0.00f, 0.55f, 1.00f), // 9  Water
    Color(0.85f, 0.85f, 0.85f), // 10 Rail
    Color(0.95f, 0.55f, 0.15f), // 11 Road surface
    Color(0.70f, 0.00f, 0.70f), // 12 Reserved
    Color(1.00f, 0.20f, 0.80f), // 13 Wire – guard
    Color(1.00f, 0.60f, 0.00f), // 14 Wire – conductor
    Color(0.00f, 0.90f, 0.90f), // 15 Transmission tower
};
static const int NUM_CLASS_COLORS = (int)(sizeof(CLASSIFICATION_COLORS) / sizeof(CLASSIFICATION_COLORS[0]));

// ---------------------------------------------------------------------------
// PointCloudNode
// ---------------------------------------------------------------------------

PointCloudNode::PointCloudNode() {
    _reader.instantiate();
}

// ---- property accessors ----------------------------------------------------

void   PointCloudNode::set_file_path(const String &p_path) { _file_path = p_path; }
String PointCloudNode::get_file_path() const               { return _file_path; }

void  PointCloudNode::set_point_size(float p_size)         { _point_size = MAX(0.1f, p_size); }
float PointCloudNode::get_point_size() const               { return _point_size; }

void                   PointCloudNode::set_color_mode(ColorMode p_mode) { _color_mode = p_mode; }
PointCloudNode::ColorMode PointCloudNode::get_color_mode() const        { return _color_mode; }

void PointCloudNode::set_max_points_per_chunk(int p_count) { _max_points_per_chunk = MAX(1000, p_count); }
int  PointCloudNode::get_max_points_per_chunk() const      { return _max_points_per_chunk; }

void PointCloudNode::set_auto_center(bool p_enable)        { _auto_center = p_enable; }
bool PointCloudNode::get_auto_center() const               { return _auto_center; }

void     PointCloudNode::set_georef_node_path(const NodePath &p_path) { _georef_node_path = p_path; }
NodePath PointCloudNode::get_georef_node_path() const                 { return _georef_node_path; }

void PointCloudNode::set_show_chunk_bounds(bool p_show) {
    _show_chunk_bounds = p_show;
    // Remove existing debug visuals
    for (int i = get_child_count() - 1; i >= 0; i--) {
        Node *child = get_child(i);
        if (child->has_meta("__pc_debug")) {
            remove_child(child);
            child->queue_free();
        }
    }
    if (_show_chunk_bounds) {
        _update_chunk_bound_visuals();
    }
}
bool PointCloudNode::get_show_chunk_bounds() const { return _show_chunk_bounds; }

float PointCloudNode::get_build_time_ms() const { return _build_time_ms; }
int   PointCloudNode::get_chunk_count() const   { return _chunk_count; }
Array PointCloudNode::get_chunk_aabbs() const   { return _chunk_aabbs; }

Ref<PointCloudReader> PointCloudNode::get_reader() const   { return _reader; }

int PointCloudNode::get_load_progress() const {
    return _reader.is_valid() ? _reader->get_load_progress() : 0;
}

// ---- loading ---------------------------------------------------------------

void PointCloudNode::load() {
    if (_file_path.is_empty()) {
        ERR_PRINT("PointCloudNode: file_path is not set.");
        return;
    }
    _has_clip = false;
    _clip_mask.clear();
    _clear_chunks();
    Error err = (Error)_reader->load(_file_path);
    if (err != OK) {
        emit_signal("load_failed", "Error " + String::num_int64((int64_t)err) + " loading: " + _file_path);
        return;
    }
    _build_mesh();
    emit_signal("loaded", _reader->get_point_count());
}

void PointCloudNode::load_async() {
    if (_is_loading) {
        WARN_PRINT("PointCloudNode: load_async() called while already loading.");
        return;
    }
    if (_file_path.is_empty()) {
        ERR_PRINT("PointCloudNode: file_path is not set.");
        return;
    }
    _is_loading = true;
    _clear_chunks();

    WorkerThreadPool::get_singleton()->add_task(
            callable_mp(this, &PointCloudNode::_load_worker),
            false,
            "PointCloudNode::load_async " + _file_path);
}

// ---- internal: worker thread -----------------------------------------------

void PointCloudNode::_load_worker() {
    Error err = (Error)_reader->load(_file_path);
    if (err != OK) {
        String msg = "Error " + String::num_int64((int64_t)err) + " loading: " + _file_path;
        callable_mp(this, &PointCloudNode::_on_load_failed).call_deferred(msg);
    } else {
        callable_mp(this, &PointCloudNode::_on_load_done).call_deferred();
    }
}

void PointCloudNode::_on_load_done() {
    _is_loading = false;
    _has_clip   = false;
    _clip_mask.clear();
    _build_mesh();
    emit_signal("loaded", _reader->get_point_count());
}

void PointCloudNode::_on_load_failed(const String &p_error) {
    _is_loading = false;
    ERR_PRINT("PointCloudNode: " + p_error);
    emit_signal("load_failed", p_error);
}

// ---- internal: mesh building -----------------------------------------------

void PointCloudNode::_clear_chunks() {
    // Remove all MeshInstance3D children (point cloud chunks + debug bounds)
    for (int i = get_child_count() - 1; i >= 0; i--) {
        Node *child = get_child(i);
        if (Object::cast_to<MeshInstance3D>(child)) {
            remove_child(child);
            child->queue_free();
        }
    }
    _chunk_count = 0;
    _chunk_aabbs.clear();
}

void PointCloudNode::_build_mesh() {
    const uint64_t t0 = Time::get_singleton()->get_ticks_usec();

    int64_t count = _reader->get_point_count();
    if (count == 0) {
        WARN_PRINT("PointCloudNode: point cloud has 0 points.");
        return;
    }

    const PackedVector3Array &positions   = _reader->get_positions();
    const PackedColorArray   &rgb_data    = _reader->get_colors();
    const PackedFloat32Array &intensities = _reader->get_intensities();
    const PackedByteArray    &classes     = _reader->get_classifications();
    const bool                file_has_rgb = _reader->has_color();

    // ---- Resolve effective colour mode -------------------------------------
    ColorMode effective = _color_mode;
    if (effective == COLOR_MODE_AUTO) {
        effective = file_has_rgb ? COLOR_MODE_RGB : COLOR_MODE_INTENSITY;
    }

    // ---- Positioning: georef or auto-center --------------------------------
    // Prefer IFCGeoreference when a node path is set and the node exists.
    bool georef_applied = false;
    if (!_georef_node_path.is_empty()) {
        Node *n = get_node_or_null(_georef_node_path);
        // Use name-based check so we interoperate with the IFCGeoreference
        // registered by the GDIFC extension, without owning/duplicating it.
        bool is_georef = n != nullptr && n->is_class("IFCGeoreference");
        if (is_georef) {
            // ---- CRS compatibility check -----------------------------------
            // The point cloud must be in the same CRS as the IFCGeoreference.
            // We cannot reproject automatically (no PROJ dependency), so we
            // log a warning when the LAS WKT CRS and the IFC ProjectedCRS name
            // are both present and clearly differ.
            const String las_wkt   = _reader->get_crs_wkt();
            const String ifc_name  = n->call("get_crs_name");
            if (!las_wkt.is_empty() && !ifc_name.is_empty() &&
                ifc_name != "NotDefined") {
                // Compare by checking whether the IFC CRS name appears anywhere
                // in the LAS WKT string (case-insensitive, tolerant of formatting).
                if (las_wkt.findn(ifc_name) == -1) {
                    WARN_PRINT(vformat(
                        "PointCloudNode: CRS mismatch — LAS file CRS does not "
                        "contain IFC CRS name \"%s\". The point cloud may be in "
                        "a different projected CRS. No reprojection is applied; "
                        "coordinates are used as-is. Verify both are in the same "
                        "CRS before using georef alignment.", ifc_name));
                }
            }
            if (las_wkt.is_empty()) {
                WARN_PRINT("PointCloudNode: LAS file contains no CRS VLR (record 2112). "
                           "Cannot verify that the point cloud is in the same CRS as "
                           "the IFCGeoreference. Assuming they match.");
            }

            set_transform(n->call("compute_cloud_transform", _reader->get_center()));
            georef_applied = true;
        } else {
            WARN_PRINT("PointCloudNode: georef_node_path does not point to an IFCGeoreference node.");
        }
    }
    if (!georef_applied && _auto_center) {
        set_position(_reader->get_center());
    }

    // ---- Spatial grid chunking --------------------------------------------
    // We divide the bounding box into a 3D grid so each cell contains at most
    // ~_max_points_per_chunk points.  Each cell becomes one ArrayMesh child,
    // giving Godot's renderer a set of AABB-cullable draw calls.

    AABB bounds = _reader->get_bounds();
    Vector3 extent = bounds.size;

    // Guard against degenerate flat clouds
    if (extent.x < 1e-4f) extent.x = 1.0f;
    if (extent.y < 1e-4f) extent.y = 1.0f;
    if (extent.z < 1e-4f) extent.z = 1.0f;

    // When a clip is active, count only visible points for chunk sizing.
    int64_t visible_count = count;
    if (_has_clip && (int64_t)_clip_mask.size() == count) {
        visible_count = 0;
        for (size_t i = 0; i < (size_t)count; i++) {
            if (_clip_mask[i]) visible_count++;
        }
    }

    // Choose grid_dim so a uniform distribution would give _max_points_per_chunk per cell
    int64_t expected_chunks = MAX((int64_t)1, (int64_t)std::ceil((double)visible_count / (double)_max_points_per_chunk));
    int grid_dim = (int)std::ceil(std::cbrt((double)expected_chunks));
    grid_dim = CLAMP(grid_dim, 1, 32); // 32^3 = 32768 max cells

    int total_cells = grid_dim * grid_dim * grid_dim;

    // Pass 1: assign every point to its grid cell
    std::vector<std::vector<int64_t>> cells((size_t)total_cells);
    {
        const Vector3 *pos_r = positions.ptr();
        const Vector3  bmin  = bounds.position;
        const float    inv_x = (float)grid_dim / extent.x;
        const float    inv_y = (float)grid_dim / extent.y;
        const float    inv_z = (float)grid_dim / extent.z;

        for (int64_t i = 0; i < count; i++) {
            if (_has_clip && (int64_t)_clip_mask.size() == count && !_clip_mask[(size_t)i]) continue;
            const Vector3 &p = pos_r[i];
            int cx = CLAMP((int)((p.x - bmin.x) * inv_x), 0, grid_dim - 1);
            int cy = CLAMP((int)((p.y - bmin.y) * inv_y), 0, grid_dim - 1);
            int cz = CLAMP((int)((p.z - bmin.z) * inv_z), 0, grid_dim - 1);
            cells[(size_t)(cx + grid_dim * (cy + grid_dim * cz))].push_back(i);
        }
    }

    // Pass 2: build one ArrayMesh per non-empty cell
    const Vector3 *pos_r = positions.ptr();
    const Color   *rgb_r = file_has_rgb ? rgb_data.ptr()    : nullptr;
    const float   *int_r = intensities.ptr();
    const uint8_t *cls_r = classes.ptr();

    for (int cell_idx = 0; cell_idx < total_cells; cell_idx++) {
        const std::vector<int64_t> &indices = cells[(size_t)cell_idx];
        if (indices.empty()) continue;

        // Compute cell AABB for debug visualization
        {
            int cz_cell = cell_idx / (grid_dim * grid_dim);
            int cy_cell = (cell_idx / grid_dim) % grid_dim;
            int cx_cell = cell_idx % grid_dim;
            Vector3 cell_min = bounds.position + Vector3(
                cx_cell * extent.x / grid_dim,
                cy_cell * extent.y / grid_dim,
                cz_cell * extent.z / grid_dim);
            Vector3 cell_size = Vector3(
                extent.x / grid_dim,
                extent.y / grid_dim,
                extent.z / grid_dim);
            _chunk_aabbs.push_back(AABB(cell_min, cell_size));
        }
        _chunk_count++;

        int64_t cell_count = (int64_t)indices.size();

        PackedVector3Array verts;
        PackedColorArray   colors;
        verts.resize(cell_count);
        colors.resize(cell_count);

        Vector3 *v_w = verts.ptrw();
        Color   *c_w = colors.ptrw();

        for (int64_t j = 0; j < cell_count; j++) {
            int64_t idx = indices[(size_t)j];
            v_w[j] = pos_r[idx];

            switch (effective) {
                case COLOR_MODE_RGB: {
                    c_w[j] = rgb_r ? rgb_r[idx] : Color(int_r[idx], int_r[idx], int_r[idx], 1.0f);
                    break;
                }
                case COLOR_MODE_INTENSITY: {
                    float v = int_r[idx];
                    c_w[j] = Color(v, v, v, 1.0f);
                    break;
                }
                case COLOR_MODE_CLASSIFICATION: {
                    int cls = (int)cls_r[idx];
                    c_w[j]  = (cls < NUM_CLASS_COLORS) ? CLASSIFICATION_COLORS[cls]
                                                       : Color(1.0f, 1.0f, 1.0f, 1.0f);
                    break;
                }
                default:
                    c_w[j] = Color(1.0f, 1.0f, 1.0f, 1.0f);
                    break;
            }
        }

        // Build ArrayMesh
        Array arrays;
        arrays.resize(Mesh::ARRAY_MAX);
        arrays[Mesh::ARRAY_VERTEX] = verts;
        arrays[Mesh::ARRAY_COLOR]  = colors;

        Ref<ArrayMesh> mesh;
        mesh.instantiate();
        mesh->add_surface_from_arrays(Mesh::PRIMITIVE_POINTS, arrays);

        // Unshaded material with vertex colours and configurable point size
        Ref<StandardMaterial3D> mat;
        mat.instantiate();
        mat->set_shading_mode(BaseMaterial3D::SHADING_MODE_UNSHADED);
        mat->set_flag(BaseMaterial3D::FLAG_ALBEDO_FROM_VERTEX_COLOR, true);
        mat->set_flag(BaseMaterial3D::FLAG_SRGB_VERTEX_COLOR, true);  // scanner RGB is sRGB; linearise before render
        mat->set_flag(BaseMaterial3D::FLAG_USE_POINT_SIZE, true);
        mat->set_point_size(_point_size);
        mesh->surface_set_material(0, mat);

        // Add MeshInstance3D child — one per spatial cell for frustum culling
        MeshInstance3D *mi = memnew(MeshInstance3D);
        mi->set_mesh(mesh);
        add_child(mi);
    }

    _build_time_ms = (float)((Time::get_singleton()->get_ticks_usec() - t0) / 1000.0);

    if (_show_chunk_bounds) {
        _update_chunk_bound_visuals();
    }
}

// ---- clipping --------------------------------------------------------------

void PointCloudNode::apply_clip(Transform3D clip_transform, Vector3 half_extents) {
    if (!_reader.is_valid()) return;
    int64_t count = _reader->get_point_count();
    if (count == 0) return;

    const PackedVector3Array &positions = _reader->get_positions();
    const Vector3 *pos_r = positions.ptr();

    _clip_mask.resize((size_t)count);

    // Pre-compute the inverse once for efficiency.
    Transform3D inv = clip_transform.affine_inverse();

    for (int64_t i = 0; i < count; i++) {
        Vector3 p = inv.xform(pos_r[i]);
        _clip_mask[(size_t)i] = (
            Math::abs(p.x) <= half_extents.x &&
            Math::abs(p.y) <= half_extents.y &&
            Math::abs(p.z) <= half_extents.z
        ) ? 1 : 0;
    }
    _has_clip = true;
    _clear_chunks();
    _build_mesh();
}

void PointCloudNode::reset_clip() {
    _has_clip = false;
    _clip_mask.clear();
    _clear_chunks();
    _build_mesh();
}

bool PointCloudNode::get_has_clip() const { return _has_clip; }

// ---- debug wireframe -------------------------------------------------------

void PointCloudNode::_update_chunk_bound_visuals() {
    for (int i = 0; i < _chunk_aabbs.size(); i++) {
        AABB aabb = _chunk_aabbs[i];
        Vector3 a  = aabb.position;
        Vector3 b  = aabb.position + aabb.size;

        PackedVector3Array verts;
        verts.resize(24);
        Vector3 *v = verts.ptrw();
        // Bottom face (y = a.y)
        v[0]  = Vector3(a.x, a.y, a.z); v[1]  = Vector3(b.x, a.y, a.z);
        v[2]  = Vector3(b.x, a.y, a.z); v[3]  = Vector3(b.x, a.y, b.z);
        v[4]  = Vector3(b.x, a.y, b.z); v[5]  = Vector3(a.x, a.y, b.z);
        v[6]  = Vector3(a.x, a.y, b.z); v[7]  = Vector3(a.x, a.y, a.z);
        // Top face (y = b.y)
        v[8]  = Vector3(a.x, b.y, a.z); v[9]  = Vector3(b.x, b.y, a.z);
        v[10] = Vector3(b.x, b.y, a.z); v[11] = Vector3(b.x, b.y, b.z);
        v[12] = Vector3(b.x, b.y, b.z); v[13] = Vector3(a.x, b.y, b.z);
        v[14] = Vector3(a.x, b.y, b.z); v[15] = Vector3(a.x, b.y, a.z);
        // Vertical edges
        v[16] = Vector3(a.x, a.y, a.z); v[17] = Vector3(a.x, b.y, a.z);
        v[18] = Vector3(b.x, a.y, a.z); v[19] = Vector3(b.x, b.y, a.z);
        v[20] = Vector3(b.x, a.y, b.z); v[21] = Vector3(b.x, b.y, b.z);
        v[22] = Vector3(a.x, a.y, b.z); v[23] = Vector3(a.x, b.y, b.z);

        Array arrays;
        arrays.resize(Mesh::ARRAY_MAX);
        arrays[Mesh::ARRAY_VERTEX] = verts;

        Ref<ArrayMesh> wire_mesh;
        wire_mesh.instantiate();
        wire_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_LINES, arrays);

        Ref<StandardMaterial3D> wire_mat;
        wire_mat.instantiate();
        wire_mat->set_shading_mode(BaseMaterial3D::SHADING_MODE_UNSHADED);
        wire_mat->set_albedo(Color(1.0f, 0.6f, 0.0f, 1.0f)); // amber
        wire_mesh->surface_set_material(0, wire_mat);

        MeshInstance3D *mi = memnew(MeshInstance3D);
        mi->set_mesh(wire_mesh);
        mi->set_meta("__pc_debug", true);
        add_child(mi);
    }
}

// ---- bindings --------------------------------------------------------------

void PointCloudNode::_bind_methods() {
    // Properties
    ClassDB::bind_method(D_METHOD("set_file_path", "path"), &PointCloudNode::set_file_path);
    ClassDB::bind_method(D_METHOD("get_file_path"),         &PointCloudNode::get_file_path);
    ADD_PROPERTY(PropertyInfo(Variant::STRING, "file_path",
                              PROPERTY_HINT_FILE, "*.las,*.laz"),
                 "set_file_path", "get_file_path");

    ClassDB::bind_method(D_METHOD("set_point_size", "size"), &PointCloudNode::set_point_size);
    ClassDB::bind_method(D_METHOD("get_point_size"),          &PointCloudNode::get_point_size);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "point_size",
                              PROPERTY_HINT_RANGE, "0.1,32.0,0.1"),
                 "set_point_size", "get_point_size");

    ClassDB::bind_method(D_METHOD("set_color_mode", "mode"), &PointCloudNode::set_color_mode);
    ClassDB::bind_method(D_METHOD("get_color_mode"),          &PointCloudNode::get_color_mode);
    ADD_PROPERTY(PropertyInfo(Variant::INT, "color_mode",
                              PROPERTY_HINT_ENUM,
                              "Auto,RGB,Intensity,Classification"),
                 "set_color_mode", "get_color_mode");

    ClassDB::bind_method(D_METHOD("set_max_points_per_chunk", "count"), &PointCloudNode::set_max_points_per_chunk);
    ClassDB::bind_method(D_METHOD("get_max_points_per_chunk"),           &PointCloudNode::get_max_points_per_chunk);
    ADD_PROPERTY(PropertyInfo(Variant::INT, "max_points_per_chunk",
                              PROPERTY_HINT_RANGE, "1000,1000000,1000"),
                 "set_max_points_per_chunk", "get_max_points_per_chunk");

    ClassDB::bind_method(D_METHOD("set_auto_center", "enable"), &PointCloudNode::set_auto_center);
    ClassDB::bind_method(D_METHOD("get_auto_center"),            &PointCloudNode::get_auto_center);
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "auto_center"),
                 "set_auto_center", "get_auto_center");

    ClassDB::bind_method(D_METHOD("set_georef_node_path", "path"), &PointCloudNode::set_georef_node_path);
    ClassDB::bind_method(D_METHOD("get_georef_node_path"),          &PointCloudNode::get_georef_node_path);
    ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "georef_node_path",
                              PROPERTY_HINT_NODE_PATH_VALID_TYPES, "IFCGeoreference"),
                 "set_georef_node_path", "get_georef_node_path");

    ClassDB::bind_method(D_METHOD("set_show_chunk_bounds", "show"), &PointCloudNode::set_show_chunk_bounds);
    ClassDB::bind_method(D_METHOD("get_show_chunk_bounds"),          &PointCloudNode::get_show_chunk_bounds);
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "show_chunk_bounds"),
                 "set_show_chunk_bounds", "get_show_chunk_bounds");

    // Methods
    ClassDB::bind_method(D_METHOD("load"),             &PointCloudNode::load);
    ClassDB::bind_method(D_METHOD("load_async"),       &PointCloudNode::load_async);
    ClassDB::bind_method(D_METHOD("get_reader"),        &PointCloudNode::get_reader);
    ClassDB::bind_method(D_METHOD("get_load_progress"), &PointCloudNode::get_load_progress);
    ClassDB::bind_method(D_METHOD("get_build_time_ms"), &PointCloudNode::get_build_time_ms);
    ClassDB::bind_method(D_METHOD("get_chunk_count"),   &PointCloudNode::get_chunk_count);
    ClassDB::bind_method(D_METHOD("get_chunk_aabbs"),   &PointCloudNode::get_chunk_aabbs);

    ClassDB::bind_method(D_METHOD("apply_clip", "clip_transform", "half_extents"), &PointCloudNode::apply_clip);
    ClassDB::bind_method(D_METHOD("reset_clip"),        &PointCloudNode::reset_clip);
    ClassDB::bind_method(D_METHOD("get_has_clip"),      &PointCloudNode::get_has_clip);

    // Signals
    ADD_SIGNAL(MethodInfo("loaded",
                          PropertyInfo(Variant::INT, "point_count")));
    ADD_SIGNAL(MethodInfo("load_failed",
                          PropertyInfo(Variant::STRING, "error")));
    ADD_SIGNAL(MethodInfo("load_progress",
                          PropertyInfo(Variant::FLOAT, "ratio")));

    // Enum constants
    BIND_ENUM_CONSTANT(COLOR_MODE_AUTO);
    BIND_ENUM_CONSTANT(COLOR_MODE_RGB);
    BIND_ENUM_CONSTANT(COLOR_MODE_INTENSITY);
    BIND_ENUM_CONSTANT(COLOR_MODE_CLASSIFICATION);
}
