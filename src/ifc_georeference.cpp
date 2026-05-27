#include "gd_ifc_georeference.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/basis.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

using namespace godot;

// ---------------------------------------------------------------------------
// IFCGeoreference — implementation
// ---------------------------------------------------------------------------

void IFCGeoreference::init(const GeorreferenceData &data) {
    const MapConversion &mc  = data.map_conversion;
    const ProjectedCRS  &crs = data.projected_crs;

    eastings_          = (double)mc.Eastings;
    northings_         = (double)mc.Northings;
    orthogonal_height_ = (double)mc.OrthogonalHeight;
    x_axis_abscissa_   = (double)mc.XAxisAbscissa;
    x_axis_ordinate_   = (double)mc.XAxisOrdinate;
    scale_             = (double)mc.Scale;

    crs_name_        = crs.Name;
    crs_description_ = crs.Description;
    geodetic_datum_  = crs.GeodeticDatum;
    vertical_datum_  = crs.VerticalDatum;
}

Transform3D IFCGeoreference::compute_cloud_transform(Vector3 godot_center) const {
    // --- rotation parameters from MapConversion ---
    // XAxisAbscissa = cos(θ), XAxisOrdinate = sin(θ)
    // Default to identity rotation when both are zero (unset).
    double a = x_axis_abscissa_;
    double b = x_axis_ordinate_;
    if (std::abs(a) < 1e-12 && std::abs(b) < 1e-12) {
        a = 1.0; b = 0.0;
    } else {
        // Normalise to pure rotation (guard against rounding in source data).
        double len = std::sqrt(a * a + b * b);
        if (len > 1e-12) { a /= len; b /= len; }
    }

    // Scale: 0 means "not specified" → treat as 1.
    double S = (scale_ > 1e-12) ? scale_ : 1.0;

    // --- centroid expressed in LAS/CRS coordinates ---
    // godot_center comes from PointCloudReader::get_center():
    //   godot_center = (cx,  cz, -cy)   [Godot Y-up]
    // where (cx, cy, cz) is the LAS bounding-box centroid in CRS space.
    double cx  = (double)godot_center.x;   // Easting  centroid
    double cy  = -(double)godot_center.z;  // Northing centroid  (godot z = -las y)
    double cz  = (double)godot_center.y;   // Elevation centroid (godot y = las z)

    double E0 = eastings_;
    double N0 = northings_;
    double H0 = orthogonal_height_;

    // delta from georef origin to cloud centroid (CRS space)
    double dE = cx - E0;
    double dN = cy - N0;

    // --- debug ---
    UtilityFunctions::print(
        "[IFCGeoreference] compute_cloud_transform:"
        "  godot_center=", godot_center,
        "  CRS centroid: E=", cx, " N=", cy, " H=", cz,
        "  IFC origin: E0=", E0, " N0=", N0, " H0=", H0,
        "  dE=", dE, " dN=", dN,
        "  a(cos)=", a, " b(sin)=", b, " S=", S);

    // --- node origin (translation part of the transform) ---
    // Standard CRS → Godot axis convention:
    //   Godot X =  Easting  delta =  S*(a*dE + b*dN)
    //   Godot Y =  Elevation delta =  S*(cz − H0)
    //   Godot Z = −Northing delta = −S*(−b*dE + a*dN) = S*(b*dE − a*dN)
    //
    // Derivation: a world point at CRS (E, N, H) with cloud centroid at (Ec,Nc,Hc)
    // has centroid-relative pos (vx, vy, vz) = (E−Ec, H−Hc, −(N−Nc)).
    // Applying the MapConversion rotation and offset gives:
    //   world.x = S*(a*(E−E0) + b*(N−N0)) = S*(a*vx − b*vz) + S*(a*dE + b*dN)
    //   world.y = S*(H − H0)              =   S*vy           + S*(cz − H0)
    //   world.z = S*(b*(E−E0) − a*(N−N0)) = S*(b*vx + a*vz) + S*(b*dE − a*dN)
    // The basis handles the vertex part; origin carries the centroid offset.
    double origin_x =  S * (a * dE + b * dN);
    double origin_y =  S * (cz - H0);
    double origin_z =  S * (b * dE - a * dN);

    // --- basis (rotation + scale) ---
    // Maps centroid-relative Godot vertex V = (vx, vy, vz) to world:
    //   world.x = S*a*vx  + (−S*b)*vz   (= S*(a*vx − b*vz))
    //   world.y =  S*vy
    //   world.z = S*b*vx  +  S*a*vz     (= S*(b*vx + a*vz))
    //
    // Godot Basis columns are the images of X, Y, Z axes:
    Vector3 col_x((float)(S * a),  0.0f,        (float)(S * b));
    Vector3 col_y(0.0f,            (float)(S),  0.0f);
    Vector3 col_z((float)(-S * b), 0.0f,        (float)(S * a));

    Vector3 origin_v((float)origin_x, (float)origin_y, (float)origin_z);
    UtilityFunctions::print(
        "[IFCGeoreference] → computed origin=", origin_v);

    return Transform3D(
        Basis(col_x, col_y, col_z),
        origin_v);
}

void IFCGeoreference::_bind_methods() {
    // MapConversion
    ClassDB::bind_method(D_METHOD("set_eastings", "v"),          &IFCGeoreference::set_eastings);
    ClassDB::bind_method(D_METHOD("get_eastings"),               &IFCGeoreference::get_eastings);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "eastings"),       "set_eastings", "get_eastings");

    ClassDB::bind_method(D_METHOD("set_northings", "v"),         &IFCGeoreference::set_northings);
    ClassDB::bind_method(D_METHOD("get_northings"),              &IFCGeoreference::get_northings);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "northings"),      "set_northings", "get_northings");

    ClassDB::bind_method(D_METHOD("set_orthogonal_height", "v"), &IFCGeoreference::set_orthogonal_height);
    ClassDB::bind_method(D_METHOD("get_orthogonal_height"),      &IFCGeoreference::get_orthogonal_height);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "orthogonal_height"), "set_orthogonal_height", "get_orthogonal_height");

    ClassDB::bind_method(D_METHOD("set_x_axis_abscissa", "v"),  &IFCGeoreference::set_x_axis_abscissa);
    ClassDB::bind_method(D_METHOD("get_x_axis_abscissa"),       &IFCGeoreference::get_x_axis_abscissa);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "x_axis_abscissa"), "set_x_axis_abscissa", "get_x_axis_abscissa");

    ClassDB::bind_method(D_METHOD("set_x_axis_ordinate", "v"),  &IFCGeoreference::set_x_axis_ordinate);
    ClassDB::bind_method(D_METHOD("get_x_axis_ordinate"),       &IFCGeoreference::get_x_axis_ordinate);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "x_axis_ordinate"), "set_x_axis_ordinate", "get_x_axis_ordinate");

    ClassDB::bind_method(D_METHOD("set_scale", "v"),             &IFCGeoreference::set_scale);
    ClassDB::bind_method(D_METHOD("get_scale"),                  &IFCGeoreference::get_scale);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "scale"),          "set_scale", "get_scale");

    // ProjectedCRS
    ClassDB::bind_method(D_METHOD("set_crs_name", "v"),          &IFCGeoreference::set_crs_name);
    ClassDB::bind_method(D_METHOD("get_crs_name"),               &IFCGeoreference::get_crs_name);
    ADD_PROPERTY(PropertyInfo(Variant::STRING, "crs_name"),      "set_crs_name", "get_crs_name");

    ClassDB::bind_method(D_METHOD("set_crs_description", "v"),   &IFCGeoreference::set_crs_description);
    ClassDB::bind_method(D_METHOD("get_crs_description"),        &IFCGeoreference::get_crs_description);
    ADD_PROPERTY(PropertyInfo(Variant::STRING, "crs_description"), "set_crs_description", "get_crs_description");

    ClassDB::bind_method(D_METHOD("set_geodetic_datum", "v"),    &IFCGeoreference::set_geodetic_datum);
    ClassDB::bind_method(D_METHOD("get_geodetic_datum"),         &IFCGeoreference::get_geodetic_datum);
    ADD_PROPERTY(PropertyInfo(Variant::STRING, "geodetic_datum"), "set_geodetic_datum", "get_geodetic_datum");

    ClassDB::bind_method(D_METHOD("set_vertical_datum", "v"),    &IFCGeoreference::set_vertical_datum);
    ClassDB::bind_method(D_METHOD("get_vertical_datum"),         &IFCGeoreference::get_vertical_datum);
    ADD_PROPERTY(PropertyInfo(Variant::STRING, "vertical_datum"), "set_vertical_datum", "get_vertical_datum");

    // Transform helper
    ClassDB::bind_method(
        D_METHOD("compute_cloud_transform", "godot_center"),
        &IFCGeoreference::compute_cloud_transform);
}
