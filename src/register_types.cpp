#include "register_types.h"

#include "point_cloud_reader.h"
#include "point_cloud_node.h"

#include <gdextension_interface.h>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>

using namespace godot;

void initialize_point_cloud_module(ModuleInitializationLevel p_level) {
    if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
        return;
    }
    ClassDB::register_class<PointCloudReader>();
    ClassDB::register_class<PointCloudNode>();
}

void uninitialize_point_cloud_module(ModuleInitializationLevel p_level) {
    if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
        return;
    }
}

extern "C" {

GDExtensionBool GDE_EXPORT point_cloud_library_init(
        GDExtensionInterfaceGetProcAddress p_get_proc_address,
        GDExtensionClassLibraryPtr p_library,
        GDExtensionInitialization *r_initialization) {
    godot::GDExtensionBinding::InitObject init_obj(
            p_get_proc_address, p_library, r_initialization);
    init_obj.register_initializer(initialize_point_cloud_module);
    init_obj.register_terminator(uninitialize_point_cloud_module);
    init_obj.set_minimum_library_initialization_level(
            MODULE_INITIALIZATION_LEVEL_SCENE);
    return init_obj.init();
}

} // extern "C"
