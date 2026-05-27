@tool
extends EditorPlugin

const _DockScript = preload("res://addons/point_cloud/point_cloud_dock.gd")

var _dock: Control


func _enter_tree() -> void:
	_dock = _DockScript.new()
	_dock.name = "PointCloud"
	_dock.editor_interface = get_editor_interface()
	add_control_to_dock(DOCK_SLOT_RIGHT_UL, _dock)


func _exit_tree() -> void:
	if _dock:
		remove_control_from_docks(_dock)
		_dock.queue_free()
		_dock = null
