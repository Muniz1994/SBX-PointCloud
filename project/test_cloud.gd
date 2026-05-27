extends Node3D

## Quick test: loads the bundled example LAZ file and prints point count.
## Attach this script to the root Node3D of a scene.

@onready var cloud := PointCloudNode.new()

func _ready() -> void:
	add_child(cloud)

	# Adjust path to where your .laz file lives.
	# Using a relative project path requires the file to be imported or
	# placed inside the Godot project folder.  For a quick test you can
	# also use an absolute OS path:
	#   cloud.file_path = "C:/path/to/Merged_clouds.laz"
	cloud.file_path = "res://Merged_clouds.laz"
	cloud.point_size = 2.0
	cloud.color_mode = PointCloudNode.COLOR_MODE_AUTO
	cloud.auto_center = true

	cloud.loaded.connect(_on_loaded)
	cloud.load_failed.connect(_on_load_failed)

	print("PointCloud: starting async load…")
	cloud.load_async()

	# Add a basic camera so you can see the result
	var cam := Camera3D.new()
	cam.position = Vector3(0.0, 10.0, 30.0)
	cam.look_at(Vector3.ZERO, Vector3.UP)
	add_child(cam)


func _on_loaded(point_count: int) -> void:
	print("PointCloud loaded — %d points" % point_count)
	print("Bounds : ", cloud.get_reader().get_bounds())
	print("Center : ", cloud.get_reader().get_center())
	print("Has RGB: ", cloud.get_reader().has_color())

	# Example: read raw positions from GDScript
	var positions: PackedVector3Array = cloud.get_reader().get_positions()
	if positions.size() > 0:
		print("First point: ", positions[0])


func _on_load_failed(error: String) -> void:
	push_error("PointCloud load failed: " + error)
