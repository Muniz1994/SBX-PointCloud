## point_cloud_dock.gd
## Editor dock for importing, previewing and configuring LAS/LAZ point clouds.
@tool
extends Control

## Set by plugin.gd immediately after construction.
var editor_interface: EditorInterface

# ------------------------------------------------------------------ state ---
var _reader: PointCloudReader = null
var _active_node: PointCloudNode = null
var _clip_box_node: Node3D = null
var _georef_node: Node = null  # selected IFCGeoreference node

# ------------------------------------------------------------------ UI refs -
var _path_edit:        LineEdit
var _status_label:     Label
var _progress_bar:     ProgressBar

var _lbl_points:       Label
var _lbl_format:       Label
var _lbl_rgb:          Label
var _lbl_size:         Label
var _lbl_center:       Label
var _lbl_load_time:    Label

var _spin_point_size:  SpinBox
var _opt_color_mode:   OptionButton
var _spin_max_chunk:   SpinBox
var _chk_auto_center:  CheckButton

var _lbl_georef_node:  Label
var _btn_pick_georef:  Button
var _btn_clear_georef: Button
var _lbl_las_crs:      Label   # WKT CRS read from the LAS VLR
var _lbl_ifc_crs:      Label   # CRS name from IFCGeoreference
var _lbl_crs_match:    Label   # match / mismatch status

var _chk_bounds:       CheckButton
var _lbl_chunks:       Label
var _lbl_total_pts:    Label
var _lbl_build_time:   Label

var _btn_create_clip:  Button
var _btn_apply_clip:   Button
var _btn_reset_clip:   Button
var _spin_clip_x:      SpinBox
var _spin_clip_y:      SpinBox
var _spin_clip_z:      SpinBox
var _lbl_clip_status:  Label

var _poll_timer:       Timer
var _preview_thread:   Thread = null


# ------------------------------------------------------------------ build ---
func _init() -> void:
	custom_minimum_size = Vector2(220, 0)
	_build_ui()


func _build_ui() -> void:
	var scroll := ScrollContainer.new()
	scroll.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	scroll.horizontal_scroll_mode = ScrollContainer.SCROLL_MODE_DISABLED
	add_child(scroll)

	var vbox := VBoxContainer.new()
	vbox.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	scroll.add_child(vbox)

	# ---- File section -------------------------------------------------------
	vbox.add_child(_section_label("File"))

	var hpath := HBoxContainer.new()
	vbox.add_child(hpath)

	_path_edit = LineEdit.new()
	_path_edit.placeholder_text = "res://my_cloud.laz"
	_path_edit.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	hpath.add_child(_path_edit)

	var btn_browse := Button.new()
	btn_browse.text = "..."
	btn_browse.tooltip_text = "Browse for a .las or .laz file"
	btn_browse.pressed.connect(_on_browse_pressed)
	hpath.add_child(btn_browse)

	var btn_preview := Button.new()
	btn_preview.text = "Load Preview"
	btn_preview.tooltip_text = "Read file and fill info panel"
	btn_preview.pressed.connect(_on_load_preview_pressed)
	vbox.add_child(btn_preview)

	var btn_add := Button.new()
	btn_add.text = "Add to Scene"
	btn_add.tooltip_text = "Create PointCloudNode and start async load"
	btn_add.pressed.connect(_on_add_to_scene_pressed)
	vbox.add_child(btn_add)

	_progress_bar = ProgressBar.new()
	_progress_bar.min_value = 0
	_progress_bar.max_value = 100
	_progress_bar.value = 0
	_progress_bar.visible = false
	_progress_bar.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	vbox.add_child(_progress_bar)

	_status_label = Label.new()
	_status_label.add_theme_color_override("font_color", Color(0.7, 0.7, 0.7))
	_status_label.text = "No file loaded."
	_status_label.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	vbox.add_child(_status_label)

	vbox.add_child(HSeparator.new())

	# ---- Cloud Info section -------------------------------------------------
	vbox.add_child(_section_label("Cloud Info"))

	var info_grid := GridContainer.new()
	info_grid.columns = 2
	info_grid.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	vbox.add_child(info_grid)

	_lbl_points    = _add_info_row(info_grid, "Points:",    "--")
	_lbl_format    = _add_info_row(info_grid, "Format:",    "--")
	_lbl_rgb       = _add_info_row(info_grid, "RGB:",       "--")
	_lbl_size      = _add_info_row(info_grid, "Size (m):",  "--")
	_lbl_center    = _add_info_row(info_grid, "Center:",    "--")
	_lbl_load_time = _add_info_row(info_grid, "Load time:", "--")

	vbox.add_child(HSeparator.new())

	# ---- Rendering section --------------------------------------------------
	vbox.add_child(_section_label("Rendering"))

	var r_grid := GridContainer.new()
	r_grid.columns = 2
	r_grid.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	vbox.add_child(r_grid)

	r_grid.add_child(_small_label("Point size:"))
	_spin_point_size = SpinBox.new()
	_spin_point_size.min_value = 0.1
	_spin_point_size.max_value = 32.0
	_spin_point_size.step = 0.5
	_spin_point_size.value = 2.0
	_spin_point_size.value_changed.connect(_on_point_size_changed)
	r_grid.add_child(_spin_point_size)

	r_grid.add_child(_small_label("Color mode:"))
	_opt_color_mode = OptionButton.new()
	_opt_color_mode.add_item("Auto",           0)
	_opt_color_mode.add_item("RGB",            1)
	_opt_color_mode.add_item("Intensity",      2)
	_opt_color_mode.add_item("Classification", 3)
	_opt_color_mode.item_selected.connect(_on_color_mode_selected)
	r_grid.add_child(_opt_color_mode)

	r_grid.add_child(_small_label("Max pts/chunk:"))
	_spin_max_chunk = SpinBox.new()
	_spin_max_chunk.min_value = 1000
	_spin_max_chunk.max_value = 1000000
	_spin_max_chunk.step = 10000
	_spin_max_chunk.value = 100000
	_spin_max_chunk.value_changed.connect(_on_max_chunk_changed)
	r_grid.add_child(_spin_max_chunk)

	r_grid.add_child(_small_label("Auto-center:"))
	_chk_auto_center = CheckButton.new()
	_chk_auto_center.button_pressed = true
	_chk_auto_center.toggled.connect(_on_auto_center_toggled)
	r_grid.add_child(_chk_auto_center)

	vbox.add_child(HSeparator.new())

	# ---- Georeferencing section ---------------------------------------------
	vbox.add_child(_section_label("Georeferencing"))

	_lbl_georef_node = Label.new()
	_lbl_georef_node.text = "(none)"
	_lbl_georef_node.add_theme_color_override("font_color", Color(0.6, 0.9, 1.0))
	_lbl_georef_node.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_lbl_georef_node.clip_text = true
	_lbl_georef_node.tooltip_text = "IFCGeoreference node used for CRS alignment"
	vbox.add_child(_lbl_georef_node)

	var georef_btns := HBoxContainer.new()
	vbox.add_child(georef_btns)

	_btn_pick_georef = Button.new()
	_btn_pick_georef.text = "Pick from Tree"
	_btn_pick_georef.tooltip_text = "Select an IFCGeoreference node from the scene tree"
	_btn_pick_georef.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_btn_pick_georef.pressed.connect(_on_pick_georef_pressed)
	georef_btns.add_child(_btn_pick_georef)

	_btn_clear_georef = Button.new()
	_btn_clear_georef.text = "Clear"
	_btn_clear_georef.tooltip_text = "Remove the CRS reference (use auto-center instead)"
	_btn_clear_georef.pressed.connect(_on_clear_georef_pressed)
	georef_btns.add_child(_btn_clear_georef)

	# CRS comparison rows
	var crs_grid := GridContainer.new()
	crs_grid.columns = 2
	crs_grid.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	vbox.add_child(crs_grid)

	_lbl_las_crs = _add_info_row(crs_grid, "LAS CRS:", "(load a file first)")
	_lbl_las_crs.tooltip_text = \
		"Coordinate Reference System read from LASF_Projection VLR (record 2112). " + \
		"Empty if the file has no CRS record."

	_lbl_ifc_crs = _add_info_row(crs_grid, "IFC CRS:", "(pick georef)")
	_lbl_ifc_crs.tooltip_text = "ProjectedCRS.Name from the selected IFCGeoreference node."

	_lbl_crs_match = Label.new()
	_lbl_crs_match.text = ""
	_lbl_crs_match.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	_lbl_crs_match.add_theme_color_override("font_color", Color(0.7, 0.7, 0.7))
	vbox.add_child(_lbl_crs_match)

	var btn_rebuild := Button.new()
	btn_rebuild.text = "Rebuild Mesh"
	btn_rebuild.tooltip_text = "Re-apply render settings and rebuild the mesh"
	btn_rebuild.pressed.connect(_on_rebuild_pressed)
	vbox.add_child(btn_rebuild)

	vbox.add_child(HSeparator.new())

	# ---- Clip Box section ---------------------------------------------------
	vbox.add_child(_section_label("Clip Box"))

	_btn_create_clip = Button.new()
	_btn_create_clip.text = "Create Clip Box"
	_btn_create_clip.tooltip_text = "Spawn a resizable hollow prism to define the clipping region"
	_btn_create_clip.pressed.connect(_on_create_clip_box_pressed)
	vbox.add_child(_btn_create_clip)

	var size_grid := GridContainer.new()
	size_grid.columns = 2
	size_grid.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	vbox.add_child(size_grid)

	size_grid.add_child(_small_label("Half-size X:"))
	_spin_clip_x = _make_clip_spin()
	_spin_clip_x.value_changed.connect(_on_clip_size_changed)
	size_grid.add_child(_spin_clip_x)

	size_grid.add_child(_small_label("Half-size Y:"))
	_spin_clip_y = _make_clip_spin()
	_spin_clip_y.value_changed.connect(_on_clip_size_changed)
	size_grid.add_child(_spin_clip_y)

	size_grid.add_child(_small_label("Half-size Z:"))
	_spin_clip_z = _make_clip_spin()
	_spin_clip_z.value_changed.connect(_on_clip_size_changed)
	size_grid.add_child(_spin_clip_z)

	var clip_btns := HBoxContainer.new()
	vbox.add_child(clip_btns)

	_btn_apply_clip = Button.new()
	_btn_apply_clip.text = "Apply Clip"
	_btn_apply_clip.tooltip_text = "Cut the cloud to points inside the clip box"
	_btn_apply_clip.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_btn_apply_clip.pressed.connect(_on_apply_clip_pressed)
	clip_btns.add_child(_btn_apply_clip)

	_btn_reset_clip = Button.new()
	_btn_reset_clip.text = "Reset Clip"
	_btn_reset_clip.tooltip_text = "Remove clip and restore all points"
	_btn_reset_clip.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_btn_reset_clip.pressed.connect(_on_reset_clip_pressed)
	clip_btns.add_child(_btn_reset_clip)

	_lbl_clip_status = Label.new()
	_lbl_clip_status.add_theme_color_override("font_color", Color(0.6, 0.9, 1.0))
	_lbl_clip_status.text = "No clip box."
	_lbl_clip_status.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	vbox.add_child(_lbl_clip_status)

	vbox.add_child(HSeparator.new())

	# ---- Debug section ------------------------------------------------------
	vbox.add_child(_section_label("Debug"))

	var d_grid := GridContainer.new()
	d_grid.columns = 2
	d_grid.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	vbox.add_child(d_grid)

	d_grid.add_child(_small_label("Chunk bounds:"))
	_chk_bounds = CheckButton.new()
	_chk_bounds.toggled.connect(_on_show_bounds_toggled)
	d_grid.add_child(_chk_bounds)

	_lbl_chunks     = _add_info_row(d_grid, "Chunks:",     "--")
	_lbl_total_pts  = _add_info_row(d_grid, "Total pts:",  "--")
	_lbl_build_time = _add_info_row(d_grid, "Build time:", "--")

	var btn_stats := Button.new()
	btn_stats.text = "Print Stats to Output"
	btn_stats.tooltip_text = "Print full cloud statistics to the Output panel"
	btn_stats.pressed.connect(_on_print_stats_pressed)
	vbox.add_child(btn_stats)

	_poll_timer = Timer.new()
	_poll_timer.wait_time = 0.1
	_poll_timer.one_shot = false
	_poll_timer.timeout.connect(_on_progress_poll)
	add_child(_poll_timer)


# ---------------------------------------------------------------- helpers ---
static func _section_label(text: String) -> Label:
	var lbl := Label.new()
	lbl.text = text
	lbl.add_theme_font_size_override("font_size", 13)
	return lbl


static func _small_label(text: String) -> Label:
	var lbl := Label.new()
	lbl.text = text
	return lbl


func _add_info_row(grid: GridContainer, caption: String, initial: String) -> Label:
	grid.add_child(_small_label(caption))
	var val := Label.new()
	val.text = initial
	val.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	val.clip_text = true
	grid.add_child(val)
	return val


static func _make_clip_spin() -> SpinBox:
	var s := SpinBox.new()
	s.min_value = 0.1
	s.max_value = 100000.0
	s.step = 1.0
	s.value = 50.0
	s.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	return s


static func _fmt_int(n: int) -> String:
	var s := str(n)
	var result := ""
	var count := 0
	for i in range(s.length() - 1, -1, -1):
		if count > 0 and count % 3 == 0:
			result = "," + result
		result = s[i] + result
		count += 1
	return result


static func _fmt_vec3(v: Vector3) -> String:
	return "(%.1f, %.1f, %.1f)" % [v.x, v.y, v.z]


# ---------------------------------------------------- clip box wireframe ---
func _build_clip_box_wireframe(hx: float, hy: float, hz: float) -> ArrayMesh:
	var corners: Array[Vector3] = [
		Vector3(-hx, -hy, -hz), Vector3(+hx, -hy, -hz),
		Vector3(+hx, -hy, +hz), Vector3(-hx, -hy, +hz),
		Vector3(-hx, +hy, -hz), Vector3(+hx, +hy, -hz),
		Vector3(+hx, +hy, +hz), Vector3(-hx, +hy, +hz),
	]
	var edges: Array[int] = [
		0, 1,  1, 2,  2, 3,  3, 0,
		4, 5,  5, 6,  6, 7,  7, 4,
		0, 4,  1, 5,  2, 6,  3, 7,
	]
	var verts := PackedVector3Array()
	verts.resize(edges.size())
	for i in edges.size():
		verts[i] = corners[edges[i]]

	var arrays := Array()
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = verts

	var mat := StandardMaterial3D.new()
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	mat.albedo_color = Color(0.0, 0.85, 1.0)
	mat.no_depth_test = true

	var mesh := ArrayMesh.new()
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_LINES, arrays)
	mesh.surface_set_material(0, mat)
	return mesh


func _refresh_clip_box_mesh() -> void:
	if not is_instance_valid(_clip_box_node):
		return
	for child in _clip_box_node.get_children():
		_clip_box_node.remove_child(child)
		child.queue_free()
	var mi := MeshInstance3D.new()
	mi.mesh = _build_clip_box_wireframe(
		_spin_clip_x.value, _spin_clip_y.value, _spin_clip_z.value)
	mi.set_meta("__pc_clip_mesh", true)
	_clip_box_node.add_child(mi)


func _destroy_clip_box() -> void:
	if is_instance_valid(_clip_box_node):
		if is_instance_valid(_clip_box_node.get_parent()):
			_clip_box_node.get_parent().remove_child(_clip_box_node)
		_clip_box_node.queue_free()
	_clip_box_node = null


# -------------------------------------------------------- signal handlers ---
func _on_browse_pressed() -> void:
	var dialog := EditorFileDialog.new()
	dialog.access    = EditorFileDialog.ACCESS_FILESYSTEM
	dialog.file_mode = EditorFileDialog.FILE_MODE_OPEN_FILE
	dialog.title     = "Open Point Cloud"
	dialog.add_filter("*.las,*.laz", "LAS / LAZ Point Clouds")
	dialog.file_selected.connect(func(path: String) -> void:
		_path_edit.text = path
		dialog.queue_free()
	)
	add_child(dialog)
	dialog.popup_centered_ratio(0.6)


func _on_load_preview_pressed() -> void:
	var path: String = _path_edit.text.strip_edges()
	if path.is_empty():
		_status_label.text = "Enter a file path first."
		return
	if _preview_thread != null and _preview_thread.is_alive():
		return
	_status_label.text = "Loading preview..."
	_progress_bar.value = 0
	_progress_bar.visible = true
	_reader = PointCloudReader.new()
	_preview_thread = Thread.new()
	_preview_thread.start(_reader.load.bind(path))
	_poll_timer.start()


func _on_add_to_scene_pressed() -> void:
	var path: String = _path_edit.text.strip_edges()
	if path.is_empty():
		_status_label.text = "Enter a file path first."
		return
	if not editor_interface:
		_status_label.text = "No editor interface."
		return
	var sel: Array[Node] = editor_interface.get_selection().get_selected_nodes()
	var parent: Node = sel[0] if sel.size() > 0 else editor_interface.get_edited_scene_root()
	if parent == null:
		_status_label.text = "Open a scene first."
		return

	var node := PointCloudNode.new()
	node.name                 = "PointCloud"
	node.file_path            = path
	node.point_size           = _spin_point_size.value
	node.color_mode           = _opt_color_mode.selected
	node.max_points_per_chunk = int(_spin_max_chunk.value)
	node.auto_center          = _chk_auto_center.button_pressed

	# Apply georef path if a node is selected.
	if is_instance_valid(_georef_node) and is_instance_valid(node):
		node.georef_node_path = node.get_path_to(_georef_node)

	parent.add_child(node)
	node.owner = editor_interface.get_edited_scene_root()

	_active_node = node
	_destroy_clip_box()
	_lbl_clip_status.text = "No clip box."

	node.loaded.connect(_on_node_loaded.bind(node), CONNECT_ONE_SHOT)
	node.load_failed.connect(_on_node_load_failed.bind(node), CONNECT_ONE_SHOT)

	_status_label.text = "Loading..."
	_progress_bar.value = 0
	_progress_bar.visible = true
	_poll_timer.start()
	node.load_async()


func _on_node_loaded(point_count: int, node: PointCloudNode) -> void:
	_poll_timer.stop()
	_progress_bar.value = 100
	_progress_bar.visible = false
	_reader = node.get_reader()
	_update_info_panel()
	_update_debug_panel()
	_status_label.text = "Added '%s' (%s pts)." % [node.name, _fmt_int(point_count)]


func _on_node_load_failed(error: String, _node: PointCloudNode) -> void:
	_poll_timer.stop()
	_progress_bar.value = 0
	_progress_bar.visible = false
	_status_label.text = "Error: " + error


func _on_progress_poll() -> void:
	# Preview thread finished?
	if _preview_thread != null and not _preview_thread.is_alive():
		var err: int = _preview_thread.wait_to_finish()
		_preview_thread = null
		_poll_timer.stop()
		_progress_bar.value = 100
		_progress_bar.visible = false
		if err != OK:
			_status_label.text = "Error %d -- could not open file." % err
			_reader = null
		else:
			_update_info_panel()
			_status_label.text = "Preview loaded."
		return
	# Active-node async load in progress?
	if is_instance_valid(_active_node) and _active_node.get_load_progress() < 100:
		var pct: int = _active_node.get_load_progress()
		_progress_bar.value = pct
		_status_label.text = "Loading... %d%%" % pct
	# Preview load in progress?
	elif _preview_thread != null:
		var pct: int = _reader.get_load_progress()
		_progress_bar.value = pct
		_status_label.text = "Loading preview... %d%%" % pct
	else:
		_poll_timer.stop()
		_progress_bar.visible = false


func _on_point_size_changed(value: float) -> void:
	if is_instance_valid(_active_node):
		_active_node.point_size = value


func _on_color_mode_selected(index: int) -> void:
	if is_instance_valid(_active_node):
		_active_node.color_mode = index


func _on_max_chunk_changed(value: float) -> void:
	if is_instance_valid(_active_node):
		_active_node.max_points_per_chunk = int(value)


func _on_auto_center_toggled(pressed: bool) -> void:
	if is_instance_valid(_active_node):
		_active_node.auto_center = pressed


func _on_pick_georef_pressed() -> void:
	if not editor_interface:
		_status_label.text = "No editor interface."
		return
	# popup_node_selector(callback, valid_types) — filters tree to IFCGeoreference nodes.
	editor_interface.popup_node_selector(
		_on_georef_node_selected,
		PackedStringArray(["IFCGeoreference"]))


func _on_georef_node_selected(path: NodePath) -> void:
	if path.is_empty():
		return
	var root := editor_interface.get_edited_scene_root()
	if root == null:
		return
	var node := root.get_node_or_null(path)
	if node == null:
		_status_label.text = "Could not find selected node."
		return
	_georef_node = node
	_lbl_georef_node.text = node.name
	_lbl_georef_node.tooltip_text = str(path)
	# Propagate to existing active node immediately.
	if is_instance_valid(_active_node):
		_active_node.georef_node_path = _active_node.get_path_to(_georef_node)
	_update_crs_labels()
	_status_label.text = "Georef set to '%s'." % node.name


func _on_clear_georef_pressed() -> void:
	_georef_node = null
	_lbl_georef_node.text = "(none)"
	_lbl_georef_node.tooltip_text = "IFCGeoreference node used for CRS alignment"
	if is_instance_valid(_active_node):
		_active_node.georef_node_path = NodePath("")
	_update_crs_labels()
	_status_label.text = "Georef cleared."


func _on_rebuild_pressed() -> void:
	if not is_instance_valid(_active_node):
		_status_label.text = "No PointCloudNode in scene yet."
		return
	_active_node.point_size           = _spin_point_size.value
	_active_node.color_mode           = _opt_color_mode.selected
	_active_node.max_points_per_chunk = int(_spin_max_chunk.value)
	_active_node.auto_center          = _chk_auto_center.button_pressed
	# Re-apply georef path.
	if is_instance_valid(_georef_node):
		_active_node.georef_node_path = _active_node.get_path_to(_georef_node)
	else:
		_active_node.georef_node_path = NodePath("")
	_active_node.load()
	_destroy_clip_box()
	_lbl_clip_status.text = "No clip box."
	_update_debug_panel()
	_status_label.text = "Mesh rebuilt."


func _on_show_bounds_toggled(pressed: bool) -> void:
	if is_instance_valid(_active_node):
		_active_node.show_chunk_bounds = pressed


# ---- Clip box handlers ------------------------------------------------------

func _on_create_clip_box_pressed() -> void:
	if not is_instance_valid(_active_node):
		_status_label.text = "Load a point cloud first."
		return
	_destroy_clip_box()

	var bounds: AABB = _active_node.get_reader().get_bounds()
	# _spin_clip_x.value = maxf(bounds.size.x * 0.5, 0.1)
	# _spin_clip_y.value = maxf(bounds.size.y * 0.5, 0.1)
	# _spin_clip_z.value = maxf(bounds.size.z * 0.5, 0.1)

	_spin_clip_x.value = 50
	_spin_clip_y.value = 50
	_spin_clip_z.value = 50

	_clip_box_node = Node3D.new()
	_clip_box_node.name = "PointCloudClipBox"
	_clip_box_node.set_meta("__pc_clip", true)

	_refresh_clip_box_mesh()

	_active_node.add_child(_clip_box_node)
	if editor_interface:
		_clip_box_node.owner = editor_interface.get_edited_scene_root()
		editor_interface.get_selection().clear()
		editor_interface.get_selection().add_node(_clip_box_node)

	_lbl_clip_status.text = "Box ready -- move in viewport, adjust sizes, then Apply."


func _on_clip_size_changed(_value: float) -> void:
	_refresh_clip_box_mesh()


func _on_apply_clip_pressed() -> void:
	if not is_instance_valid(_active_node):
		_lbl_clip_status.text = "Load a point cloud first."
		return
	if not is_instance_valid(_clip_box_node):
		_lbl_clip_status.text = "Create a clip box first."
		return

	var clip_xform: Transform3D = _active_node.global_transform.affine_inverse() * _clip_box_node.global_transform
	var half_extents := Vector3(_spin_clip_x.value, _spin_clip_y.value, _spin_clip_z.value)

	_status_label.text = "Applying clip..."
	_active_node.apply_clip(clip_xform, half_extents)
	_update_debug_panel()
	_lbl_clip_status.text = "Clip active -- %d chunks visible." % _active_node.get_chunk_count()
	_status_label.text = "Clip applied."


func _on_reset_clip_pressed() -> void:
	if not is_instance_valid(_active_node):
		return
	_active_node.reset_clip()
	_update_debug_panel()
	_lbl_clip_status.text = "Clip cleared -- all points restored."
	_status_label.text = "Clip reset."


func _on_print_stats_pressed() -> void:
	if not _reader:
		print("[PointCloud] No cloud loaded in dock.")
		return
	print("=== Point Cloud Stats ===")
	print("  File       : ", _reader.get_source_path())
	print("  Points     : ", _fmt_int(_reader.get_point_count()))
	print("  Format ID  : %d" % _reader.get_point_format_id())
	print("  Has RGB    : ", _reader.has_color())
	print("  Bounds     : ", _reader.get_bounds())
	print("  Center     : ", _fmt_vec3(_reader.get_center()))
	print("  Load time  : %.1f ms" % _reader.get_load_time_ms())
	if is_instance_valid(_active_node):
		print("  Chunks     : %d" % _active_node.get_chunk_count())
		print("  Build time : %.1f ms" % _active_node.get_build_time_ms())
		print("  Has clip   : ", _active_node.get_has_clip())
	print("=========================")


# -------------------------------------------------------- update panels -----
func _update_info_panel() -> void:
	if not _reader:
		return
	_lbl_points.text    = _fmt_int(_reader.get_point_count())
	_lbl_format.text    = "LAS %d" % _reader.get_point_format_id()
	_lbl_rgb.text       = "Yes" if _reader.has_color() else "No"
	var b: AABB         = _reader.get_bounds()
	_lbl_size.text      = "%.1f x %.1f x %.1f" % [b.size.x, b.size.y, b.size.z]
	_lbl_center.text    = _fmt_vec3(_reader.get_center())
	_lbl_load_time.text = "%.1f ms" % _reader.get_load_time_ms()
	_update_crs_labels()


func _update_crs_labels() -> void:
	# ── LAS CRS ──────────────────────────────────────────────────────────────
	var las_wkt := ""
	if _reader:
		las_wkt = _reader.get_crs_wkt()
	if las_wkt.is_empty():
		_lbl_las_crs.text = "(no CRS VLR in file)"
		_lbl_las_crs.add_theme_color_override("font_color", Color(0.8, 0.6, 0.3))
	else:
		# Show only the first line of the WKT (it can be thousands of characters)
		var first_line: String = las_wkt.split("\n")[0].strip_edges()
		_lbl_las_crs.text = first_line if first_line.length() <= 60 else first_line.substr(0, 57) + "..."
		_lbl_las_crs.tooltip_text = las_wkt
		_lbl_las_crs.add_theme_color_override("font_color", Color(0.7, 0.7, 0.7))

	# ── IFC CRS ───────────────────────────────────────────────────────────────
	var ifc_crs_name := ""
	if is_instance_valid(_georef_node) and _georef_node.has_method("get_crs_name"):
		ifc_crs_name = _georef_node.get_crs_name()
	if ifc_crs_name.is_empty() or ifc_crs_name == "NotDefined":
		_lbl_ifc_crs.text = "(pick georef node)"
		_lbl_ifc_crs.add_theme_color_override("font_color", Color(0.7, 0.7, 0.7))
	else:
		_lbl_ifc_crs.text = ifc_crs_name
		_lbl_ifc_crs.add_theme_color_override("font_color", Color(0.6, 0.9, 1.0))

	# ── Match check ───────────────────────────────────────────────────────────
	# NOTE: no reprojection is done; this is a visual sanity check only.
	if las_wkt.is_empty() or ifc_crs_name.is_empty() or ifc_crs_name == "NotDefined":
		_lbl_crs_match.text = ""
		return
	if las_wkt.findn(ifc_crs_name) >= 0:
		_lbl_crs_match.text = "CRS match OK"
		_lbl_crs_match.add_theme_color_override("font_color", Color(0.3, 0.9, 0.4))
	else:
		_lbl_crs_match.text = \
			"WARNING: LAS CRS may not match IFC CRS.\n" + \
			"No reprojection is applied. Ensure both files\n" + \
			"use the same projected coordinate system."
		_lbl_crs_match.add_theme_color_override("font_color", Color(1.0, 0.5, 0.2))


func _update_debug_panel() -> void:
	if not is_instance_valid(_active_node):
		return
	var r := _active_node.get_reader()
	if r:
		_lbl_total_pts.text = _fmt_int(r.get_point_count())
	_lbl_chunks.text     = str(_active_node.get_chunk_count())
	_lbl_build_time.text = "%.1f ms" % _active_node.get_build_time_ms()
