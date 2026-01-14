extends Node2D

var EXE_PATH = "../build/traffic_map"
var OUT_PATH = "/home/polimerto/Desktop/Coding/varna_traffic_map/build/out.txt"
var ITERATIONS = 3;

var f_gr = preload("res://f.tres")
var r_gr = preload("res://r.tres")
var k_gr = preload("res://k.tres")
var c_gr = preload("res://c.tres")
var l_gr = preload("res://l.tres")
var s_gr = preload("res://s.tres")
# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	var output = []
	OS.execute("pwd", [], output)
	#OS.execute(EXE_PATH, [str(ITERATIONS)], output)
	print("Running in: ", output[0])
	#print(output[1])
	var lines = 0;
	for line in FileAccess.open(OUT_PATH, FileAccess.READ).get_as_text().split("\n"):
		if !line: continue
		var args = line.split(" ")
		var l: Line2D = Line2D.new()
		l.add_point(Vector2( int(args[0])/100.0, int(args[1])/-100.0) )
		l.add_point(Vector2( int(args[2])/100.0, int(args[3])/-100.0) )
		if(args[4] == "f"): l.gradient = f_gr
		if(args[4] == "r"): l.gradient = r_gr
		if(args[4] == "k"): l.gradient = k_gr
		if(args[4] == "c"): l.gradient = c_gr
		if(args[4] == "l"): l.gradient = l_gr
		if(args[4] == "s"): l.gradient = s_gr
		else:
			l.default_color = Color.from_hsv(ord(args[4]) / 20.0, 1, 1)
		add_child(l)
		lines += 1
	print("segments: ", lines)


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _input(event: InputEvent) -> void:
	if event.is_pressed() or event is InputEventMouseButton:
		if Input.is_key_pressed(KEY_UP) or Input.is_mouse_button_pressed(MOUSE_BUTTON_WHEEL_UP):
			$Camera2D.zoom *= 1.1
			$Camera2D.position += (get_viewport().get_mouse_position() - get_viewport_rect().size/2) / $Camera2D.zoom / 10
		if Input.is_key_pressed(KEY_DOWN) or Input.is_mouse_button_pressed(MOUSE_BUTTON_WHEEL_DOWN):
			$Camera2D.zoom /= 1.1
			$Camera2D.position -= (get_viewport().get_mouse_position() - get_viewport_rect().size/2) / $Camera2D.zoom / 10
		if Input.is_key_pressed(KEY_F):
			for line in get_children():
				if(line is Line2D and line.gradient == f_gr): line.visible = !line.visible
		if Input.is_key_pressed(KEY_R):
			for line in get_children():
				if(line is Line2D and line.gradient == r_gr): line.visible = !line.visible
		if Input.is_key_pressed(KEY_K):
			for line in get_children():
				if(line is Line2D and line.gradient == k_gr): line.visible = !line.visible
		if Input.is_key_pressed(KEY_C):
			for line in get_children():
				if(line is Line2D and line.gradient == c_gr): line.visible = !line.visible
		if Input.is_key_pressed(KEY_L):
			for line in get_children():
				if(line is Line2D and line.gradient == l_gr): line.visible = !line.visible
		if Input.is_key_pressed(KEY_S):
			for line in get_children():
				if(line is Line2D and line.gradient == s_gr): line.visible = !line.visible
	if event is InputEventMouseMotion and Input.is_mouse_button_pressed(MOUSE_BUTTON_LEFT):
		$Camera2D.position -= event.screen_relative / $Camera2D.zoom
