extends Node2D

var EXE_PATH = "../build/traffic_map"
var OUT_PATH = "../build/out.txt"
var ITERATIONS = 100
var RUN_SIM = false

var f_gr = preload("res://f.tres")
var r_gr = preload("res://r.tres")
var k_gr = preload("res://k.tres")
var c_gr = preload("res://c.tres")
var l_gr = preload("res://l.tres")
var s_gr = preload("res://s.tres")
var time: float = 0.0
var step = 0.1
var cars = {}
# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	var output: Array = []
	if(RUN_SIM):
		OS.execute(EXE_PATH, ["Varna", str(ITERATIONS)], output)
		print(output[0])
	#var t = Timer.new()
	#add_child(t)
	#t.start(5)
	#await t.timeout
	var lines = 0;
	var curr_cars = []
	for line in FileAccess.open(OUT_PATH, FileAccess.READ).get_as_text().split("\n"):
		if !line: continue
		var args = line.split(" ")
		if(args[0] == "seg"):
			var l: Line2D = Line2D.new()
			l.show_behind_parent = true
			l.add_point(Vector2( int(args[1])/100.0, int(args[2])/-100.0) )
			l.add_point(Vector2( int(args[3])/100.0, int(args[4])/-100.0) )
			if(args[5] == "f"): l.gradient = f_gr
			if(args[5] == "r"): 
				l.gradient = r_gr
				l.z_index = 0
			if(args[5] == "k"): 
				l.gradient = k_gr
				l.z_index = 99
			if(args[5] == "c"): l.gradient = c_gr
			if(args[5] == "l"): l.gradient = l_gr
			if(args[5] == "s"): 
				l.gradient = s_gr
				l.z_index = 100
			else:
				l.default_color = Color.from_hsv(ord(args[5]) / 20.0, 1, 1)
			add_child(l)
			lines += 1
		if(args[0] == "car"):
			curr_cars.append(Vector2(float(args[1]) / 100.0, float(args[2]) / -100.0))
		if(args[0] == "t"):
			cars[time] = curr_cars.duplicate()
			time = snapped(float(args[1]), step);
			curr_cars.clear()
	print("segments: ", lines)
	cars[time] = curr_cars
	time = 0.0
		
func _draw():
	
	#draw_circle(Vector2(0, 0), 1, Color.CORAL, true)
	#draw_circle(Vector2(0, 0), 500000, Color.CORAL, false, 10000)
	#draw_circle(Vector2(5, 0), 1, Color.CORAL, true)
	#draw_circle(Vector2(0, 5), 1, Color.CORAL, true)
	#var l: Line2D = Line2D.new()
	#l.add_point(Vector2( 5, 0 ))
	#l.add_point(Vector2( 0, 5 ))
	#l.width = 1
	#add_child(l)
	time = snapped(time, 0.1)
	if(cars.has(time)):
		for car in cars[time]:
			draw_circle(car, 5, Color.CORAL, true)
	

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
		if Input.is_key_pressed(KEY_LEFT):
			time -= 0.1
			print("time: ", time)
			queue_redraw()
		if Input.is_key_pressed(KEY_RIGHT):
			time += 0.1
			print("time: ", time)
			queue_redraw()
			
	if event is InputEventMouseMotion and Input.is_mouse_button_pressed(MOUSE_BUTTON_LEFT):
		$Camera2D.position -= event.screen_relative / $Camera2D.zoom
