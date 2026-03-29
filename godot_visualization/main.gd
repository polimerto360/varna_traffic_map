extends Node2D

var EXE_PATH = "../build/traffic_map"
var OUT_PATH = "/home/polimerto/Desktop/Coding/varna_traffic_map/build/out.txt"
#var OUT_PATH = "../build/out.txt"
var ITERATIONS = 100
var RUN_SIM = false

var f_gr = preload("res://f.tres")
var r_gr = preload("res://r.tres")
var k_gr = preload("res://k.tres")
var c_gr = preload("res://c.tres")
var l_gr = preload("res://l.tres")
var s_gr = preload("res://s.tres")
var time: float = 0.0
var step = 1
var timer: Timer
var input_file: FileAccess = FileAccess.open(OUT_PATH, FileAccess.READ)
var timestamps = {}
var cars: Array
var traffic_lights: Array

enum out_types {
	TIME = 			0,
	SEGMENT = 		0b1,
	TRAFFIC_LIGHT = 0b10,
	CAR = 			0b11
}

enum segment_types {
	FORWARD = 	0,
	REVERSE = 	0b0100,
	LONG = 		0b1000,
	BLACK_SEG =	0b1100
}

enum car_types {
	RED = 		0,
	GREEN = 	0b00100,
	BLUE = 		0b01000,
	DARK_BLUE = 0b01100,
	ORANGE =	0b10000,
	BLACK = 	0b10100
}

enum traffic_light_types {
	DEFINITION = 	0,
	SERIES = 		0b0100
}
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
	step = input_file.get_double()
	var lines = 0;
	
	while !input_file.eof_reached():
		var obj = input_file.get_8()
		
		match obj & 3:
			out_types.TIME:
				timestamps[snappedf(input_file.get_double(), step)] = input_file.get_position() + 8
			out_types.SEGMENT: # segment
				var x1 = input_file.get_32()
				var y1 = input_file.get_32()
				var x2 = input_file.get_32()
				var y2 = input_file.get_32()
				var l: Line2D = Line2D.new()
				l.show_behind_parent = true
				l.add_point(Vector2( int(x1)/100.0, int(y1)/-100.0) )
				l.add_point(Vector2( int(x2)/100.0, int(y2)/-100.0) )
				match obj ^ out_types.SEGMENT:
					segment_types.FORWARD: l.gradient = f_gr
					segment_types.REVERSE: l.gradient = r_gr
					segment_types.BLACK_SEG: l.gradient = k_gr
					segment_types.LONG: l.gradient = l_gr
					segment_types.FORWARD: l.gradient = f_gr
					segment_types.FORWARD: l.gradient = f_gr
					_: l.default_color = Color.from_hsv(obj / 20.0, 1, 1)
				add_child(l)
				lines += 1
				$Camera2D.position = l.points[0]
			out_types.CAR:
				input_file.get_64()
			out_types.TRAFFIC_LIGHT:
				if(obj ^ out_types.TRAFFIC_LIGHT != 0): input_file.seek(input_file.get_position() + tl_buffer_size())
				else: traffic_lights.push_back([read_point(input_file), read_point(input_file), 0])
	print("segments: ", lines)
	time = 0.0
	timer = Timer.new()
	add_child(timer)
	timer.timeout.connect(time_out)
	timer.one_shot = false
	timer.start(step);
func time_out():
	time += step
	print("time = ", time)
	queue_redraw()
func tl_buffer_size():
	return traffic_lights.size() / 8 + 1 if(traffic_lights.size() % 8) else 0
func read_point(input: FileAccess):
	return Vector2(float(input.get_32())/100.0, float(input.get_32())/-100.0)
func read_curr_time():
	if(!timestamps.has(time)):
		print("not found!")
		return
	var l_ind = timestamps[time]
	input_file.seek(l_ind)
	
	var obj = input_file.get_8()
	cars = []
	#traffic_lights = []
	while !input_file.eof_reached() && obj != 0:
		match obj & 3:
			out_types.CAR: cars.push_back([read_point(input_file), obj ^ out_types.CAR])
			out_types.TRAFFIC_LIGHT: 
				var buffer:PackedByteArray = input_file.get_buffer(tl_buffer_size())
				var index = 0
				for i in buffer:
					for j in 8:
						if(index == traffic_lights.size()): break
						traffic_lights[index][2] = ((i >> (7-j)) & 1)
						printraw(traffic_lights[index][2])
						index += 1
					printraw("\n")
			_: assert(false, "bad input")
		obj = input_file.get_8()
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
	time = snappedf(time, step)
	read_curr_time()
	
	 
	for car in cars:
		var c: Color
		match car[1]:
			car_types.RED: c = Color.RED
			car_types.BLUE: c = Color.BLUE
			car_types.DARK_BLUE: c = Color.DARK_BLUE
			car_types.GREEN: c = Color.LIME
			car_types.BLACK: c = Color.BLACK
			car_types.ORANGE: c = Color.ORANGE
		draw_circle(car[0], 5, c, true)
	
	for trl in traffic_lights:
		draw_line(trl[0], trl[1], Color.LIME_GREEN if trl[2] == 0 else Color.RED, 2)
	

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
			step = -abs(step)
		if Input.is_key_pressed(KEY_RIGHT):
			step = abs(step)
		if Input.is_key_pressed(KEY_PERIOD):
			time += abs(step)
			print("time = ", time)
			queue_redraw()
		if Input.is_key_pressed(KEY_COMMA):
			time -= abs(step)
			print("time = ", time)
			queue_redraw()
		if Input.is_key_pressed(KEY_SPACE):
			if timer.is_stopped():
				timer.start(abs(step))
			else:
				timer.stop()
		if Input.is_key_pressed(KEY_BRACKETLEFT): time -= 50.0 if Input.is_key_pressed(KEY_SHIFT) else 5.0
		if Input.is_key_pressed(KEY_BRACKETRIGHT): time += 50.0 if Input.is_key_pressed(KEY_SHIFT) else 5.0
			
	if event is InputEventMouseMotion and Input.is_mouse_button_pressed(MOUSE_BUTTON_LEFT):
		$Camera2D.position -= event.screen_relative / $Camera2D.zoom
