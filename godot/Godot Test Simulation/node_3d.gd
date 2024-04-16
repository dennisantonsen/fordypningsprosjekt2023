extends Node3D

var valve_index = 0
@onready var robot = $GridContainer/SubViewportContainer/SubViewport/Robot
@onready var valves = get_tree().get_nodes_in_group("valves")

func _ready():
	get_tree().call_group("robot", "update_target_location", valves[valve_index].global_transform.origin)
	robot.valve_inspected.connect(_on_valve_inspected)

func _on_valve_inspected():
	valve_index += 1
	if valve_index <= len(valves)-1:
		print('--- Moving on to Valve ' + str(valve_index+1) + ' ---\n')
		_ready()
	else:
		valve_index = 0
		print('--- Moving on to Valve ' + str(valve_index+1) + ' ---\n')
		_ready()
