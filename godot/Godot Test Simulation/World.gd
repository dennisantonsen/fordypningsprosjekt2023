extends Node3D

@onready var valve = $"Floor/Valve 1/CollisionShape3D"

func _physics_process(delta):
	get_tree().call_group("robot", "update_target_location", valve.global_transform.origin)
