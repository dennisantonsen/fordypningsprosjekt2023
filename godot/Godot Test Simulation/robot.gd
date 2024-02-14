extends CharacterBody3D

@onready var nav_agent = $NavigationAgent3D
var SPEED = 3.0

func _physics_process(delta):
	var current_location = global_transform.origin
	var next_location = nav_agent.get_next_path_position()
	var new_velocity = (next_location - current_location).normalized() * SPEED
	
	velocity = new_velocity
	move_and_slide()
	
	# Cast a ray towards the target Valve
	var space_state = get_world_3d().direct_space_state
	var robot_position = global_transform.origin
	var target_position = nav_agent.target_position
	var direction = (target_position - robot_position).normalized()
	var query = PhysicsRayQueryParameters3D.create(robot_position, target_position)
	query.exclude = [self]
	var result = space_state.intersect_ray(query)
	
	if result:
		print("Hit at point:", result.position)
		print("Collider: ", result.collider)
		# Handle what happens when the ray hits the Valve
	else:
		print("Cannot see the valve")

func update_target_location(target_location):
	nav_agent.target_position = target_location

func _on_navigation_agent_3d_target_reached():
	print("test")
