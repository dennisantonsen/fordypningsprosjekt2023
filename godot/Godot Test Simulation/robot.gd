extends CharacterBody3D
signal valve_inspected

@onready var nav_agent = $NavigationAgent3D

#Import the the objects in the "valves" group and the "walkway" group
@onready var valves = get_tree().get_nodes_in_group("valves")
@onready var walkway_array = get_tree().get_nodes_in_group("walkway")

#The resolution of which the robot moves along the edges of the walkway
@onready var d = 1

#Declaring various variables
var space_state = null
var robot_position = null
var target_position = null
var direction = null
var inspection_points = null
var optimal_inspection_point = null
var min_valve_distance = null
var query = null
var result = null
var cast_direction = null

func _ready():
	
	#Running the algorithm from scratch for each valve
	for valve_index in len(valves):
	
		await get_tree().create_timer(2).timeout
		
		space_state = get_world_3d().direct_space_state
		robot_position = global_transform.origin
		target_position = nav_agent.target_position
		direction = (target_position - robot_position).normalized()
		
		#List of all potential inspection points that the algorithm will test
		inspection_points = find_closest_points(target_position, d)

		#Initializing the optimal inspection point and valve distance
		optimal_inspection_point = null
		min_valve_distance = INF
		query = null
		result = null

		for point in inspection_points:
			
			#Teleport the robot to the inspection point for testing
			global_transform.origin = point
			
			await get_tree().create_timer(1).timeout
			
			#Updating the robots position
			robot_position = global_transform.origin
			
			#Check if there are obstacles between the robot and the valve
			query = PhysicsRayQueryParameters3D.create(point, target_position)
			query.exclude = [self]
			result = space_state.intersect_ray(query)
			cast_direction = (target_position - point).normalized()
			
			print("Inspecting Valve "+str(valve_index+1)+". Located at: ", target_position)
			print("Robot inspecting from position: ", robot_position, ". With direction: ", cast_direction)
			
			await get_tree().create_timer(1).timeout
		
			if ((result.collider == valves[valve_index]) && (point.distance_squared_to(target_position) < min_valve_distance)):
				#Update the optimal inspection point
				optimal_inspection_point = point
				min_valve_distance = point.distance_squared_to(target_position)
				print("\nOptimal inspection point: ", optimal_inspection_point, ". With direction: ", cast_direction,"\n")
				
				#Signal that the inspection is complete
				emit_signal("valve_inspected")
				
				#Quits searching when optimal inspection point is found
				#As the algorithm starts with the closest inspection point to the valve, and then gradually increases the distance, the consequtive points will in theory be less and less optimal disregarding obstacles
				break
			else:
				print("\nInspection point is not optimal\n")
			
			await get_tree().create_timer(1).timeout

func find_closest_points(target_position, d):
	var inspection_points = []
	var sorted_points = []

	for walkway_instance in walkway_array:
		var walkway_mesh = walkway_instance.get_mesh()
		
		if walkway_mesh:
			var walkway_mesh_data = walkway_mesh.surface_get_arrays(0)
			var vertices = walkway_mesh_data[Mesh.ARRAY_VERTEX]
			var indices = walkway_mesh_data[Mesh.ARRAY_INDEX]

			for i in range(indices.size() - 1):
				var start_point = walkway_instance.global_transform.origin + vertices[indices[i]]
				var end_point = walkway_instance.global_transform.origin + vertices[indices[i + 1]]
				
				var edge_length = start_point.distance_to(end_point)
				var num_points = int(edge_length / d) - 1
				
				var direction = (end_point - start_point).normalized()
				var distance_between_points = d * direction
				
				for j in range(num_points):
					var point = start_point + distance_between_points * (j + 1)
					var distance = target_position.distance_to(point)
					var add_point = true
					
					for existing_point in inspection_points:
						if point.distance_to(existing_point["point"]) < d:
							add_point = false
							break
					
					if add_point:
						inspection_points.append({"point": point, "distance": distance})

	# Sorting the points based on distance
	inspection_points.sort_custom(compare_distances)

	# Extracting only the "point" information
	for point_data in inspection_points:
		sorted_points.append(point_data["point"])

	return sorted_points

#Function to compare distances for sorting
func compare_distances(a, b):
	return a["distance"] < b["distance"]

func update_target_location(target_location):
	nav_agent.target_position = target_location
