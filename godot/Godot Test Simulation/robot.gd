extends CharacterBody3D
signal valve_inspected

@onready var nav_agent = $NavigationAgent3D

#Import the the objects in the "valves" group and the "walkway" group
@onready var valves = get_tree().get_nodes_in_group("valves")
@onready var walkway_array = get_tree().get_nodes_in_group("walkway")

#The resolution of which the robot moves along the edges of the walkway. Smaller resolution results in more points visited by the robot and the chance for an optimal inspection point increases
@onready var resolution = 3

#The delay (in sec) added in the code to give the algorithm time to run and process information. Too short of a delay will cause the algorithm not to function
@onready var delay = 0.0001

#Array to store optimal inspection pose found by the algorithm
@onready var optimal_inspection_pose = {}
@onready var optimal_inspection_pose_inspections = {}

#Declaring various variables
var space_state = null
var robot_position = null
var target_position = null
var direction = null
var inspection_points = null
var min_valve_distance = null
var query = null
var result = null
var cast_direction = null
var inspection_points_length = null
var count = null
var file = null
var json = null
var data_json = null
var data_dict = null
var orientation = null

func _ready():
	
	#Running the algorithm from scratch for each valve
	for valve_index in len(valves):
	
		await get_tree().create_timer(delay).timeout
		
		space_state = get_world_3d().direct_space_state
		robot_position = global_transform.origin
		target_position = valves[valve_index].global_transform.origin
		direction = (target_position - robot_position).normalized()
		
		#List of all potential inspection points that the algorithm will test, and define the amount of points the robot will visit and a counting variable to the algorithm knows when to terminate in case there are no optimal inspection points
		inspection_points = find_closest_points(target_position, resolution)
		inspection_points_length = len(inspection_points)
		count = 0

		#Initializing the optimal inspection point and valve distance
		min_valve_distance = INF
		query = null
		result = null

		for point in inspection_points:
			
			count += 1
			
			#Teleport the robot to the inspection point for testing
			global_transform.origin = point
			
			await get_tree().create_timer(delay).timeout
			
			#Updating the robots position
			robot_position = global_transform.origin
			
			#Check if there are obstacles between the robot and the valve
			query = PhysicsRayQueryParameters3D.create(point, target_position)
			query.exclude = [self]
			result = space_state.intersect_ray(query)
			cast_direction = (target_position - point).normalized()
			
			orientation = Quaternion(Vector3.FORWARD, cast_direction)
			global_transform = Transform3D(Basis(orientation), robot_position)
			
			print("Inspecting Valve "+str(valve_index+1)+". Located at: ", target_position)
			print("Robot inspecting from position: ", robot_position, ". With direction: ", cast_direction)
			
			await get_tree().create_timer(delay).timeout
		
			if ((result.collider == valves[valve_index]) && (point.distance_squared_to(target_position) < min_valve_distance)):
				#Update the optimal inspection point
				min_valve_distance = point.distance_squared_to(target_position)
				print("\nOptimal inspection point: ", point, ". With direction: ", cast_direction,"\n")
				
				optimal_inspection_pose["pose"] = {}
				optimal_inspection_pose["pose"]["position"] = {}
				optimal_inspection_pose["pose"]["orientation"] = {}
				optimal_inspection_pose["inspections"] = []
				#optimal_inspection_pose["inspections"]["inspection_target"] = {}
				optimal_inspection_pose_inspections["type"] = "Image"
				optimal_inspection_pose_inspections["inspection_target"] = {}
				
				optimal_inspection_pose["pose"]["position"]["x"] = point.x
				optimal_inspection_pose["pose"]["position"]["y"] = point.y
				optimal_inspection_pose["pose"]["position"]["z"] = point.z
				optimal_inspection_pose["pose"]["position"]["frame"] = "asset"
				
				#Rotation from the forward vector to the direction vector that points towards the valve represented in quaternions. The rotation is relative to the forward vector defined in Godot
				orientation = Quaternion(Vector3.FORWARD, cast_direction)
				
				optimal_inspection_pose["pose"]["orientation"]["x"] = orientation.x
				optimal_inspection_pose["pose"]["orientation"]["y"] = orientation.y
				optimal_inspection_pose["pose"]["orientation"]["z"] = orientation.z
				optimal_inspection_pose["pose"]["orientation"]["w"] = orientation.w
				optimal_inspection_pose["pose"]["orientation"]["frame"] = "asset"
				
				optimal_inspection_pose_inspections["inspection_target"]["x"] = target_position.x
				optimal_inspection_pose_inspections["inspection_target"]["y"] = target_position.y
				optimal_inspection_pose_inspections["inspection_target"]["z"] = target_position.z
				optimal_inspection_pose_inspections["inspection_target"]["frame"] = "robot"
				optimal_inspection_pose["inspections"].append(optimal_inspection_pose_inspections)
				
				file = FileAccess.open("res://default_turtlebot.json", FileAccess.READ)
				data_json = file.get_as_text()
				data_dict = JSON.parse_string(data_json)
				file.close()
				
				data_dict["mission_definition"]["tasks"].append(optimal_inspection_pose)
				data_json = JSON.stringify(data_dict)
				
				file = FileAccess.open("res://default_turtlebot.json", FileAccess.WRITE)
				file.store_string(data_json)
				file.close()
				
				#Signal that the inspection is complete
				emit_signal("valve_inspected")
				
				#Quits searching when optimal inspection point is found
				#As the algorithm starts with the closest inspection point to the valve, and then gradually increases the distance, the consequtive points will in theory be less and less optimal disregarding obstacles
				break
			else:
				print("\nInspection point is not optimal\n")
				
				#If the count variable has reached the amount of inspection points (i.e. the algorithm has run through all the points on the walkway) the inspection is finished
				if count == inspection_points_length:
					print("No optimal inspection point found for Valve "+str(valve_index+1)+". Try a smaller resolution\n")
					emit_signal("valve_inspected")
			
			await get_tree().create_timer(delay).timeout

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
