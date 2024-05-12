extends CharacterBody3D
signal valve_inspected



#################### Settings ####################

#The resolution of which the robot moves along the edges of the walkway. Higher resolution / decrease in the walkway_resolution variable (counter-intuitive) results in more points visited by the robot and the chance for an optimal inspection point increases
@onready var walkway_resolution = 0.75

#The height of the camera (in m) that is attached to the robot
@onready var max_camera_height = 3
@onready var min_camera_height = 0.2

#The amount of heights to search, between max and min height. Divides the height interval into "height_resolution" different heights. Ex: height_resolutuon = 5 will divide the interval into 5 different heights and search at these 5 heights. Needs to be an integer
@onready var height_resolution = 5

#If only one height is to be searched, then specify which height here (in m). Do not need to change this if the algorithm is to check several heights (the height_resolution is set to an integer larger than 1)
@onready var static_camera_height = 3

#The delay (in sec) is mainly in place for giving the algorithm time to save images from the inspections. This can also be used to delay the algorithm if the user wants to be able to see how the robot moves and position itself when inspecting the valves. Too short delay will result in black/gray inspection images. All other results (non-image results) should be fine however
@onready var delay = 0

#The algorithm will cast the corners of an equilateral triangle to assess the visibility from an inspection point. The sides of the makeshift equilateral triangle has the length raycast_triangle_length (in m). Take note that if this variable is set too high, the raycast triangle might jump past an obstacle right next to it, which might not necessarily ensure too good of a view
@onready var raycast_triangle_length = 0.1

#The minimum distance (in m) the camera can be away from the valve. This is to prevent the camera from being too close to the valve or potentially ending up inside the valve. If the algorithm stops due to an error that the ray-cast has failed, then this value should be increased as this means that the ray-cast was fired too close to the valve.
@onready var min_distance_from_valve = 0.8

#If the variable terminate_early is set to Y (yes), then the algorithm will terminate early if it find that the robot moves further away than the closest point currently found. This will result in the algorithm not finding an optimal inspection pose at a certain height if this inspection pose will be further away than the most optimal (and thus closest point) found thus far (such a pose would need a manual assessment from the resulting inspection images anyways, so it does not remove any automation aspects). This will also save screenshots of the optimal poses found. To deactivate, put any string value other than "Y" (ex. "N")
@onready var terminate_early = "Y"

#Location of the JSON mission file template (either empty or with a previous mission that should be extended with the results from this algorithm iteration)
@onready var json_file_template = "res://default_turtlebot_empty.json"

#Location of the JSON file where the optimal inspection points should be saved to. The results will be extended to the template file, so keep in mind if there are existing inspections in the template. Also keeo in mind that any other file with this name will be overwritten completely
@onready var json_file = "res://default_turtlebot.json"

#Change the iteration identifier to store different iteration results
@onready var iteration = "generalperformance"

##################################################



@onready var nav_agent = $NavigationAgent3D
@onready var camera = $Camera3D

#Import the the objects in the "valves" group and the "walkway" group
@onready var valves = get_tree().get_nodes_in_group("valves")
@onready var walkway_array = get_tree().get_nodes_in_group("walkway")

#Array to store optimal inspection pose found by the algorithm
@onready var optimal_inspection_pose = {}
@onready var optimal_inspection_pose_inspections = {}

#Declaring various variables
var space_state = null
var robot_position = null
var target_position = null
var direction = null
var inspection_points = null
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
var dir = null
var raycast_origin = null
var side_direction = null
var clear_sight = null
var raycast_corner
var camera_height_list = []
var current_optimal_point = []
var current_optimal_direction = []
var current_optimal_height = []
var current_optimal_orientation = []
var current_valve_location = []
var current_minimal_distance = []
var iteration_results = ""
var first_addition_to_file = 1
var raycast_fired = 1

func _ready():
	
	#Giving the algorithm 1 second to load everything
	await get_tree().create_timer(1).timeout
	
	#Making a folder to store results
	dir = DirAccess.open("res://")
	dir.make_dir(iteration)
	
	#Making a folder to store images specifically for the optimal inspections in a separate folder, if the algorithm is to terminate early
	if terminate_early == "Y":
		dir = DirAccess.open("res://"+str(iteration))
		dir.make_dir("optimal results")
	
	#Initializing list to store current most optimal position and minimal distance found
	for valve_index in len(valves):
		current_optimal_point.append(0)
		current_optimal_direction.append(0)
		current_optimal_height.append(0)
		current_optimal_orientation.append(0)
		current_valve_location.append(0)
		current_minimal_distance.append(INF)
	
	#Dividing the camera height interval into the specified resolution
	if height_resolution >= 2:
		camera_height_list = divide_heights(max_camera_height, min_camera_height, height_resolution)
	else:
		camera_height_list.append(static_camera_height)
	
	for camera_height in camera_height_list:
		
		#Making a folder to store results
		dir = DirAccess.open("res://"+str(iteration))
		dir.make_dir("results camera height - "+str(camera_height))
		
		#Moving the robot's camera to the correct height
		global_transform.origin = Vector3(0, 0, 0)
		camera.global_transform.origin = Vector3(0, 0, 0)
		camera.global_transform.origin = Vector3(0, camera_height, 0)
	
		#Running the search algorithm from scratch for each valve
		for valve_index in len(valves):
			
			space_state = get_world_3d().direct_space_state
			robot_position = global_transform.origin
			target_position = valves[valve_index].global_transform.origin
			direction = (target_position - robot_position).normalized()
			
			#List of all potential inspection points that the algorithm will test, and define the amount of points the robot will visit and a counting variable to the algorithm knows when to terminate in case there are no optimal inspection points
			inspection_points = find_closest_points(target_position, walkway_resolution)
			inspection_points_length = len(inspection_points)
			count = 0
				
			for point in inspection_points:
				
				count += 1
				raycast_fired = 0
				
				#Teleport the robot to the inspection point for testing
				global_transform.origin = point
				
				#Updating the robots position
				robot_position = global_transform.origin
				
				raycast_origin = point+Vector3(0, camera_height, 0)
				
				#If the ray-cast is fired too close to a valve, the ray-cast will fail. This if statement checks if the current pose is at least as far away from the valve as min_distance_from_valve
				if raycast_origin.distance_to(target_position) > 0.1:
					
					#Check if there are obstacles between the robot and the valve using a ray-cast
					query = PhysicsRayQueryParameters3D.create(raycast_origin, target_position)
					query.exclude = [self]
					result = space_state.intersect_ray(query)
					cast_direction = (target_position - raycast_origin).normalized()
					raycast_fired = 1

					#Rotation from the forward vector to the direction vector that points towards the valve represented in quaternions. The rotation is relative to the forward vector defined in Godot
					orientation = Quaternion(Vector3.FORWARD, cast_direction)
					camera.global_transform.basis = Basis(orientation)
				
				await get_tree().create_timer(delay).timeout
				
				#If the current inspection point is further away than the closest optimal point currently found by the algorithm, the algorithm will move to the next valve
				if terminate_early == "Y" and raycast_origin.distance_squared_to(target_position) > current_minimal_distance[valve_index]:
					print("This point is further away than the closest optimal point found thus far. Moving on to the next valve.\n")
					emit_signal("valve_inspected")
					break
				
				print("Inspecting Valve "+str(valve_index+1)+". Located at: ", target_position)
				print("Robot inspecting from position: ", robot_position, ". With direction: ", cast_direction,". And camera height: ", camera_height, "\n")
				
				#Resetting the clear_sight variable
				clear_sight = 0
			
				#The if statement will confirm that this point is an optimal point for this camera height. The nested if statements will assess the view towards to valve from the inspection point
				if raycast_fired == 1 and result.collider == valves[valve_index] and raycast_origin.distance_to(target_position) > min_distance_from_valve:
					
					#Checking the first corner of the makeshift raycast triangle to evaluate the view to the valve
					side_direction = cast_direction.cross(Vector3(0, 1, 0)).normalized()
					raycast_corner = raycast_origin + side_direction * raycast_triangle_length
					query = PhysicsRayQueryParameters3D.create(raycast_corner, target_position)
					query.exclude = [self]
					result = space_state.intersect_ray(query)
					
					#Another if statement to confirm that the robot can see the valve from the first corner of the makeshift raycast triangle
					if result.collider == valves[valve_index]:
						
						#Checking the second corner of the triangle
						raycast_corner = raycast_origin + side_direction.rotated(Vector3(0, 1, 0), -120).normalized() * raycast_triangle_length
						query = PhysicsRayQueryParameters3D.create(raycast_corner, target_position)
						query.exclude = [self]
						result = space_state.intersect_ray(query)
						
						if result.collider == valves[valve_index]:
							
							#Checking the final corner of the triangle
							raycast_corner = raycast_origin + side_direction.rotated(Vector3(0, 1, 0), 120).normalized() * raycast_triangle_length
							query = PhysicsRayQueryParameters3D.create(raycast_corner, target_position)
							query.exclude = [self]
							result = space_state.intersect_ray(query)
							
							#If the valve is visible from all corners of the triangle, in addition to the center of the triangle, then the inspection point is deemed viable
							if result.collider == valves[valve_index]:
								
								clear_sight = 1
								
				if clear_sight == 1:
					
					#Update the optimal inspection pose
					print("Optimal inspection point at camera height ", camera_height,": ", point, ". With direction: ", cast_direction,"\n")
					
					#Storing iteration results
					iteration_results = iteration_results + "Valve "+str(valve_index+1)+": \n Position: "+str(point)+"\n Orientation: "+str(orientation)+"\n Direction: "+str(cast_direction)+"\n Camera Height: "+str(camera_height)+"\n--- \n"
					file = FileAccess.open("res://"+iteration+"/"+iteration+" results.txt", FileAccess.WRITE)
					file.store_string(iteration_results)
					file.close()
					
					#Saving a screenshot of the inspection pose
					camera.get_viewport().get_texture().get_image().save_png("res://"+iteration+"/"+"results camera height - "+str(camera_height)+"/Valve "+str(valve_index+1)+".png")
					
					#Activating the terminate early option will allow for the algorithm to store images of the optimal inspections specifically in a speparate folder
					if terminate_early == "Y":
						#await get_tree().create_timer(delay).timeout
						camera.get_viewport().get_texture().get_image().save_png("res://"+iteration+"/optimal results/Valve "+str(valve_index+1)+".png")
					
					#The if statement will confirm that the algorithm considers this point the most optimal thus far for all camera heights
					if raycast_origin.distance_squared_to(target_position) < current_minimal_distance[valve_index]:
						
						current_minimal_distance[valve_index] = raycast_origin.distance_squared_to(target_position)
						current_optimal_point[valve_index] = point
						current_optimal_direction[valve_index] = cast_direction
						current_optimal_orientation[valve_index] = orientation
						current_valve_location[valve_index] = target_position
						current_optimal_height[valve_index] = camera_height
						
						print("Most optimal inspection point of all camera heights thus far: ", point, ". With direction: ", cast_direction,". And height: ", camera_height, "\n")
					
					#Signal that the inspection is complete
					emit_signal("valve_inspected")
					
					#Quits searching when optimal inspection point is found, as the algorithm starts with the closest inspection point to the valve, and then gradually increases the distance, the consequtive points will in theory be less and less optimal disregarding obstacles
					break
				else:
					print("Inspection point is not optimal\n")
					
					#If the count variable has reached the amount of inspection points (i.e. the algorithm has run through all the points on the walkway) the inspection is finished
					if count == inspection_points_length:
						print("No optimal inspection point found for Valve "+str(valve_index+1)+". Try a smaller resolution or a different camera height.\n")
						emit_signal("valve_inspected")
				
		#This if statement checks if we have checked all heights (the final height we check will always be the last height of the camera_height_list). If we are done checking all heights, we add the most optimal points automatically to the tasks list
		if camera_height == camera_height_list[-1]:
			for vi in len(valves):
				create_json(vi, current_optimal_point[vi], current_optimal_orientation[vi], current_valve_location[vi], current_optimal_height[vi], first_addition_to_file)
				first_addition_to_file = 0
			
			print("Inspection complete.")

#Function to find the closest points on the walkway to the valve and sort them in order of length (closest point first)
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

	#Sorting the points based on distance
	inspection_points.sort_custom(compare_distances)

	#Extracting only the "point" information
	for point_data in inspection_points:
		sorted_points.append(point_data["point"])

	return sorted_points

#Function to compare distances for sorting
func compare_distances(a, b):
	return a["distance"] < b["distance"]

#Function to divide the heights between the minimum and mazimum height in parts based on the height_resolution setting
func divide_heights(max_height, min_height, n: int):
	var heights = []
	var step = (max_height - min_height) / (n - 1)
	
	for i in range(n):
		var height = min_height + step * i
		heights.append(height)
	
	return heights

#Function to create the JSON file containing the optimal poses found during the iteration
func create_json(valve_index, point, orientation, target_position, camera_height, first_addition_to_file):
	optimal_inspection_pose["pose"] = {}
	optimal_inspection_pose["pose"]["position"] = {}
	optimal_inspection_pose["pose"]["orientation"] = {}
	optimal_inspection_pose["inspections"] = []
	optimal_inspection_pose["tag"] = "Valve "+str(valve_index+1)+" at camera height "+str(camera_height)
	optimal_inspection_pose_inspections["type"] = "Image"
	optimal_inspection_pose_inspections["inspection_target"] = {}
	
	optimal_inspection_pose["pose"]["position"]["x"] = point.x
	optimal_inspection_pose["pose"]["position"]["y"] = point.y
	optimal_inspection_pose["pose"]["position"]["z"] = point.z
	optimal_inspection_pose["pose"]["position"]["frame"] = "asset"
	
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
	
	if first_addition_to_file == 1:
		file = FileAccess.open(json_file_template, FileAccess.READ)
		data_json = file.get_as_text()
		data_dict = JSON.parse_string(data_json)
		file.close()
	else:
		file = FileAccess.open(json_file, FileAccess.READ)
		data_json = file.get_as_text()
		data_dict = JSON.parse_string(data_json)
		file.close()
	
	data_dict["mission_definition"]["tasks"].append(optimal_inspection_pose)
	data_json = JSON.stringify(data_dict)
	
	#Adding the inspections to the task list
	file = FileAccess.open(json_file, FileAccess.WRITE)
	file.store_string(data_json)
	file.close()
