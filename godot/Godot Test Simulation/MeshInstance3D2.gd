extends MeshInstance3D

func _ready():
	create_cross_mesh()

func create_cross_mesh():
	var cross_mesh = ArrayMesh.new()

	# Define vertices for the cross with thicker lines
	var vertices = PackedVector3Array([
		Vector3(-4, 0, 1),  # Left end (top)
		Vector3(-4, 0, -1),  # Left end (bottom)
		Vector3(8, 0, 1),   # Right end (top)
		Vector3(8, 0, -1),   # Right end (bottom)
		Vector3(2, 0, 4),   # Top end (right)
		Vector3(-2, 0, 4),  # Top end (left)
		Vector3(2, 0, -4),   # Bottom end (right)
		Vector3(-2, 0, -4),  # Bottom end (left)
	])

	# Define indices for the thicker lines
	var indices = PackedInt32Array([
		# Horizontal lines
		0, 1,
		2, 3,
		# Vertical lines
		4, 5,
		6, 7,
		# Thicker lines
		0, 2,  # Top line
		1, 3,  # Bottom line
		4, 6,  # Right line
		5, 7,  # Left line
	])

	# Create surface arrays
	var arrays = []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = vertices
	arrays[Mesh.ARRAY_INDEX] = indices

	# Create the surface
	cross_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_LINES, arrays)

	# Assign mesh to MeshInstance
	self.mesh = cross_mesh

	# Set colors for visual verification
	var material = StandardMaterial3D.new()
	material.albedo_color = Color(1, 0, 0) # Red color
	self.material_override = material
