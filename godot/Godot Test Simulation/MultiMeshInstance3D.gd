extends MultiMeshInstance3D

func _ready():
	self.multimesh.set_instance_transform(0, Transform3D(Basis(), Vector3(-0.5,0,0.8)))
	self.multimesh.set_instance_transform(1, Transform3D(Basis(Vector3(0,1,0), PI/2), Vector3(2,0,-2.5)))
