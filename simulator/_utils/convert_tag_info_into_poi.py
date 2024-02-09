# Must be run from within the workspace container (devcontainer)

in_directory  = "/workspace/_utils/"
in_filename   = 'tags_huldra_medium_in.txt'
in_file       = in_directory + in_filename
out_directory = "/workspace/_utils/"
out_filename  = 'tags_huldra_medium_out.txt'
out_file      = out_directory + out_filename

tag_list = []
x_list = []
y_list = []
z_list = []
roll_list = []
pitch_list = []
yaw_list = []

with open(in_file) as f:
    for line in f:
        print(line[:1])
        print(line[3:14])
        print(line[18:-2])
        if line[:1].isdigit():
            tag_list.append(line[:-1])
        elif line[3:14] == 'xCoordinate':
            x_list.append(line[17:-2])
        elif line[3:14] == 'yCoordinate':
            y_list.append(line[17:-2])
        elif line[3:14] == 'zCoordinate':
            z_list.append(line[17:-2])
        elif line[2:6] == 'roll':
            roll_list.append(line[9:-1])
        elif line[2:7] == 'pitch':
            pitch_list.append(line[9:-1])
        elif line[2:5] == 'yaw':
            yaw_list.append(line[9:-1]) # Watch out! This will turn out wrong for the last one if we don't have trailing line break

with open(out_file, 'a') as f:
    for i in range(0, len(tag_list)):
        x = - int(y_list[i])/1000.0
        y = int(z_list[i])/1000.0
        z = int(x_list[i])/1000.0
        roll = roll_list[i]
        pitch = pitch_list[i]
        yaw = yaw_list[i]
        f.write(f"POI('{tag_list[i]}', obj_to_gazebo_point([{x}, {y}, {z}]), orientation_from_euler({roll}, {pitch}, {yaw})),\n")