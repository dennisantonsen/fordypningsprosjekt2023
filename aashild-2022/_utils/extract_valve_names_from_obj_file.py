in_directory  = "/workspace/resources/huldra-models/huldra-medium/meshes/"
in_filename   = 'huldra-medium.obj'
in_file       = in_directory + in_filename
out_directory = "/workspace/_utils/"
out_filename  = 'valve_names_from_huldra_medium.txt'
out_file      = out_directory + out_filename

name_list = []

with open(in_file) as f:
    for line in f:
        # We are looking for this format:
        #o /[0-9][0-9]-[0-9]
        if line[:3] == 'o /' and line[3:4].isdigit() and line[4:5].isdigit() and line[5:6] == '-' and line[6:7].isdigit():
            name_list.append(line[3:12])

with open(out_file, 'a') as f:
    for i in range(0, len(name_list)):
        f.write(name_list[i] + '\n')