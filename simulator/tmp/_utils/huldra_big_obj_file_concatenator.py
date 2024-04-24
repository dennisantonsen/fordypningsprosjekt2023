# Must be run from within the workspace container (devcontainer)

#https://stackoverflow.com/questions/17749058/combine-multiple-text-files-into-one-text-file-using-python
import glob, os

in_directory  = "/workspace/resources/huldra-models/huldra-big/meshes"
in_extension  = '.obj'
out_directory = '/workspace/resources/huldra-models/huldra-big/meshes/'
out_filename  = 'huldra-big.obj'
out_file      = out_directory + out_filename

os.chdir(in_directory)

with open(out_file, 'wb') as outfile:
    print('Concatenating files into', out_file)
    for file in glob.glob("*" + in_extension + ""):
        if file == out_filename:
            continue
        print('Reading', file, '...')
        with open(file, "rb") as infile:
            outfile.write(infile.read())

print('Done.')