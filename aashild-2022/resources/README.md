# Resources

In this folder, we store most of the 3D model files used in the project. The Huldra model files stored in this directory are used by the other containers.

See docker-compose.yml for details about how these files are mounted into the containers.

## Plants

Some of the models (e.g. huldra-small-area and huldra-smaller) are extracted parts of the Huldra plant.

## Walkways

The models with "walkway" in the name (e.g. huldra-small-area-walkway and huldra-smaller-walkway) are models of walkway only. huldra-smaller-walkway is the walkway that is relevant for the huldra-smaller model.

## Valves

The strangely named models (e.g. 20-2000VF) are valves (or at least some kinds of points of interest). 

## Github LFS

Some of the models in the resources directory are too big for Github. They have to be uploaded with Github LFS. When your Github LFS is full, you have a problem. Therefore, some of the files here are divited into several smaller files (e.g. `huldra-medium.obj` are divided into `huldra-medium.obj.part1` and `huldra-medium.obj.part2`). Just add part1 and part2 together to "make" the original file.