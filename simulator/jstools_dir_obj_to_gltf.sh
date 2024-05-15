#!/bin/bash

#######################################################################################
### PURPOSE
###

# Convert OBJ to GLTF


#######################################################################################
### DOCS
###

# - https://github.com/CesiumGS/obj2gltf


#######################################################################################
### HOW TO USE
### 

# ./jstools_dir_obj_to_gltf source-directory target-directory


#######################################################################################
### Check for prerequisites binaries
###

printf "Check for neccesary executables... "

hash obj2gltf 2>/dev/null || {
    echo -e "\nERROR: obj2gltf not found in PATH. Exiting... " >&2
    exit 1
}

printf "Done.\n"


#######################################################################################
### READ INPUTS
###

SOURCE_DIR="$1"
TARGET_DIR="$2"

if [[ -z "${SOURCE_DIR}" ]]; then
    echo "ERROR: Please provide a source directory" >&2
    exit 1
fi
if [[ ! -d "${SOURCE_DIR}" ]]; then
    echo "ERROR: Cannot find source directory" >&2
    exit 1
fi

if [[ -z "${TARGET_DIR}" ]]; then
    TARGET_DIR="gltf"
    echo "No input target directory given. Will default to ${TARGET_DIR}."
fi
if [[ ! -d "$TARGET_DIR" ]]; then
    mkdir "${TARGET_DIR}"
fi

# Remove trailing /
SOURCE_DIR="${SOURCE_DIR%/}"
TARGET_DIR="${TARGET_DIR%/}"


#######################################################################################
### MAIN
###

for filepath in "${SOURCE_DIR}"/*.obj; do    
    filename="${filepath##*/}"
    target_filepath="${TARGET_DIR}/${filename%%.*}.gltf"
    echo "${target_filepath}"
    obj2gltf -i "${filepath}" -o "${target_filepath}"
done
