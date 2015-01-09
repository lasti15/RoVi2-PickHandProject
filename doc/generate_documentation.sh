#!/bin/bash

set -o nounset
set -o errexit

# Where to place the generated documentation and associated files
documentation_output_dir="$(pwd)/generated"
[ -d "${documentation_output_dir}" ] || mkdir "${documentation_output_dir}"

#$1=package_path
function extract_package_name_from_package_path () {
    trimmed_package_path=$(echo "${1}" | sed -e 's/\/package.xml//g')
    package_name=$(echo ${trimmed_package_path} | sed -e 's/.*\///g')
    echo "${package_name}"
}

#$1=package_name
function generate_documentation_and_tagfile () {
    absolute_path_to_package="$(rospack find ${1})"
    package_documentation_output_dir="${documentation_output_dir}/${1}"
    [ -d "${package_documentation_output_dir}" ] && rm -r "${package_documentation_output_dir}"
    rosdoc_lite -o "${package_documentation_output_dir}" --generate_tagfile="${documentation_output_dir}/${1}.tag" "${absolute_path_to_package}"
}

# Find all CAROS packages
found_packages="$(find ../ -name 'package.xml')"
package_names=""

# Get package names
for found_package in ${found_packages}; do
    package_names="${package_names} $(extract_package_name_from_package_path ${found_package})"
done

# Generate documentation and tagfiles
for package_name in ${package_names}; do
    echo "------------------------------------------------------------------------"
    echo "Generating documentation and tagfile for ${package_name}"
    echo "------------------------------------------------------------------------"
    generate_documentation_and_tagfile "${package_name}" > "${documentation_output_dir}/${package_name}".log 2>&1
done

exit 0

# sort and unique, to remove old entries in the tagfiles.yaml or simply remove it (make a backup) before each run/invocation of this script or similar
