#!/bin/bash

set -o nounset
set -o errexit

# Where to place the generated documentation and associated files
documentation_output_dir="$(pwd)/generated"
[ -d "${documentation_output_dir}" ] || mkdir "${documentation_output_dir}"

yaml_collection_of_tagfiles="${documentation_output_dir}/collection_of_tagfiles.yaml"
yaml_collection_of_tagfiles_excluding_current_package="${documentation_output_dir}/collection_of_tagfiles_excluding_current_package.yaml"
# Clean out the collection of tagfiles
[ -f "${yaml_collection_of_tagfiles}" ] && rm "${yaml_collection_of_tagfiles}"
[ -f "${yaml_collection_of_tagfiles_excluding_current_package}" ] && rm "${yaml_collection_of_tagfiles_excluding_current_package}"

#$1=package_path
function extract_package_name_from_package_path () {
    trimmed_package_path=$(echo "${1}" | sed -e 's/\/package.xml//g')
    package_name=$(echo ${trimmed_package_path} | sed -e 's/.*\///g')
    echo "${package_name}"
}

#$1=package_name, $2=tagfile_path
function add_tagfile_to_yaml_collection_of_tagfiles () {
    cat >> "${yaml_collection_of_tagfiles}" <<EOF
- docs_url: ../../../${1}/html/c++
  location: file://${2}
EOF
}

#$1=package_name, $2=use_tagfiles{yes|no}
function generate_documentation_and_tagfile () {
    absolute_path_to_package="$(rospack find ${1})"
    package_documentation_output_dir="${documentation_output_dir}/${1}"
    package_tagfile="${documentation_output_dir}/${1}.tag"
    [ -d "${package_documentation_output_dir}" ] && rm -r "${package_documentation_output_dir}"
    if [ "${2}" = "yes" ]; then
        rosdoc_lite -o "${package_documentation_output_dir}" --tagfile="${yaml_collection_of_tagfiles_excluding_current_package}" --generate_tagfile="${package_tagfile}" "${absolute_path_to_package}"
    else
        rosdoc_lite -o "${package_documentation_output_dir}" --generate_tagfile="${package_tagfile}" "${absolute_path_to_package}"
        add_tagfile_to_yaml_collection_of_tagfiles "${1}" "${package_tagfile}"
    fi
}

#$1=package_name
function generate_documentation_using_tagfiles () {
    sed -e "/\/${1}\(\/\|\.\)/d" "${yaml_collection_of_tagfiles}" > "${yaml_collection_of_tagfiles_excluding_current_package}"
    generate_documentation_and_tagfile "${1}" "yes"
    # Cleanup the temporary file
    [ -f "${yaml_collection_of_tagfiles_excluding_current_package}" ] && rm "${yaml_collection_of_tagfiles_excluding_current_package}"
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
    generate_documentation_and_tagfile "${package_name}" "no" > "${documentation_output_dir}/${package_name}.log" 2>&1
done

# Generate documentation using tagfiles
for package_name in ${package_names}; do
    echo "########################################################################"
    echo "Generating documentation using tagfiles for ${package_name}"
    echo "########################################################################"
    generate_documentation_using_tagfiles "${package_name}" > "${documentation_output_dir}/${package_name}_with_tagfiles.log" 2>&1
done

exit 0

# sort and unique, to remove old entries in the tagfiles.yaml or simply remove it (make a backup) before each run/invocation of this script or similar
