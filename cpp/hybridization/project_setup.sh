#!/usr/bin/env bash

INITIAL_DIR=$PWD
PROJECT_NAME_S_CASE=cmake_template
PROJECT_NAME_SS_CASE=CMAKE_TEMPLATE
TEMPLATE_MODULE_S_CASE=template_library
TEMPLATE_CLASS_S_CASE=template_class
TEMPLATE_CLASS_P_CASE=TemplateClass
TEMPLATE_CLASS_SS_CASE=TEMPLATE_CLASS
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
GIT_PROJECT=${DIR##*/}

# For colored output
export BLACK='\033[0;30m'
export RED='\033[0;31m'
export GREEN='\033[0;32m'
export ORANGE='\033[0;33m'
export BLUE='\033[0;34m'
export PURPLE='\033[0;35m'
export CYAN='\033[0;36m'
export LGRAY='\033[0;37m'

export DGRAY='\033[1;30m'
export LRED='\033[1;31m'
export LGREEN='\033[1;32m'
export YELLOW='\033[1;33m'
export LBLUE='\033[1;34m'
export LPURPLE='\033[1;35m'
export LCYAN='\033[1;36m'
export WHITE='\033[1;37m'

export NC='\033[0m'

ask_yn() {
  local prompt ans
	if [[ "${2,,}" == 'n' ]]; then
		prompt="${GREEN}${1}${NC} [y/${CYAN}N${NC}]"
	else
		prompt="${GREEN}${1}${NC} [${CYAN}Y${NC}/n]"
	fi

	echo -en "${prompt} "
	read ans

	if [[ "${2,,}" == 'n' ]]; then
		return $([ "${ans,,}" == "y" ] && echo 1 || echo 0)
	else
		return $([ "${ans,,}" == "n" ] && echo 0 || echo 1)
	fi
}

ask() {
  local prompt ans
  prompt="${GREEN}${1}${NC}"
	echo -en "${prompt} "
	read ans

  local correct=0
  local max_tries=3
  local tries=0
  while true; do
    ((tries=tries+1))
    ask_yn "Is \"${ORANGE}${ans}${GREEN}\" correct?"
    correct=$?
    if [[ $correct -gt 0 || $tries -ge $max_tries ]]; then break; fi

    echo -en "${prompt} "
	  read ans
  done

	eval $2=$ans
}

to_snake_case() {
  eval $2=$(echo $1 | sed -r 's/([A-Z])/_\L\1/g' | sed 's/^_//' | sed 's/-/_/g')
}

to_pascal_case() {
  eval $2=$(echo $1 | sed -r 's/[_-]([a-zA-Z])/\U\1/g')
}

to_camel_case() {
  local pascal_case
  to_pascal_case $1 pascal_case
  eval $2=$(echo $1 | sed -r 's/^([A-Z])/\L\1/')
}

to_upper_case() {
  eval $2=$(echo $1 | sed -r 's/(.*)/\U\1/')
}

to_screaming_snake_case() {
  local camel_case upper_case
  to_snake_case $1 camel_case
  to_upper_case $camel_case upper_case
  eval $2=$upper_case
}

to_title_case() {
  local pascal_case title_case
  to_pascal_case $1 pascal_case
  title_case="$(echo $pascal_case | sed -r 's/^(.)/\U\1/' | sed -r 's/([A-Z])/ \1/g')"
  eval $2="\$title_case"
}

update_project_files() {
  local file_to_update

  # Update deb files
  to_title_case $GIT_PROJECT GIT_PROJECT_T_CASE
  echo "$GIT_PROJECT_T_CASE" > deb/doc
  sed -i -r "s/${PROJECT_NAME_S_CASE}/${GIT_PROJECT_S_CASE}/g" deb/postinst
  sed -i -r "s/${PROJECT_NAME_S_CASE}/${GIT_PROJECT_S_CASE}/g" deb/postrm
  sed -i -r "s/${PROJECT_NAME_S_CASE}/${GIT_PROJECT_S_CASE}/g" Dockerfile
  sed -i -r "s/${PROJECT_NAME_S_CASE}/${GIT_PROJECT_S_CASE}/g" CMakeLists.txt
  if [ -f ${PROJECT_NAME_S_CASE}Config.cmake.in ]; then
    mv ${PROJECT_NAME_S_CASE}Config.cmake.in ${GIT_PROJECT_S_CASE}Config.cmake.in
  fi
}

add_new_module() {
  ask "What's the name of your new module? (snake_case)" module_name
  ask "What's the name of the first class in ${module_name}? (PascalCase)" class_name
  to_snake_case $module_name MODULE_NAME_S_CASE
  to_pascal_case $class_name CLASS_NAME_P_CASE
  to_snake_case $class_name CLASS_NAME_S_CASE
  to_screaming_snake_case $CLASS_NAME_S_CASE CLASS_NAME_SS_CASE

  # Copy and rename template files
  cd $DIR/modules
  cp -r ${TEMPLATE_MODULE_S_CASE} ${MODULE_NAME_S_CASE}
  cd ${MODULE_NAME_S_CASE}
  mv include/${PROJECT_NAME_S_CASE} include/${GIT_PROJECT_S_CASE}
  mv include/${GIT_PROJECT_S_CASE}/${TEMPLATE_MODULE_S_CASE} include/${GIT_PROJECT_S_CASE}/${MODULE_NAME_S_CASE}
  find . -type f -name "*${TEMPLATE_MODULE_S_CASE}*" |
    while IFS= read file_name; do
      mv $file_name $(echo $file_name | sed -e "s/${TEMPLATE_MODULE_S_CASE}/${MODULE_NAME_S_CASE}/")
    done
  mv include/${GIT_PROJECT_S_CASE}/${MODULE_NAME_S_CASE}/${TEMPLATE_CLASS_S_CASE}.h include/${GIT_PROJECT_S_CASE}/${MODULE_NAME_S_CASE}/${CLASS_NAME_S_CASE}.h
  mv src/${TEMPLATE_CLASS_S_CASE}.cpp src/${CLASS_NAME_S_CASE}.cpp
  mv test/test_${TEMPLATE_CLASS_S_CASE}.cpp test/test_${CLASS_NAME_S_CASE}.cpp

  local file_to_update

  # Update include guard, namespaces and class name in header
  file_to_update=include/${GIT_PROJECT_S_CASE}/${MODULE_NAME_S_CASE}/${CLASS_NAME_S_CASE}.h
  sed -i -r "s/${PROJECT_NAME_SS_CASE}/${GIT_PROJECT_SS_CASE}/g" $file_to_update
  sed -i -r "s/${TEMPLATE_CLASS_SS_CASE}/${CLASS_NAME_SS_CASE}/g" $file_to_update
  sed -i -r "s/${PROJECT_NAME_S_CASE}/${GIT_PROJECT_S_CASE}/g" $file_to_update
  sed -i -r "s/${TEMPLATE_MODULE_S_CASE}/${MODULE_NAME_S_CASE}/g" $file_to_update
  sed -i -r "s/${TEMPLATE_CLASS_P_CASE}/${CLASS_NAME_P_CASE}/g" $file_to_update

  # Update namespaces and class name in source
  file_to_update=src/${CLASS_NAME_S_CASE}.cpp
  sed -i -r "s/${PROJECT_NAME_S_CASE}/${GIT_PROJECT_S_CASE}/g" $file_to_update
  sed -i -r "s/${TEMPLATE_MODULE_S_CASE}/${MODULE_NAME_S_CASE}/g" $file_to_update
  sed -i -r "s/${TEMPLATE_CLASS_S_CASE}/${CLASS_NAME_S_CASE}/g" $file_to_update
  sed -i -r "s/${TEMPLATE_CLASS_P_CASE}/${CLASS_NAME_P_CASE}/g" $file_to_update

  # Update namespaces and class name in test
  file_to_update=test/test_${CLASS_NAME_S_CASE}.cpp
  sed -i -r "s/${PROJECT_NAME_S_CASE}/${GIT_PROJECT_S_CASE}/g" $file_to_update
  sed -i -r "s/${TEMPLATE_MODULE_S_CASE}/${MODULE_NAME_S_CASE}/g" $file_to_update
  sed -i -r "s/${TEMPLATE_CLASS_S_CASE}/${CLASS_NAME_S_CASE}/g" $file_to_update
  sed -i -r "s/${TEMPLATE_CLASS_P_CASE}/${CLASS_NAME_P_CASE}/g" $file_to_update
  file_to_update=test/CMakeLists.txt
  sed -i -r "s/${TEMPLATE_MODULE_S_CASE}/${MODULE_NAME_S_CASE}/g" $file_to_update
  sed -i -r "s/${TEMPLATE_CLASS_S_CASE}/${CLASS_NAME_S_CASE}/g" $file_to_update

  # Update module CMakeLists.txt
  file_to_update=CMakeLists.txt
  sed -i -r "s/${TEMPLATE_MODULE_S_CASE}/${MODULE_NAME_S_CASE}/g" $file_to_update
  sed -i -r "s/${TEMPLATE_CLASS_S_CASE}/${CLASS_NAME_S_CASE}/g" $file_to_update

  # Update top level CMakeLists.txt
  cd ${DIR}
  sed -i -r "s/${TEMPLATE_MODULE_S_CASE}/${MODULE_NAME_S_CASE}/" CMakeLists.txt
  echo -e "${ORANGE}Done${NC}"

  # Update toplevel test
  file_to_update=test/test_${MODULE_NAME_S_CASE}.cpp
  mv test/test_${TEMPLATE_MODULE_S_CASE}.cpp $file_to_update
  sed -i -r "s/${PROJECT_NAME_S_CASE}/${GIT_PROJECT_S_CASE}/g" $file_to_update
  sed -i -r "s/${TEMPLATE_MODULE_S_CASE}/${MODULE_NAME_S_CASE}/g" $file_to_update
  sed -i -r "s/${TEMPLATE_CLASS_S_CASE}/${CLASS_NAME_S_CASE}/g" $file_to_update
  sed -i -r "s/${TEMPLATE_CLASS_P_CASE}/${CLASS_NAME_P_CASE}/g" $file_to_update
  file_to_update=test/CMakeLists.txt
  sed -i -r "s/${TEMPLATE_MODULE_S_CASE}/${MODULE_NAME_S_CASE}/g" $file_to_update

  MODULE_CREATED=1
}

delete_templates_and_examples() {
  rm -r $DIR/apps/examples
  rm -r $DIR/modules/examples
  rm -r $DIR/modules/${TEMPLATE_MODULE_S_CASE}
  rm $DIR/test/test_calculator.cpp

  sed -i '/add_subdirectory(modules\/examples\/operator)/d' CMakeLists.txt
  sed -i '/add_subdirectory(modules\/examples\/addition)/d' CMakeLists.txt
  sed -i '/add_subdirectory(modules\/examples\/calculator)/d' CMakeLists.txt
  sed -i '/add_subdirectory(modules\/examples\/calculator-cli)/d' CMakeLists.txt
  sed -i '/add_subdirectory(apps\/examples\/calculator-runner)/d' CMakeLists.txt

  sed -i '/test_calculator.cpp/d' test/CMakeLists.txt
  sed -i '/${PROJECT_NAME}::calculator/d' test/CMakeLists.txt
  sed -i '/${PROJECT_NAME}::addition/d' test/CMakeLists.txt
}

self_destruct() {
  echo -e "${RED}Self destructing in..."
  echo -e "3"
  sleep 1
  echo -e "2"
  sleep 1
  echo -e "1"
  sleep 1
  echo -e "0${NC}"
  rm $(basename "$0")
}

__main() {
  to_snake_case $GIT_PROJECT GIT_PROJECT_S_CASE
  to_screaming_snake_case $GIT_PROJECT GIT_PROJECT_SS_CASE

  update_project_files

  MODULE_CREATED=0
  while true; do
    add_new_module
    ask_yn "Create another module?" n
    (( $? )) || break;
  done

  if [ $MODULE_CREATED -gt 0 ]; then
    echo -e "${ORANGE}Please delete the templates and examples prior to your first commit.${NC}"
    ask_yn "Would you like me to do this for you now?"
    (( $? )) && delete_templates_and_examples && self_destruct
  fi
}

__main