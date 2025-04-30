#!/bin/bash
# To use this locally, go to the root of your workspace
# cd ~/<workspace-name>/
# Then run ./src/easy_manipulation_deployment/coverage.sh html
# Coverage report will be generate automatically

# Enable branch coverage for local report
# ./src/easy_manipulation_deployment/coverage.sh html

if [ "$1" = "ci" ]; then
  cd ~/target_ws || return 1
fi

branch_command=()
branch_html_command=()
if [ "$2" = "--branch" ]; then
  branch_command=(--rc "lcov_branch_coverage=1")
  branch_html_command=(--branch-coverage)
fi

package_name="easy_perception_deployment"

ignored_files="*/test/*"

# Install LCOV
if [ "$(dpkg-query -W -f='${Status}' lcov 2>/dev/null | grep -c 'ok installed')" -eq 0 ];
then
  sudo apt-get install -y -qq lcov
fi


# Capture initial coverage info
lcov --capture --initial \
     --directory build \
     --output-file initial_coverage.info "${branch_command[@]}" | grep -ve "^Processing"
# Capture tested coverage info
lcov --capture \
     --directory build \
     --output-file test_coverage.info "${branch_command[@]}" | grep -ve "^Processing"
# Combine two report (exit function  when none of the records are valid)
lcov --add-tracefile initial_coverage.info \
     --add-tracefile test_coverage.info \
     --output-file coverage.info "${branch_command[@]}" || return 0 \
  && rm initial_coverage.info test_coverage.info
# Extract repository files
lcov --extract coverage.info "$(pwd)/src/$package_name/*" \
     --output-file coverage.info "${branch_command[@]}" | grep -ve "^Extracting"
# Filter out ignored files
lcov --remove coverage.info "$ignored_files" \
     --output-file coverage.info "${branch_command[@]}" | grep -ve "^removing"
if [ "$1" = "ci" ]; then
  # Some sed magic to remove identifiable absolute path
  sed -i "s~$(pwd)/src/$package_name/~~g" coverage.info
  lcov --list coverage.info "${branch_command[@]}"

  cd - || return 1
  cp -r ~/target_ws/coverage.info .

elif [ "$1" = "html" ]; then
  genhtml "${branch_html_command[@]}" coverage.info -o coverage
fi
