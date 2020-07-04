#!/bin/sh

find $PWD/src \
  -name "*.cpp" \
  -print0 \
  | xargs \
    --null \
    -i \
    sh -c 'echo "clang-format -> {}";clang-format -i {};'

find $PWD/src \
  -name "*.hpp" \
  -print0 \
  | xargs \
    --null \
    -i \
    sh -c 'echo "clang-format -> {}";clang-format -i {};'

# # Run clang-format on header
# find $PWD/include -name "*.hpp" -print0 | xargs --null -i  sh -c 'echo "clang-format -> {}";clang-format -i {};'
#
# # Run clang-format on tests
# find $PWD/tests -name "*.cpp" -print0 | xargs --null -i  sh -c 'echo "clang-format -> {}";clang-format -i {};'
