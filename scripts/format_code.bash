#!/bin/sh

# Run clang-format on source
cd src
find . -name "*.cpp" -print0 | xargs --null -i  sh -c 'echo "clang-format -> {}";clang-format -i {};'
cd ..

# Run clang-format on header
cd include
find . -name "*.hpp" -print0 | xargs --null -i  sh -c 'echo "clang-format -> {}";clang-format -i {};'
cd ..

# Run clang-format on tests
cd tests
find . -name "*.cpp" -print0 | xargs --null -i  sh -c 'echo "clang-format -> {}";clang-format -i {};'
cd ..
