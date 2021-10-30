#!bin/bash
cppcheck --enable=all --std=c++11 -I src/ --suppress=missingIncludeSystem $( find . -name *.cpp) > results/cppcheck_result.txt
echo "Results are stored in results/cppcheck_result.txt"