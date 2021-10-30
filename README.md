# First Publisher/Subscriber ROS package
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
-----------------------
## Overview
This repository consist a publisher and subscriber node in C++. The publisher `talker.cpp` publishes a string message on the rostopic "/chatter" and along with this the subscriber `listener.cpp` subscribes to that topic and prints corresponding output received.

## License
MIT License  
Copyright (c) 2021 Pratik Bhujbal
```
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
```
## Dependencies

1. Ubuntu 18.04
2. ROS Melodic

## Build your package
```
mkdir -p <your_workspace_name>/src
cd <your_workspace_name>/src
git clone https://github.com/Prat33k-dev/beginner_tutorials.git
cd <your_workspace_name>
catkin_make
```
## Running
Open your workspace in new terminal
```
roscore
```
1. For running publisher node (in another terminal)
```
source devel/setup.bash
rosrun beginner_tutorials talker
```
2. For running subscriber node (in another terminal)
```
source devel/setup.bash
rosrun beginner_tutorials listener
```
## Run cppcheck and cpplint
* The Output txt files will be saved under results folder  

For cppcheck
```bash
sh run_cppcheck.sh
```
For cpplint
```
sh run_cpplint.sh 
`````

## Author
- Pratik Bhujbal  UID: 117555295   
  Github URL: https://github.com/prat33k-dev
