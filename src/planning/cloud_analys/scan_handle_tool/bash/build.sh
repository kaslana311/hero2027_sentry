#!/bin/bash

# 获得当前脚本所在路径
DIR=$(cd "$(dirname "$0")"; pwd)
# 切换到项目根目录

cd "$DIR/.."
mkdir -p build
cd build

# 编译scan_handle_tool节点
cmake .. && make install

