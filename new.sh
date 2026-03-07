#!/bin/bash
#gnome-terminal -- bash -c "cd /home/dji/hero2026_-sentry && ./startup_system_try.sh"
# 1. 核心：先进入原脚本所在的目录
# 这样原脚本里的所有相对路径就都能找到了
cd /home/dji/hero2026_-sentry/bash

# 2. 执行原脚本
# 使用 ./ 确保是在当前目录下执行
./startup_system_try.sh
