# Smart Knob Panel — ESP-IDF project
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
add_compile_options(-fdiagnostics-color=always)
# 供 clangd / IDE 跳转与补全（生成 build/compile_commands.json）
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
project(smart-knob-panel)
