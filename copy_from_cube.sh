#!/bin/bash

# Define the directories
INC_DIR="include"
SRC_DIR="src"
STM32_INC_DIR="STM32CUBE_Generated/Core/Inc"
STM32_SRC_DIR="STM32CUBE_Generated/Core/Src"
MAIN_C_FILE="src/main.c"
MAIN_CPP_FILE="src/main.cpp"

# Delete everything in inc and src folders that is not AA_user
find "$INC_DIR" -mindepth 1 ! -name 'AA_user*' -delete
find "$SRC_DIR" -mindepth 1 ! -name 'AA_user*' -delete

# Copy the contents of STM32/inc to inc
cp -r "$STM32_INC_DIR"/* "$INC_DIR"

# Copy the contents of STM32/src to src
cp -r "$STM32_SRC_DIR"/* "$SRC_DIR"

# Rename main.c to main.cpp
if [ -f "$MAIN_C_FILE" ]; then
    mv "$MAIN_C_FILE" "$MAIN_CPP_FILE"
    echo "Renamed main.c to main.cpp"
else
    echo "main.c not found"
fi

echo "Operation completed successfully."