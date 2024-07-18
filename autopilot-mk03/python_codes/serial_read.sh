#!/bin/bash

# Get the current date and time in the format YYYYMMDD_HHMMSS
current_date_time=$(date +"%Y%m%d_%H%M%S")

# Define the file name with the current date and time
file_name="data_$current_date_time.txt"

stty -F /dev/ttyUSB0 115200
cat /dev/ttyUSB0 > $file_name