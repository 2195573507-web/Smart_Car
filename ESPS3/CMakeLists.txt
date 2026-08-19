cmake_minimum_required(VERSION 3.16)

set(COMPONENTS main)

# The normal gateway relays both IMU telemetry and calibration frames. Set ON
# only for the bounded BMI323 diagnostic image.
set(SMARTCAR_BMI323_DEBUG_ONLY OFF CACHE BOOL
    "Run the S3 BMI323 log-forwarding diagnostic configuration")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(smartcar_s3_gateway)
