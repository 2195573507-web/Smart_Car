cmake_minimum_required(VERSION 3.16)

set(COMPONENTS main)

# 正常网关会转发 IMU 遥测和标定帧；仅在需要受限 BMI323 诊断镜像时打开。
# 其它调试开关和日志周期统一见 Common/SmartCarDebug/smartcar_debug_config.h。
set(SMARTCAR_BMI323_DEBUG_ONLY OFF CACHE BOOL
    "Run the S3 BMI323 log-forwarding diagnostic configuration")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(smartcar_s3_gateway)
