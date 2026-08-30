idf_component_register(
    SRCS "app_parser.c"
         "../../../Common/SRP/srp_crc.c"
         "../../../Common/SRP/srp_wire.c"
         "../../../Common/SRP/srp_codec.c"
         "../../../Common/SRP/srp_link.c"
    INCLUDE_DIRS "include" "../../../Common/SRP/include"
)
