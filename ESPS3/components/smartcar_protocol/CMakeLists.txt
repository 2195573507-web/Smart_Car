idf_component_register(
    SRCS "app_parser.c"
         "../../../Common/SCBP_CAN/scbp_crc.c"
         "../../../Common/SCBP_CAN/scbp_wire.c"
         "../../../Common/SCBP_CAN/scbp_parser.c"
         "../../../Common/SCBP_CAN/scbp_link.c"
    INCLUDE_DIRS "include" "../../../Common/SCBP_CAN/include"
)
