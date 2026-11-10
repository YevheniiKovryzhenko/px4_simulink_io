px4_add_module(
    MODULE drivers__distance_sensor__lightware_sf45_can
    MAIN lightware_sf45_can
    SRCS lightware_sf45_can.cpp
    DEPENDS px4_work_queue CollisionPrevention
    MODULE_CONFIG module.yaml
)
