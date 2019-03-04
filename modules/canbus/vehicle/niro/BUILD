load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "niro_vehicle_factory",
    srcs = [
        "niro_vehicle_factory.cc",
    ],
    hdrs = [
        "niro_vehicle_factory.h",
    ],
    deps = [
        ":niro_controller",
        ":niro_message_manager",
        "//modules/canbus/vehicle:abstract_vehicle_factory",
    ],
)

cc_library(
    name = "niro_message_manager",
    srcs = [
        "niro_message_manager.cc",
    ],
    hdrs = [
        "niro_message_manager.h",
    ],
    deps = [
        "//modules/canbus/common:canbus_common",
        "//modules/canbus/proto:canbus_proto",
        "//modules/canbus/vehicle:message_manager_base",
        "//modules/canbus/vehicle/niro/protocol:canbus_niro_protocol",
    ],
)

cc_library(
    name = "niro_controller",
    srcs = [
        "niro_controller.cc",
    ],
    hdrs = [
        "niro_controller.h",
    ],
    deps = [
        ":niro_message_manager",
        "//modules/canbus/can_comm:can_sender",
        "//modules/canbus/common:canbus_common",
        "//modules/canbus/proto:canbus_proto",
        "//modules/canbus/vehicle:message_manager_base",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/canbus/vehicle/niro/protocol:canbus_niro_protocol",
    ],
)

cpplint()