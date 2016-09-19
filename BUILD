# Bazel (http://bazel.io/) BUILD file for wc_chassis

COPTS = [
    "-std=c++11",
]
LINK_OPTS = []

# wc_chassis
cc_binary(
    name = "wc_chassis",
    srcs = glob([
    "sdk/src/TransferDevice/*.cpp",
    "wc_chassis/src/*.cpp",
    "sdk/src/Comm/*.cpp",
    "sdk/include/*.h",
    "sdk/include/*.hpp",
    "wc_chassis/include/*.h",
    ]),
    includes = [
        "sdk/include",
        "wc_chassis/include",
    ],
    copts = COPTS,
    linkopts = LINK_OPTS,
    defines = [
        "VERIFY_REMTOE_ID=0",
       # "TEST",
    ],
    linkstatic = True,
    deps = [
        "//gslib:gslib",
        "@io_bazel_rules_ros//ros:roscpp",
        "@io_bazel_rules_ros//ros:tf",
        "@io_bazel_rules_ros//ros:boost",
        "@io_bazel_rules_ros//ros:nav_msgs",
        "//autoscrubber_services:autoscrubber_services",
        "//gs:gs",
    ],
    visibility = ["//visibility:public"],
)
