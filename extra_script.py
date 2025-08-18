Import("env")

env.Append(LINKFLAGS=[
    "-L$PROJECT_DIR/etc/ld",
    "-Tshell.ld",
    "-Wl,-Map=$BUILD_DIR\\firmware.map"
])

