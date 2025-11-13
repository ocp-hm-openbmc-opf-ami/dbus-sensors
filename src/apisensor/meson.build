src_inc = include_directories('..')

executable(
    'apisensor',
    'APISensor.cpp',
    'APISensorMain.cpp',
    dependencies: [default_deps, gpiodcxx, i2c, thresholds_dep, utils_dep],
    include_directories: src_inc,
    install: true,
    install_dir: '/usr/libexec/dbus-sensors',
)

