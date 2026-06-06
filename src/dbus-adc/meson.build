if not get_option('dbus-adc').allowed()
    subdir_done()
endif

executable(
    'dbusadcsensor',
    'DBusADCSensor.cpp',
    'DBusADCSensorMain.cpp',
    dependencies: [default_deps, gpiodcxx, thresholds_dep, utils_dep],
    include_directories: src_inc,
    install: true,
    install_dir: '/usr/libexec/dbus-sensors',
)
