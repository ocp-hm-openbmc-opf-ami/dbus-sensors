src_inc = include_directories('..')

executable(
    'logstatus',
    'LoggingStatus.cpp',
    'LoggingStatusMain.cpp',
    dependencies: [default_deps, utils_dep],
    include_directories: src_inc,
    install: true,
    install_dir: '/usr/libexec/dbus-sensors',
)
