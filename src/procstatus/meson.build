src_inc = include_directories('..')

executable(
    'processorstatus',
    'ProcessorStatus.cpp',
    'ProcessorStatusMain.cpp',
    dependencies: [default_deps, gpiodcxx, utils_dep],
    include_directories: src_inc,
    install: true,
    install_dir: '/usr/libexec/dbus-sensors',
)
