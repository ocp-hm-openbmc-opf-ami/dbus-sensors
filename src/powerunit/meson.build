src_inc = include_directories('..')

executable(
    'powerunitstatus',
    'PowerUnit.cpp',
    'PowerUnitMain.cpp',
    dependencies: [default_deps, utils_dep],
    include_directories: src_inc,
    install: true,
    install_dir: '/usr/libexec/dbus-sensors',
)
