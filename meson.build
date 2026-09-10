project(
    'dbus-sensors',
    'cpp',
    default_options: ['warning_level=3', 'werror=true', 'cpp_std=c++23'],
    license: 'Apache-2.0',
    version: '0.1',
    meson_version: '>=1.1.1',
)

# Enable io_uring for all daemons with below flags.
#    '-DBOOST_ASIO_HAS_IO_URING',
#    '-DBOOST_ASIO_DISABLE_EPOLL',
# Note, there is currently an issue with CPUSensor when used in
# conjunction with io_uring. So it has not been changed to use
# random file access. But we'd like to enable it for all daemons.
# https://github.com/openbmc/dbus-sensors/issues/19

add_project_arguments(
    '-Wno-psabi',
    '-Wuninitialized',
    '-DBOOST_SYSTEM_NO_DEPRECATED',
    '-DBOOST_ASIO_NO_DEPRECATED',
    '-DBOOST_ERROR_CODE_HEADER_ONLY',
    '-DBOOST_NO_RTTI',
    '-DBOOST_NO_TYPEID',
    '-DBOOST_ALL_NO_LIB',
    '-DBOOST_ASIO_DISABLE_THREADS',
    '-DBOOST_ALLOW_DEPRECATED_HEADERS',
    '-DBOOST_ASIO_HAS_IO_URING',
    '-DBOOST_ASIO_DISABLE_EPOLL',
    '-DFEATURE_APISENSOR_SUPPORT',
    language: 'cpp',
)

if (get_option('tests').enabled())
    add_project_arguments(
        '-Wno-error=maybe-uninitialized',
        language: 'cpp',
    )
endif

threads = dependency('threads')
# i2c-tools doesn't ship a pkg-config file for libi2c
i2c = meson.get_compiler('cpp').find_library('i2c')
uring = dependency('liburing', include_type: 'system')
nlohmann_json_dep = dependency('nlohmann_json', include_type: 'system')
gpiodcxx = dependency('libgpiodcxx', default_options: ['bindings=cxx'])
boost = dependency('boost', version: '>=1.88.0', include_type: 'system', fallback: 'boost-meson',)
if (get_option('tests').enabled())
    sdbusplus = subproject('sdbusplus').get_variable('sdbusplus_dep')
    meson.override_dependency('sdbusplus', sdbusplus)
    phosphor_dbus_interfaces_dep = subproject('phosphor_dbus_interfaces').get_variable('phosphor_dbus_interfaces_dep')
    phosphor_logging_dep = subproject('phosphor-logging').get_variable('phosphor_logging_dep')
else
    sdbusplus = dependency('sdbusplus')
    phosphor_logging_dep = dependency('phosphor-logging')
endif
tests_opt = get_option('tests')
if tests_opt.enabled()
    add_project_arguments('-DUNIT_TESTING', language: 'cpp')
    peci_dep = dependency('lib_peci', required: false)
    if not peci_dep.found()
        peci_proj = subproject('lib_peci', required: true)
        peci_dep = peci_proj.get_variable('libpeci_dep')
    endif
endif
libudev = dependency('libudev', required : false, include_type: 'system')
libpsmapisensor = dependency('libpsmapisensor', required : false , include_type: 'system')
boost_dep = dependency(
    'boost',
    modules: ['coroutine'],
)

default_deps = [
    boost,
    boost_dep,
    nlohmann_json_dep,
    phosphor_logging_dep,
    sdbusplus,
    uring,
    libpsmapisensor,
]
if tests_opt.enabled()
    default_deps += [
        peci_dep,
        ]
endif

subdir('service_files')
subdir('src')
