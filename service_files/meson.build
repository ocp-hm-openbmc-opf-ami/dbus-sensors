systemd = dependency('systemd')
systemd_system_unit_dir = systemd.get_variable(
    'systemdsystemunitdir',
    pkgconfig_define: ['prefix', get_option('prefix')],
)

unit_files = [
    ['adc', 'xyz.openbmc_project.adcsensor.service'],
    ['dbus-adc', 'xyz.openbmc_project.dbusadcsensor.service'],
    ['intel-cpu', 'xyz.openbmc_project.intelcpusensor.service'],
    ['exit-air', 'xyz.openbmc_project.exitairsensor.service'],
    ['fan', 'xyz.openbmc_project.fansensor.service'],
    ['hwmon-temp', 'xyz.openbmc_project.hwmontempsensor.service'],
    ['ipmb', 'xyz.openbmc_project.ipmbsensor.service'],
    ['intrusion', 'xyz.openbmc_project.intrusionsensor.service'],
    ['mcu', 'xyz.openbmc_project.mcutempsensor.service'],
    ['nvme', 'xyz.openbmc_project.nvmesensor.service'],
    ['psu', 'xyz.openbmc_project.psusensor.service'],
    ['external', 'xyz.openbmc_project.externalsensor.service'],
    ['procstatus', 'xyz.openbmc_project.processorstatus.service'],
    ['system', 'xyz.openbmc_project.systemsensor.service'],
    ['powerunit', 'xyz.openbmc_project.powerunitstatus.service'],
    ['acpisystem', 'xyz.openbmc_project.acpisystemstatus.service'],
    ['psustatus', 'xyz.openbmc_project.psustatus.service'],
    ['osstatus', 'xyz.openbmc_project.osstatus.service'],
    ['batterystatus', 'xyz.openbmc_project.batterystatus.service'],
    ['acpidevice', 'xyz.openbmc_project.acpidevicestatus.service'],
    ['digital', 'xyz.openbmc_project.digitaldiscrete.service'],
    ['bmc-firmware-health', 'xyz.openbmc_project.bmcfirmwarehealth.service'],
]

fs = import('fs')
foreach tuple : unit_files
    if get_option(tuple[0]).allowed()
        fs.copyfile(
            tuple[1],
            install: true,
            install_dir: systemd_system_unit_dir,
        )
    endif
endforeach
