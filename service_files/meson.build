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
    ['damaged-sensor', 'xyz.openbmc_project.damagedsensor.service'],
    ['logstatus', 'xyz.openbmc_project.logstatus.service'],

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

if get_option('apisensor').allowed()
  num_apisensor_reactors = get_option('num_apisensor_reactors')

  foreach i : range(1, num_apisensor_reactors + 1)
    conf = configuration_data()
    conf.set('REACTOR_ID', i)

    # Set exec args only for reactor > 1
    if i == 1
      conf.set('EXEC_ARGS', '')
    else
      conf.set('EXEC_ARGS', ' ' + i.to_string())
    endif

    out_file = 'xyz.openbmc_project.apisensor@' + i.to_string() + '.service'

    configure_file(
      input: 'xyz.openbmc_project.apisensor@.service.in',
      output: out_file,
      configuration: conf,
      install: true,
      install_dir: systemd_system_unit_dir,
    )
  endforeach

  # Install the @.service template (mandatory for systemctl enable to work)
  configure_file(
    input: 'xyz.openbmc_project.apisensor@.service.in',
    output: 'xyz.openbmc_project.apisensor@.service',
    configuration: configuration_data(),
    install: true,
    install_dir: systemd_system_unit_dir,
  )
endif
