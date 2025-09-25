#
# Template for Web Builder, iMXRT1062
#

[platformio]
src_dir = src
include_dir = src

[common]
build_flags   =
  -I bluetooth # workaround for relative 3rd party plugin includes starting with ../
  -g3 
  -fmax-errors=5
  -fno-strict-aliasing
  -D OVERRIDE_MY_MACHINE

lib_archive   = no
lib_deps      = 
extra_scripts =
src_filter    = +<src/*>

[env]
framework     = arduino
extra_scripts = ${common.extra_scripts}
build_flags   = ${common.build_flags}
lib_deps      = ${common.lib_deps}
monitor_speed = 250000
monitor_flags =

[eth_networking]
build_flags =
lib_deps = /home/webbuilder/grblHAL/teensy41_ethernet

[env:%env_name%]
board = %board%
platform = teensy@4.16
build_flags = ${common.build_flags}
%build_flags%
lib_deps = ${common.lib_deps}
%lib_deps%
  /home/webbuilder/grblHAL/uSDFS
