#
# Template for Web Builder, iMXRT1062
#

[platformio]
src_dir = src
include_dir = src

[common]
build_flags   =
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
lib_deps = /home/terjeio/grblHAL/teensy41_ethernet-master

[env:%env_name%]
board = %board%
platform = teensy@4.16
build_flags = ${common.build_flags}
%build_flags%
lib_deps = ${common.lib_deps}
%lib_deps%
  /home/terjeio/grblHAL/uSDFS-master
