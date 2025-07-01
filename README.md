# Components for simulating hardware

In some cases it might be useful rather testing something on the  
real machine, to simulate this in a simulated environment.  
But the simulation environment should be as tight as possible to  
your real machine. The idea is to use the existing .hal files and  
detecting during launching linuxcnc if the real hardware exist. In case
it does not exist some hardware simulation will be loaded. 
Also some additional components might be needed to fulfil a proper simulation. 

Currently these are e.g. mesacard 7i76e, touch probe, fork light barrier, 
calibration ring, calibration quader.

## Detecting Hardware  

In my case the the main ini file for launching linuxcnc is  
e.g. CNC_AxisDisplay_3Axis.ini.
To be flexible i divided the setup for all the elements into  
seperate files (but this is not necessary for idea of simulating hardware) 

~~~ini
...
[HOSTMOT2]
DRIVER=hm2_eth 
IPADDR="192.168.1.121"
BOARD=7i76e
CONFIG="num_encoders=1 num_pwmgens=1 num_stepgens=3 sserial_port_0=20xxxx"

[HAL]
HALUI = halui
TWOPASS = on,verbose
HALFILE = Mesacard/hm2_7i76e.tcl

HALFILE = AxisJoints/x_axis.hal

...

~~~

As the hal files are not as flexible, we need to change to tcl-script  

Mesacard/hm2_7i76e.tcl
~~~ tcl
set kins_kinematics [lindex $::KINS(KINEMATICS) 0]
loadrt $kins_kinematics

loadrt $::EMCMOT(EMCMOT) \
    base_period_nsec=$::EMCMOT(BASE_PERIOD) \
    servo_period_nsec=$::EMCMOT(SERVO_PERIOD) \
    num_joints=$::TRAJ(AXES) num_dio=4 num_aio=4



# Hybrid check: ping Mesa board
set board_ip_ping [string trim [lindex $::HOSTMOT2(IPADDR) 0] "\""]
puts "board_ip: $board_ip_ping"

# Cache variable: mesa_card_type = "real" | "stub"
# Only determine type if not already set
if {![info exists ::mesa_card_type]} {

    # Try pinging with retries
    set retries 3
    set ::mesa_card_type "stub"
    for {set i 0} {$i < $retries} {incr i} {
        set ping_result [catch {exec ping -c 1 -W 1 $board_ip_ping}]
        if {$ping_result == 0} {
            set ::mesa_card_type "real"
            break
        }
        after 300
    }
}

if {$::mesa_card_type eq "real"} {
    puts "INFO: Mesa board is online at $board_ip_ping — loading hm2_eth"
	loadrt hostmot2
    loadrt hm2_eth board_ip=[lindex $::HOSTMOT2(IPADDR) 0] config=[lindex $::HOSTMOT2(CONFIG) 0]

} else {
    puts "WARNING: Mesa board at $board_ip_ping not reachable — loading mesa_7i76e_stub"
    loadrt mesa_7i76e_stub config=[lindex $::HOSTMOT2(CONFIG) 0]
}

addf hm2_7i76e.0.read  servo-thread
# addf hm2_7i76e.0.write servo-thread

# Shared simulated parameters
setp hm2_7i76e.0.watchdog.timeout_ns       5000000
setp hm2_7i76e.0.dpll.01.timer-us          -50
setp hm2_7i76e.0.stepgen.timer-number      1

~~~

Note: even without the simulation the addf hm2_7i76e.0.write servo-thread need to be added at a later stage 
of the launch process, from my experience, otherwise it failed.

Overall if the card can not be pinged, the stub component will be loaded.  
As usually the simulated environment runs on a seperate computer  
(virtual machine) without access to the mesa card, this works.

## Loading additional components and wiring

For illustration i use the: 
HALFILE = AxisJoints/x_axis.hal

This file is the same as for real hardware. Only on the end it will  
will include the file (source AxisJoints/x_axis_sim.tcl)  
for additional settings for simulation

~~~hal
...

# ---setup home / limit switch signals---

net both-home-x     =>  joint.0.home-sw-in
net both-home-x     =>  joint.0.neg-lim-sw-in
net both-home-x     =>  joint.0.pos-lim-sw-in

source AxisJoints/x_axis_sim.tcl

~~~

and this AxisJoints/x_axis_sim.tcl loads the additions
when the real hardware is not available.
For the simulation the end-stop trigger needs to be simulated.

~~~tcl
if {$::mesa_card_type eq "stub"} {
	loadrt comp names=comp_x
	addf   comp_x                   servo-thread
	net   Xhomeswpos   => comp_x.in0
	sets  Xhomeswpos $::JOINT_0(HOME_OFFSET)

	net   x-pos-cmd    => comp_x.in1
	setp  comp_x.hyst  .02
	net   home-x-sim-switch hm2_7i76e.0.7i76.0.0.input-04-sim  <= comp_x.out
}
~~~

As the hm2_7i76e.0.7i76.0.0.input-xx pins are defined as OUT, the component  
simulating the mesa card has additional hm2_7i76e.0.7i76.0.0.input-xx-sim pins  
defined as IN, from where you can connect/trigger/simulate the  
hm2_7i76e.0.7i76.0.0.input-xx pins 

~~~bash
halcmd show pin hm2_7i76e.*

Component Pins:
Owner   Type  Dir         Value  Name
...
    46  bit   OUT         FALSE  hm2_7i76e.0.7i76.0.0.input-04 ==> both-home-x
    46  bit   OUT          TRUE  hm2_7i76e.0.7i76.0.0.input-04-not
    46  bit   IN          FALSE  hm2_7i76e.0.7i76.0.0.input-04-sim <== home-x-sim-switch
    46  bit   OUT         FALSE  hm2_7i76e.0.7i76.0.0.input-05 ==> both-home-y
    46  bit   OUT          TRUE  hm2_7i76e.0.7i76.0.0.input-05-not
    46  bit   IN          FALSE  hm2_7i76e.0.7i76.0.0.input-05-sim <== home-y-sim-switch
    46  bit   OUT         FALSE  hm2_7i76e.0.7i76.0.0.input-06 ==> both-home-z
    46  bit   OUT          TRUE  hm2_7i76e.0.7i76.0.0.input-06-not
    46  bit   IN          FALSE  hm2_7i76e.0.7i76.0.0.input-06-sim <== home-z-sim-switch
...
~~~

## Compiling the components



