#!/bin/bash

sudo halcompile --install hm2_eth_mock.c
sudo halcompile --install sim_fork_light_barrier.comp
sudo halcompile --install sim_workpiece_quad.comp
sudo halcompile --install sim_workpiece_ring.comp