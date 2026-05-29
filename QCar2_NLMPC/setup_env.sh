#!/usr/bin/env bash

export ACADOS_SOURCE_DIR=/home/khanh/deps/acados
export LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH

source /home/khanh/deps/acados_venv/bin/activate

export PYTHONPATH=$ACADOS_SOURCE_DIR/interfaces/acados_template:$PYTHONPATH
export PYTHONPATH=/home/khanh/QCar2_NLMPC:$PYTHONPATH