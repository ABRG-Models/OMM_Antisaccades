#!/bin/bash

# Run this script from the base OMM_Antisaccades directory

# Run the AS_healthy experiment from the model in ./spineml N times
N=10

# Set path to simulation engine.
S2B_BASE="${HOME}/SpineML_2_BRAHMS"
S2B="${S2B_BASE}/convert_script_s2b"

# e5 for AS_healthy and all the logs; e21 for AS_healthy, minimal logs
AS_HEALTHY_MAX_LOGS=5
AS_HEALTHY_MIN_LOGS=21

for i in $(seq 1 $N); do
    echo "Simulation ${i}..."
    ${S2B} -e ${AS_HEALTHY_MIN_LOGS} -m spineml -o tmp/AS_Healthy_run${i}
    # Clean up model and code directories
    rm -rf ./tmp/AS_Healthy_run${i}/model
    rm -rf ./tmp/AS_Healthy_run${i}/run/code
    rm -rf ./tmp/AS_Healthy_run${i}/output.script
done

# Check pro/anti and output to stdout
for i in $(seq 1 $N); do
    octave analysis/read_saccsim_side.m tmp/AS_Healthy_run${i}
done
