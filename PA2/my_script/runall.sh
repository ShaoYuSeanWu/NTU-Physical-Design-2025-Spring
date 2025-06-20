#!/bin/bash

EXE=$1
if [ -z "$EXE" ]; then
    echo "Usage: $0 <executable>"
    exit 1
fi

INPUT_DIR="./input_pa2"
OUTPUT_DIR="./output_pa2"
BENCHLIST="ami33 ami49 apte hp xerox"

for BENCH in $BENCHLIST; do
    echo "Running $BENCH"
    $EXE 0.5 $INPUT_DIR/$BENCH.block $INPUT_DIR/$BENCH.nets $OUTPUT_DIR/$BENCH.rpt > ./sean_log.log
    bash ./evaluator/evaluator.sh $INPUT_DIR/$BENCH.block $INPUT_DIR/$BENCH.nets $OUTPUT_DIR/$BENCH.rpt 0.5
    printf "\n\n"
done