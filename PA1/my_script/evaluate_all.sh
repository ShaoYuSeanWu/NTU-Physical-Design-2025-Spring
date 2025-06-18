#!/bin/bash
if [ -z "$1" ]; then
    echo "Usage: $0 <path_to_binary>"
    exit 1
fi

EXE="$1"
if [ ! -f "$EXE" ]; then
    echo "Error: File $EXE not found!"
    exit 1
fi
if [ ! -x "$EXE" ]; then
    echo "Error: File $EXE is not executable!"
    exit 1
fi

INPUT_DIR="./input_pa1"
OUTPUT_DIR="./output_pa1"
EVA="./evaluator/evaluator.sh"

BENCHMARKS=$(ls "$INPUT_DIR"/)
# BENCHMARKS=("input_1.dat")
echo BENCHMARKS: $BENCHMARKS

# test all benchmarks
# record the cpu time and real time for each case
# the input filenames like "input_1.dat", while the output filenames like "output_1.dat"
for benchmark in $BENCHMARKS; do
    printf "\n\n"
    # get the number of the benchmark
    num=$(echo "$benchmark" | grep -o '[0-9]\+')
    echo "Running $benchmark..."
    # run the program and measure time
    # { time "$EXE $INPUT_DIR/$benchmark $OUTPUT_DIR/output_$num.dat"; } > "$OUTPUT_DIR/time_$num.dat" 2>&1
    (time $EXE $INPUT_DIR/$benchmark $OUTPUT_DIR/output_$num.dat) > "$OUTPUT_DIR/time_$num.dat" 2>&1
    # check if the program ran successfully
    if [ $? -ne 0 ]; then
        echo "Error: $EXE $INPUT_DIR/$benchmark failed!"
        continue
    fi
    # calculate the cou time and real time
    # get the cpu time and real time from the output

    # output of time command
    # real	0m0.002s
    # user	0m0.000s
    # sys	0m0.002s

    # we need to get the time in second 
    # cpu time is user time + system time
    # ./my_script/get_time.sh "$OUTPUT_DIR/time_$num.dat" > "$OUTPUT_DIR/time_$num.txt"
    ./my_script/extract_time < "$OUTPUT_DIR/time_$num.dat" > "$OUTPUT_DIR/time_$num.txt"
    CPU_TIME=$(grep "cpu" "$OUTPUT_DIR/time_$num.txt" | awk '{print $2}')
    REAL_TIME=$(grep "real" "$OUTPUT_DIR/time_$num.txt" | awk '{print $2}')
    echo "CPU Time: $CPU_TIME"
    echo "Real Time: $REAL_TIME"

    # use evaluator given by the TA
    bash $EVA "$INPUT_DIR/$benchmark" "$OUTPUT_DIR/output_$num.dat" $CPU_TIME > "$OUTPUT_DIR/eval_$num.txt"
    # check if the evaluator ran successfully
    cat $OUTPUT_DIR/eval_$num.txt | grep "Congratulations! Legal Solution!!"
    if [ $? -ne 0 ]; then
        echo "Error: $EXE $INPUT_DIR/$benchmark is illegal!"
    else
        # grep the cut size
        cutsize=$(cat $OUTPUT_DIR/eval_$num.txt | grep "Cut size" | awk '{print $5}')
        echo -e "\033[32mCut size: $cutsize\033[0m"
        # grep the last line of the output
        # the score could be 4.0 to 9.5
        score=$(tail -n 1 "$OUTPUT_DIR/eval_$num.txt" | grep -o '[0-9]\+\.[0-9]\+')
        echo "CPU time Score: $score" 
        # append the score to the file
        echo "$score" >> "$OUTPUT_DIR/cputime_score.txt"
    fi

    # evaluate the ouput use the real time
    bash $EVA "$INPUT_DIR/$benchmark" "$OUTPUT_DIR/output_$num.dat" $REAL_TIME > "$OUTPUT_DIR/eval_$num.txt"
    # check if the evaluator ran successfully
    cat $OUTPUT_DIR/eval_$num.txt | grep "Congratulations! Legal Solution!!"
    if [ $? -ne 0 ]; then
        echo "Error: $EXE $INPUT_DIR/$benchmark is illegal!"
    else
        # grep the last line of the output
        # the score could be 4.0 to 9.5
        score=$(tail -n 1 "$OUTPUT_DIR/eval_$num.txt" | grep -o '[0-9]\+\.[0-9]\+')
        # append the score to the file
        echo "real time Score: $score" 
        # append the score to the file
        echo "$score" >> "$OUTPUT_DIR/realtime_score.txt"
    fi

    # remove the time file
    rm "$OUTPUT_DIR/time_$num.dat"
    rm "$OUTPUT_DIR/time_$num.txt"
    rm "$OUTPUT_DIR/eval_$num.txt"
done

printf "\n\n"
# print all scores
echo "CPU time scores:"
cat "$OUTPUT_DIR/cputime_score.txt"
echo "Real time scores:"
cat "$OUTPUT_DIR/realtime_score.txt"
# calculate the average score
echo "Average CPU time score:"
awk '{s+=$1} END {print s/NR}' "$OUTPUT_DIR/cputime_score.txt"
echo "Average Real time score:"
awk '{s+=$1} END {print s/NR}' "$OUTPUT_DIR/realtime_score.txt"
# remove the score files
rm "$OUTPUT_DIR/cputime_score.txt"
rm "$OUTPUT_DIR/realtime_score.txt"
