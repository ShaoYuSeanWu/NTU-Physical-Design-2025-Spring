#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include "partitioner.h"
using namespace std;
using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;

int main(int argc, char** argv)
{
    fstream input, output;

    if (argc == 3) {
        input.open(argv[1], ios::in);
        output.open(argv[2], ios::out);
        if (!input) {
            cerr << "Cannot open the input file \"" << argv[1]
                 << "\". The program will be terminated..." << endl;
            exit(1);
        }
        if (!output) {
            cerr << "Cannot open the output file \"" << argv[2]
                 << "\". The program will be terminated..." << endl;
            exit(1);
        }
    }
    else {
        cerr << "Usage: ./fm <input file> <output file>" << endl;
        exit(1);
    }
    auto start_time = high_resolution_clock::now();
    Partitioner* partitioner = new Partitioner(input);
    auto readfile_time = high_resolution_clock::now();
    partitioner->partition();
    auto partition_time = high_resolution_clock::now();
    partitioner->printSummary();
    partitioner->writeResult(output);
    auto writefile_time = high_resolution_clock::now();

    duration<double, std::milli> readfile_duration = readfile_time - start_time;
    duration<double, std::milli> partition_duration = partition_time - readfile_time;
    duration<double, std::milli> writefile_duration = writefile_time - partition_time;
    duration<double, std::milli> total_duration = writefile_time - start_time;

    cout << "Read file time: " << readfile_duration.count() << " ms" << endl;
    cout << "Partition time: " << partition_duration.count() << " ms" << endl;
    cout << "Write file time: " << writefile_duration.count() << " ms" << endl;
    cout << "Total time: " << total_duration.count() << " ms" << endl;

    return 0;
}
