#ifndef PARTITIONER_H
#define PARTITIONER_H

#include <fstream>
#include <vector>
#include <map>
#include <unordered_map>
#include <stack>
#include "cell.h"
#include "net.h"
using namespace std;

class Partitioner
{
public:
    // constructor and destructor
    Partitioner(fstream& inFile) :
        _cutSize(0), _netNum(0), _cellNum(0), _maxPinNum(0), _bFactor(0),
        _accGain(0), _maxAccGain(0), _iterNum(0) {
        parseInput(inFile);
        _partNum[0] = 0;
        _partNum[1] = 0;
        _partSize[0] = 0;
        _partSize[1] = 0;
    }
    ~Partitioner() {
        clear();
    }

    // basic access methods
    int getCutSize() const          { return _cutSize; }
    int getNetNum() const           { return _netNum; }
    int getCellNum() const          { return _cellNum; }
    double getBFactor() const       { return _bFactor; }
    int getPartSize(int part) const { return _partSize[part]; }

    // modify method
    void parseInput(fstream& inFile);
    void partition();
    void partitionFM();

    // member functions about reporting
    void printSummary() const;
    void reportNet() const;
    void reportCell() const;
    void writeResult(fstream& outFile);

    // sean's functions
    void initPartition();
    void initPartition_2();
    void initPartition_cheng();
    void updatePartGainInfo();
    void initGain();
    void initBList();
    int chooseCell();
    void updateGain(int cellId);
    void bListDelete(Cell* cell);
    void bListInsert(Cell* cell);
    void bucketListDebug();
    
    void coarsening();
    bool edgeCoarsening();
    bool FirstChoiceCoarsening();
    void adjustNetInfo(const unordered_map<int, int>& cellIdMap);
    int* heapOrder(int flag); // cell id array ordering in min/max heap according to pin number
    void unCoarsening();

private:
    int                 _cutSize;       // cut size
    int                 _partNum[2];   // cell number of partition A(0) and B(1)
    int                 _partSize[2];   // size of partition A(0) and B(1)
    int                 _netNum;        // number of nets
    int                 _cellNum;       // number of cells
    int                 _maxPinNum;     // Pmax for building bucket list
    double              _bFactor;       // the balance factor to be met
    vector<Net*>        _netArray;      // net array of the circuit
    vector<Cell*>       _cellArray;     // cell array of the circuit
    map<int, Node*>     _bList[2];      // bucket list of partition A(0) and B(1)
    map<string, int>    _netName2Id;    // mapping from net name to id
    map<string, int>    _cellName2Id;   // mapping from cell name to id

    stack<vector<Cell*> > _cellStack;    // stack of cell array
    stack<vector<Net*> > _netStack;      // stack of net array
    stack<int> _cellNumStack;          // stack of cell number
    stack<int> _netNumStack;           // stack of net number
    stack<int> _maxPinNumStack;        // stack of max pin number

    int                _maxCellSize;   // maximum cell size
    int                _minCellSize;   // minimum cell size

    int                 _accGain;       // accumulative gain
    int                 _maxAccGain;    // maximum accumulative gain
    int                 _moveNum;       // number of cell movements
    int                 _iterNum;       // number of iterations
    int                 _bestMoveNum;   // store best number of movements
    int                 _unlockNum[2];  // number of unlocked cells
    vector<int>         _moveStack;     // history of cell movement

    // Clean up partitioner
    void clear();
};

#endif  // PARTITIONER_H
