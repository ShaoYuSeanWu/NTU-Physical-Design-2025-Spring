#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <cassert>
#include <vector>
#include <cmath>
#include <map>
#include <unordered_map>
#include<cstdlib>
#include <set>
#include <unordered_set>
#include <forward_list>
#include <stack>
#include <chrono>
#include "cell.h"
#include "net.h"
#include "partitioner.h"
using namespace std;
using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;


void Partitioner::parseInput(fstream& inFile)
{
    string str;
    // Set balance factor
    inFile >> str;
    _bFactor = stod(str);

    // Set up whole circuit
    while (inFile >> str) {
        if (str == "NET") {
            string netName, cellName, tmpCellName = "";
            inFile >> netName;
            int netId = _netNum;
            _netArray.push_back(new Net(netName));
            _netName2Id[netName] = netId;
            while (inFile >> cellName) {
                if (cellName == ";") {
                    tmpCellName = "";
                    break;
                }
                else {
                    // a newly seen cell
                    if (_cellName2Id.count(cellName) == 0) {
                        int cellId = _cellNum;
                        _cellArray.push_back(new Cell(cellName, 0, cellId));
                        _cellName2Id[cellName] = cellId;
                        _cellArray[cellId]->addNet(netId);
                        _cellArray[cellId]->incPinNum();
                        _netArray[netId]->addCell(cellId);
                        ++_cellNum;
                        tmpCellName = cellName;
                    }
                    // an existed cell
                    else {
                        if (cellName != tmpCellName) {
                            assert(_cellName2Id.count(cellName) == 1);
                            int cellId = _cellName2Id[cellName];
                            _cellArray[cellId]->addNet(netId);
                            _cellArray[cellId]->incPinNum();
                            _netArray[netId]->addCell(cellId);
                            tmpCellName = cellName;
                        }
                    }
                }
            }
            ++_netNum;
        }
    }
    return;
}

// multilevel 2 way partition
// author: sean
void Partitioner::partition()
{
    coarsening();
    initPartition_cheng();
    partitionFM();
    // print initial partition result
    cout << endl;
    cout << "^^^^^^^^^^^^^^^^^^^^^^^^^^" << endl;
    cout << "initial partition result" << endl;
    cout << "cell number: " << _cellNum << endl;
    cout << "net number: " << _netNum << endl;
    cout << "cut size: " << _cutSize << endl;
    cout << "iteration number: " << _iterNum << endl;
    cout << "^^^^^^^^^^^^^^^^^^^^^^^^^^" << endl;
    cout << endl;
    unCoarsening();
    return;
}

/**
 * Generates an array of cell IDs ordered as a min-heap or max-heap based on the number of pins.
 *
 * @author sean
 * @param flag Determines the type of heap:
 *             - 0: Constructs a min-heap (cells with fewer pins are prioritized).
 *             - 1: Constructs a max-heap (cells with more pins are prioritized).
 * @return A dynamically allocated array of cell IDs ordered as a heap.
 *
 * The function first initializes the array with cell IDs. It then rearranges the array
 * into a heap structure using a bottom-up heap construction approach. The heap property
 * is maintained based on the number of pins associated with each cell.
 *
 * Note: The caller is responsible for deallocating the returned array.
 */
int* Partitioner::heapOrder(int flag) {
    int* order = new int[_cellNum];
    for (int i = 0; i < _cellNum; ++i) {
        order[i] = _cellArray[i]->getNode()->getId();
    }
    // start initial min heap according to the pin number
    for (int i=(_cellNum - 1) / 2; i >= 0; --i) {
        int j = i;
        while (j < _cellNum) {
            int left = 2 * j + 1;
            int right = 2 * j + 2;
            if (left >= _cellNum) {
                break;
            }
            if (right >= _cellNum) {
                right = left;
            }

            int changeIndex;
            if (flag == 0) {
                changeIndex = (_cellArray[order[left]]->getPinNum() < _cellArray[order[right]]->getPinNum()) ? left : right;
            }
            else {
                changeIndex = (_cellArray[order[left]]->getPinNum() > _cellArray[order[right]]->getPinNum()) ? left : right;
            }
            if (_cellArray[order[j]]->getPinNum() > _cellArray[order[changeIndex]]->getPinNum() && flag == 0) {
                swap(order[j], order[changeIndex]);
                j = changeIndex;
            }
            else if (_cellArray[order[j]]->getPinNum() < _cellArray[order[changeIndex]]->getPinNum() && flag == 1) {
                swap(order[j], order[changeIndex]);
                j = changeIndex;
            }
            else {
                break;
            }
        }
    }
    return order;
}

// coarsening phase
// if the cell number is larger than 2500, keep coarsening the cells
// can choose edge coarsening or first choice coarsening
// author: sean
void Partitioner::coarsening(){
    int coarseningNum = 0;
    while (_cellNum > 2500) {
        cout << "coarsening iter: " << coarseningNum << endl;
        
        _cellStack.push(_cellArray);
        _netStack.push(_netArray);
        _cellNumStack.push(_cellNum);
        _netNumStack.push(_netNum);
        _maxPinNumStack.push(_maxPinNum);
        
        auto start_time = high_resolution_clock::now();
        bool hasCoarsening = edgeCoarsening();
        // bool hasCoarsening = FirstChoiceCoarsening();
        auto end_time = high_resolution_clock::now();
        duration<double, std::milli> duration = end_time - start_time;
        
        if (!hasCoarsening || _cellNum == _cellNumStack.top()) {
            // if no cells are coarsened, break the loop
            _cellArray = _cellStack.top();
            _netArray = _netStack.top();
            _cellNum = _cellNumStack.top();
            _netNum = _netNumStack.top();
            _maxPinNum = _maxPinNumStack.top();
            
            // pop the stack
            _cellStack.pop();
            _netStack.pop();
            _cellNumStack.pop();
            _netNumStack.pop();
            _maxPinNumStack.pop();
            break;
        }
        cout << "coarsening time: " << duration.count() << " ms" << endl;
        cout << "---------------------------" << endl;
        
        coarseningNum++;
    }
    
    // coarsening summary
    cout << "==========================" << endl;
    cout << " coarsening summary" << endl;
    cout << "coarsening number: " << coarseningNum << endl;
    cout << "cell number: " << _cellNum << endl;
    cout << "net number: " << _netNum << endl;
    cout << "==========================" << endl;
    return;
}

// first choice coarsening
// author: sean
bool Partitioner::FirstChoiceCoarsening() {
    cout << "First Choice coarsening ... " << endl;
    auto start_time = high_resolution_clock::now();
    
    int *cellOrder = heapOrder(0); // cell id array ordering in min heap according to pin number
    
    _cellArray.clear();
    _netArray.clear();
    _cellNum = 0;
    _netNum = 0;
    const vector<Cell*> &prevCellArray = _cellStack.top();
    const vector<Net*> &prevNetArray = _netStack.top();
    unordered_map<int, float> cellMetricMap; // previous cell metric
    unordered_map<int, float> clausterMetricMap; // new clauster cell metric
    unordered_map<int, int> cellIdMap; // original cell id to new cell id
    for (size_t ii = 0; ii < prevCellArray.size(); ++ii) { // O(P)
        int i = cellOrder[ii]; // i is current cellId
        // if the cell is locked, means it is marked, already coarsened
        if (prevCellArray[i]->getLock()) {
            continue;
        }
        // if the cell is not locked, we need to find the neighbor cell with the max metric
        // find the cell with the max metric 1 / (netList.size() - 1)
        cellMetricMap.clear();
        clausterMetricMap.clear();
        float maxMetric = 0.0;
        for (const auto &netId : prevCellArray[i]->getNetList()) {
            // get the net
            Net* netptr = prevNetArray[netId];
            
            if (netptr->getSize() == 0 || netptr->getSize() == 1) {
                continue;
            }
            
            float metric = 1.0 / (netptr->getSize() - 1);
            // get the cell list of the net
            const unordered_set<int> &cellList = netptr->getCellList();
            // calculate each cell's metric
            for (const auto &cellId : cellList) {
                if (cellId == i) {
                    continue;
                }

                
                if (cellMetricMap.count(cellId) == 0) {
                    // cellMetricMap[cellId] = metric;
                    if (prevCellArray[cellId]->getLock()){
                        // the cell is already claustered
                        clausterMetricMap[cellIdMap[cellId]] = metric / _cellArray[cellIdMap[cellId]]->getSize();
                    }
                    else{
                        cellMetricMap[cellId] = metric / prevCellArray[cellId]->getSize();
                    }
                }
                else {
                    // cellMetricMap[cellId] += metric;
                    if (prevCellArray[cellId]->getLock()){
                        // the cell is already claustered
                        clausterMetricMap[cellIdMap[cellId]] += metric / _cellArray[cellIdMap[cellId]]->getSize();
                    }
                    else{
                        cellMetricMap[cellId] += metric / prevCellArray[cellId]->getSize();
                    }
                }
            }
        }
        // find the max metric cell
        int maxCellId = -1;
        bool isClausterNode = false;
        for (auto cellId : cellMetricMap) {
            if (cellId.second > maxMetric) {
                maxMetric = cellId.second;
                maxCellId = cellId.first;
            }
        }
        for (auto cellId : clausterMetricMap) {
            if (cellId.second > maxMetric) {
                maxMetric = cellId.second;
                maxCellId = cellId.first;
                isClausterNode = true;
            }
        }

        // if the max metric cell is not -1, coarsen cell i and maxCellId
        if (maxCellId != -1 && !isClausterNode) {
            // coarsen the two cells that haven't been claustered
            // string cellName = prevCellArray[i]->getName() + "_" + prevCellArray[maxCellId]->getName();
            string cellName = "";
            Cell * cellptr = new Cell(cellName, 0, _cellNum);
            cellptr->setSize(prevCellArray[i]->getSize() + prevCellArray[maxCellId]->getSize());
            cellptr->addCellGroup(i);
            cellptr->addCellGroup(maxCellId);
            // union the net list of the two cells, and setNetList
            unordered_set<int> netList = prevCellArray[i]->getNetList();
            unordered_set<int> maxNetList = prevCellArray[maxCellId]->getNetList();
            netList.insert(maxNetList.begin(), maxNetList.end());
            cellptr->setNetList(netList);
            cellptr->setPinNum(netList.size());
            // end coarsening the cells

            cellIdMap[i] = _cellNum;
            cellIdMap[maxCellId] = _cellNum;
            _cellArray.push_back(cellptr);
            _cellNum++;
            
            // mark the cells as locked
            prevCellArray[i]->lock();
            prevCellArray[maxCellId]->lock();
        }
        else if (maxCellId != -1) {
            // coarsen a clauster and a cell
            Cell *cellptr = _cellArray[maxCellId];
            cellptr->setSize(prevCellArray[i]->getSize() + cellptr->getSize());
            cellptr->addCellGroup(i);
            // union the net list of the two cells, and setNetList
            for (const auto &n : prevCellArray[i]->getNetList()) {
                cellptr->addNet(n);
            }
            cellptr->setPinNum(cellptr->getNetList().size());
            // end coarsening the cells
            cellIdMap[i] = maxCellId;
            // mark the cells as locked
            prevCellArray[i]->lock();
            

        }
    }
    
    auto phase1_end_time = high_resolution_clock::now();

    if (_cellArray.size() == 0) {
        // if no cells are coarsened, return false
        // cout << "no cells are coarsened" << endl;
        return false;
    }
    
    // if some cells are not locked, we need to add them to the cell array
    for (size_t i = 0; i < prevCellArray.size(); ++i) {
        if (prevCellArray[i]->getLock()) {
            continue;
        }
        cellIdMap[i] = _cellNum;
        string cellName = prevCellArray[i]->getName();
        Cell* cellptr = new Cell(cellName, 0, _cellNum);
        cellptr->setSize(prevCellArray[i]->getSize());
        cellptr->setPinNum(prevCellArray[i]->getPinNum());
        cellptr->setNetList(prevCellArray[i]->getNetList());
        cellptr->addCellGroup(i);
        _cellArray.push_back(cellptr);
        _cellNum++;
    }

    auto phase2_end_time = high_resolution_clock::now();
    
    // adjust the net array and the netlist in the cell
    // reduce the net number and update the net array
    adjustNetInfo(cellIdMap);

    auto phase3_end_time = high_resolution_clock::now();
    duration<double, std::milli> phase1_duration = phase1_end_time - start_time;
    duration<double, std::milli> phase2_duration = phase2_end_time - phase1_end_time;
    duration<double, std::milli> phase3_duration = phase3_end_time - phase2_end_time;
    cout << "phase 1 time: " << phase1_duration.count() << " ms" << endl;
    cout << "phase 2 time: " << phase2_duration.count() << " ms" << endl;
    cout << "adjustNetInfo time: " << phase3_duration.count() << " ms" << endl;
    
    return true;
}

// edge coarsening
// author: sean
bool Partitioner::edgeCoarsening() {
    cout << "edge coarsening ... " << endl;
    auto start_time = high_resolution_clock::now();
    
    int *cellOrder = heapOrder(1); // cell id array ordering in min heap according to pin number
    _cellArray.clear();
    _netArray.clear();
    _cellNum = 0;
    _netNum = 0;
    const vector<Cell*> &prevCellArray = _cellStack.top();
    const vector<Net*> &prevNetArray = _netStack.top();
    unordered_map<int, float> cellMetricMap;
    unordered_map<int, int> cellIdMap; // original cell id to new cell id
    for (size_t ii = 0; ii < prevCellArray.size(); ++ii) { // O(P)
        int i = cellOrder[ii]; // i is current cellId
        // if the cell is locked, means it is marked, already coarsened
        if (prevCellArray[i]->getLock()) {
            continue;
        }
        // if the cell is not locked, we need to find the neighbor cell with the max metric
        // find the cell with the max metric 1 / (netList.size() - 1)
        cellMetricMap.clear();
        float maxMetric = 0.0;
        for (const auto &netId : prevCellArray[i]->getNetList()) {
            // get the net
            Net* netptr = prevNetArray[netId];
            
            if (netptr->getSize() == 0 || netptr->getSize() == 1) {
                continue;
            }
            
            float metric = 1.0 / (netptr->getSize() - 1);
            // get the cell list of the net
            const unordered_set<int>& cellList = netptr->getCellList();
            // calculate each cell's metric
            for (const auto &cellId : cellList) {
                if (cellId == i) {
                    continue;
                }
                if (prevCellArray[cellId]->getLock()) {
                    continue;
                }
                if (cellMetricMap.count(cellId) == 0) {
                    // cellMetricMap[cellId] = metric;
                    cellMetricMap[cellId] = metric / prevCellArray[cellId]->getSize();
                }
                else {
                    // cellMetricMap[cellId] += metric;
                    cellMetricMap[cellId] += metric / prevCellArray[cellId]->getSize();
                }
            }
        }
        // find the max metric cell
        int maxCellId = -1;
        for (auto cellId : cellMetricMap) {
            if (cellId.second > maxMetric) {
                maxMetric = cellId.second;
                maxCellId = cellId.first;
            }
        }
        // if the max metric cell is not -1, coarsen cell i and maxCellId
        if (maxCellId != -1) {
            // coarsen the cells
            // string cellName = prevCellArray[i]->getName() + "_" + prevCellArray[maxCellId]->getName();
            string cellName = "";
            Cell * cellptr = new Cell(cellName, 0, _cellNum);
            cellptr->setSize(prevCellArray[i]->getSize() + prevCellArray[maxCellId]->getSize());
            cellptr->addCellGroup(i);
            cellptr->addCellGroup(maxCellId);
            // union the net list of the two cells, and setNetList
            // unordered_set<int> netList = prevCellArray[i]->getNetList();
            // unordered_set<int> maxNetList = prevCellArray[maxCellId]->getNetList();
            // netList.insert(maxNetList.begin(), maxNetList.end());
            unordered_set<int> netList;
            netList.insert(prevCellArray[i]->getNetList().begin(), prevCellArray[i]->getNetList().end());
            netList.insert(prevCellArray[maxCellId]->getNetList().begin(), prevCellArray[maxCellId]->getNetList().end());
            cellptr->setNetList(netList);
            cellptr->setPinNum(netList.size());
            // end coarsening the cells

            cellIdMap[i] = _cellNum;
            cellIdMap[maxCellId] = _cellNum;
            _cellArray.push_back(cellptr);
            _cellNum++;
            
            // mark the cells as locked
            prevCellArray[i]->lock();
            prevCellArray[maxCellId]->lock();
        }
    }
    
    if (_cellArray.size() == 0) {
        // if no cells are coarsened, return false
        // cout << "no cells are coarsened" << endl;
        return false;
    }

    auto phase1_end_time = high_resolution_clock::now();
    
    // if some cells are not locked, we need to add them to the cell array
    for (size_t i = 0; i < prevCellArray.size(); ++i) {
        if (prevCellArray[i]->getLock()) {
            continue;
        }
        cellIdMap[i] = _cellNum;
        string cellName = prevCellArray[i]->getName();
        Cell* cellptr = new Cell(cellName, 0, _cellNum);
        cellptr->setSize(prevCellArray[i]->getSize());
        cellptr->setPinNum(prevCellArray[i]->getPinNum());
        cellptr->setNetList(prevCellArray[i]->getNetList());
        cellptr->addCellGroup(i);
        _cellArray.push_back(cellptr);
        _cellNum++;
    }

    auto phase2_end_time = high_resolution_clock::now();
    
    // adjust the net array and the netlist in the cell
    // reduce the net number and update the net array
    adjustNetInfo(cellIdMap);

    auto phase3_end_time = high_resolution_clock::now();
    duration<double, std::milli> phase1_duration = phase1_end_time - start_time;
    duration<double, std::milli> phase2_duration = phase2_end_time - phase1_end_time;
    duration<double, std::milli> phase3_duration = phase3_end_time - phase2_end_time;
    cout << "phase 1 time: " << phase1_duration.count() << " ms" << endl;
    cout << "phase 2 time: " << phase2_duration.count() << " ms" << endl;
    cout << "adjustNetInfo time: " << phase3_duration.count() << " ms" << endl;
    
    return true;
}

// adjust the net array and the netlist in the cell
// reduce the net number and update the net array
// author: sean
void Partitioner::adjustNetInfo(const unordered_map<int, int>& cellIdMap) {
    map<int, int> netIdMap; // original net id to new net id
    
    // build new netArray
    for (size_t i = 0; i < _netStack.top().size(); ++i) {
        Net* netptr = _netStack.top()[i];
        const unordered_set<int>& cellList = netptr->getCellList();
        unordered_set<int> newCellList;
        for (const auto& cellId : cellList) {
            // get the new cell id
            // debug
            if (cellIdMap.count(cellId) == 0) {
                cerr << "Error: cell id not found in cellIdMap\n";
                cerr << "cellId: " << cellId << "\n";
                cerr << "netId: " << i << "\n";
                continue;
            }
            // end debug
            int newCellId = cellIdMap.at(cellId);
            // add the new cell id to the new cell list
            newCellList.insert(newCellId);
        }
        // abandon the net if the new cell list is empty or only one cell
        if (newCellList.size() == 0 || newCellList.size() == 1) {
            continue;
        }
        
        netIdMap[i] = _netNum;

        string netName = netptr->getName();
        // create a new net
        Net* newNet = new Net(netName);
        newNet->setCellList(newCellList);
        _netArray.push_back(newNet);
        ++_netNum;
    }
    
    // update the net list in the cell
    int maxSize = 0;
    int minSize = 1000000;
    for (size_t i = 0; i < _cellArray.size(); ++i) {
        Cell* cellptr = _cellArray[i];
        const unordered_set<int>& netList = cellptr->getNetList();
        unordered_set<int> newNetList;
        for (const auto& netId : netList) {
            if (netIdMap.count(netId) == 0) {
                continue;
            }
            // get the new net id
            // add the new net id to the new net list
            newNetList.insert(netIdMap.at(netId));
        }
        // set the new net list to the cell
        cellptr->setNetList(newNetList);
        cellptr->setPinNum(newNetList.size());

        // debug
        if (cellptr->getSize() > maxSize) {
            maxSize = cellptr->getSize();
        }
        if (cellptr->getSize() < minSize) {
            minSize = cellptr->getSize();
        }
        // end debug
    }

    // debug
    cout << "max size: " << maxSize << endl;
    cout << "min size: " << minSize << endl;
    // end debug
    return;
}

// perform uncoarsening and FM refinement
// author: sean
void Partitioner::unCoarsening() {
    // uncoarsening
    while (!_cellNumStack.empty()) {
        cout << "uncoarsening number " << _cellNumStack.size() - 1 << endl;
        
        // update the parity of the cell
        for (auto cellptr : _cellArray) {
            for (auto cellId : cellptr->getCellGroup()) {
                _cellStack.top()[cellId]->setPart(cellptr->getPart());
            }
        }
        clear();
        
        // restore the cell and net array
        _cellArray = _cellStack.top();
        _netArray = _netStack.top();
        _cellNum = _cellNumStack.top();
        _netNum = _netNumStack.top();
        _maxPinNum = _maxPinNumStack.top();
        
        // pop the stack
        _cellStack.pop();
        _netStack.pop();
        _cellNumStack.pop();
        _netNumStack.pop();
        _maxPinNumStack.pop();
        
        cout << "cell number: " << _cellNum << endl;
        cout << "net number: " << _netNum << endl;
        cout << "start refinement ..." << endl;
        auto p_start_time = high_resolution_clock::now();
        updatePartGainInfo();
        partitionFM();
        auto p_end_time = high_resolution_clock::now();
        duration<double, std::milli> p_duration = p_end_time - p_start_time;
        cout << "end refinement, time: " << p_duration.count() << " ms" << endl;
        cout << "iteration number: " << _iterNum << endl;
        cout << "cut size: " << _cutSize << endl;
        cout << "==========================" << endl << endl;
    }
    return;
}

// perfrom FM partition
// author: sean
void Partitioner::partitionFM()
{
    int minPartSize = ceil((_partSize[0] + _partSize[1]) * (1.0 - _bFactor) / 2.0);
    // calculate the _maxPinNum
    _maxPinNum = 0;
    for (auto cellptr : _cellArray) {
        if (cellptr->getPinNum() > _maxPinNum) {
            _maxPinNum = cellptr->getPinNum();
        }
    }


    // calculate initial cutsize
    _cutSize = 0;
    for (auto netptr : _netArray) {
        if (netptr->getPartCount(0) > 0 && netptr->getPartCount(1) > 0) {
            ++_cutSize;
        }
    }

    // iterative loop
    _iterNum = 0;
    while (_maxAccGain > 0 || _iterNum == 0  || _partSize[0] < minPartSize || _partSize[1] < minPartSize) {
        
        // initial bucket list
        initBList();
        
        // keep moving unlocked cells until no cell can be moved
        _moveNum = 0;
        _moveStack.clear();
        while (true) {
            int cellId = chooseCell();
            if (cellId == -1) {
                break;
            }
            Cell* cellptr = _cellArray[cellId];
            // lock the cell
            cellptr->lock();
            bListDelete(cellptr);
            // add the cell to the move list
            _moveNum++;
            _moveStack.push_back(cellId);
            updateGain(cellId);

        }
        
        // calculate the accumulative gain
        _accGain = 0;
        _maxAccGain = -1 * _maxPinNum;
        _bestMoveNum = 0;
        for (int i=0; i<_moveNum; ++i) {
            _accGain += _cellArray[_moveStack[i]]->getGain();
            if (_accGain > _maxAccGain) {
                _maxAccGain = _accGain;
                _bestMoveNum = i + 1;
            }
        }
        // move cell according to the max accumulative gain
        // change back the partition of the cells that are moved, but we don't want to move
        for (int i=_bestMoveNum; i<_moveNum; ++i) {
            _cellArray[_moveStack[i]]->move();
        }


        updatePartGainInfo();
        // update the cutsize by adding the max accumulative gain
        if (_maxAccGain > 0 || (_partSize[0] < minPartSize || _partSize[1] < minPartSize)) {
            _cutSize -= _maxAccGain;
        }
        
        ++_iterNum;
    }

    return;
}

// initialize the partition
// author: sean
void Partitioner::initPartition()
{
    // initial partition
    int initApartNum = ceil(_cellNum * (1.0 - _bFactor) / 2.0);
    for (size_t i = 0 ; i < _cellArray.size(); i++) {
        if (i % 2 == 0) {
            _cellArray[i]->setPart(0);
            ++_partNum[0];
        }
        else {
            _cellArray[i]->setPart(1);
            ++_partNum[1];
        }
    }
    updatePartGainInfo();
    return;
}

// initialize the partition
// author: sean
void Partitioner::initPartition_2()
{
    // initial partition
    int initApartNum = ceil(_cellNum * (1.0 - _bFactor * 1.2) / 2.0);
    // int p;
    for (size_t i = 0 ; i < _cellArray.size(); i++) {
        // p = rand() % 2;
        if (i < initApartNum) {
            _cellArray[i]->setPart(0);
            ++_partNum[0];
        }
        else {
            _cellArray[i]->setPart(1);
            ++_partNum[1];
        }
    }
    updatePartGainInfo();
    return;
}

void Partitioner::initPartition_cheng(){
    int n = 0;
    bool end = false;
    for(int i=0;i<_netNum;i++){
        const unordered_set<int> &list = _netArray[i]->getCellList();
        for(const auto& cellId : list){
            if(!_cellArray[cellId]->getPart()){
                n++;
                _cellArray[cellId]->move();
                _partNum[1]++;
            }
            if (n>_cellNum*0.5){
                end = true;
                // break;
                // return;   
            }
        }
        if (end){
            break;
        }
    }
    _partNum[0] = _cellNum - _partNum[1];
    // cout << "part 0: " << _partSize[0] << endl;
    // cout << "part 1: " << _partSize[1] << endl;
    updatePartGainInfo();
    return;
}

// update the partition information
// update the part size
// update the part count of each net (F/T array)
// unlock all cells
// calculate each cell's gain
// author: sean
void Partitioner::updatePartGainInfo() {
    // calculate the part size while unclock all cells
    // also set the gain of each cell to 0
    _partNum[0] = 0;
    _partNum[1] = 0;
    _partSize[0] = 0;
    _partSize[1] = 0;
    _maxCellSize = 0;
    _minCellSize = 1000000;
    for (auto cellptr : _cellArray) {
        cellptr->unlock();
        cellptr->setGain(0);
        if (cellptr->getPart() == 0) {
            ++_partNum[0];
            _partSize[0] += cellptr->getSize();
        }
        else {
            ++_partNum[1];
            _partSize[1] += cellptr->getSize();
        }
        if (cellptr->getSize() > _maxCellSize) {
            _maxCellSize = cellptr->getSize();
        }
        if (cellptr->getSize() < _minCellSize) {
            _minCellSize = cellptr->getSize();
        }
    }
    
    // calculate the part count of each net (from/to array)
    for (auto netptr : _netArray) {
        // reset the part count of the net
        netptr->setPartCount(0, 0);
        netptr->setPartCount(1, 0);
        unordered_set<int> cellList = netptr->getCellList();
        for (auto cellId : cellList) {
            if (_cellArray[cellId]->getPart() == 0) {
                netptr->incPartCount(0);
            }
            else {
                netptr->incPartCount(1);
            }
        }
    }

    initGain();
    return;

}

// initialize the gain of each cell
// note that the cell gain need to be set to 0 before calling this function
// author: sean
void Partitioner::initGain(){
    for (auto netptr : _netArray){
        unordered_set<int> cellList = netptr->getCellList();
        for (auto cellId : cellList){
            bool fromPart = _cellArray[cellId]->getPart();
            bool ToPart = !fromPart;
            if (netptr->getPartCount(fromPart) == 1){
                _cellArray[cellId]->incGain();
            }
            if (netptr->getPartCount(ToPart) == 0){
                _cellArray[cellId]->decGain();
            }
        }
    }
    return;
}

// update the unlocked cells' gain
// update the bucket list
// author: sean
void Partitioner::updateGain(int cellId) {
    bool fromPart = _cellArray[cellId]->getPart();
    bool toPart = !fromPart;
    
    // update gains according to the To part
    // T(n) = 0 then increment gains of all unlocked cells
    // T(n) = 1 then decrement gains only To part unlocked cells
    for (auto netId : _cellArray[cellId]->getNetList()) {
        int numTopart = _netArray[netId]->getPartCount(toPart);
        for (auto testCellId : _netArray[netId]->getCellList()) {
            if (numTopart == 0 && !_cellArray[testCellId]->getLock()) {
                bListDelete(_cellArray[testCellId]);
                _cellArray[testCellId]->incGain();
                bListInsert(_cellArray[testCellId]);
            }
            else if (numTopart == 1 && !_cellArray[testCellId]->getLock() && _cellArray[testCellId]->getPart() == toPart) {
                bListDelete(_cellArray[testCellId]);
                _cellArray[testCellId]->decGain();
                bListInsert(_cellArray[testCellId]);
            }
        }

        // update the part count of the net
        _netArray[netId]->decPartCount(fromPart);
        _netArray[netId]->incPartCount(toPart);
    }

    _cellArray[cellId]->move();
    _partNum[fromPart]--;
    _partNum[toPart]++;
    _partSize[fromPart] -= _cellArray[cellId]->getSize();
    _partSize[toPart] += _cellArray[cellId]->getSize();
    
    // update gains accordint to the From part
    // if F(n) = 0 then decrement gains of all unlocked cells
    // if F(n) = 1 then increment gains only From part unlocked cells
    for (auto netId : _cellArray[cellId]->getNetList()) {
        int numFrompart = _netArray[netId]->getPartCount(fromPart);
        for (auto testCellId : _netArray[netId]->getCellList()) {
            if (numFrompart == 0 && !_cellArray[testCellId]->getLock()) {
                bListDelete(_cellArray[testCellId]);
                _cellArray[testCellId]->decGain();
                bListInsert(_cellArray[testCellId]);
            }
            else if (numFrompart == 1 && !_cellArray[testCellId]->getLock() && _cellArray[testCellId]->getPart() == fromPart) {
                bListDelete(_cellArray[testCellId]);
                _cellArray[testCellId]->incGain();
                bListInsert(_cellArray[testCellId]);
            }
        }
    }
    return;
}

// initialize BucketList, meanwhile
// calculate _maxGainCell
// author: sean
void Partitioner::initBList() {
    _bList[0].clear();
    _bList[1].clear();
    for (auto cellptr : _cellArray) {
        bListInsert(cellptr);
    }
}

// delete a node in a bList according to its part and gain
// author: sean
void Partitioner::bListDelete(Cell* cell) {
    int gain = cell->getGain();
    Node* node = cell->getNode();
    int part = cell->getPart();
    Node* prev = node->getPrev();
    Node* next = node->getNext();
    if (prev == nullptr && next == nullptr) {
        // the node is the only node in the list
        _bList[part].erase(gain);
        // _bList[part][gain] = nullptr;
    }
    else if (prev == nullptr) {
        // the node is the head of the list
        _bList[part][gain] = next;
        next->setPrev(nullptr);
        node->setNext(nullptr);
    }
    else if (next == nullptr) {
        // the node is the tail of the list
        prev->setNext(nullptr);
        node->setPrev(nullptr);
    }
    else {
        // the node in the middle of the list
        prev->setNext(next);
        next->setPrev(prev);
        node->setPrev(nullptr);
        node->setNext(nullptr);
    }
    return;
}

// insert a node in a bList according to its part and gain
// insert into the front of bucket list
// author: sean
void Partitioner::bListInsert(Cell* cell) {
    int gain = cell->getGain();
    Node* node = cell->getNode();
    int part = cell->getPart();
    if (_bList[part].count(gain) == 0) {
        _bList[part][gain] = node;
        node->setPrev(nullptr);
        node->setNext(nullptr);
    }
    else {
        Node* head = _bList[part][gain];
        node->setPrev(nullptr);
        node->setNext(head);
        if (head != nullptr) {
            // means the the gain is empty
            head->setPrev(node);
        }
        _bList[part][gain] = node;
    }
    return;
}


int Partitioner::chooseCell() {
    map<int, Node*>::reverse_iterator it0 = _bList[0].rbegin();
    map<int, Node*>::reverse_iterator it1 = _bList[1].rbegin();
    if (it0 == _bList[0].rend() && it1 == _bList[1].rend()) {
        // no cell in two buketlists
        return -1;
    }
    
    int minPartSize = (_partSize[0] + _partSize[1]) * (1.0 - _bFactor) / 2.0;
    int MaxPartSize = (_partSize[0] + _partSize[1]) * (1.0 + _bFactor) / 2.0;
    bool allowPartA2PartB = (_partSize[0] - _minCellSize >= minPartSize && _partSize[1] + _minCellSize <= MaxPartSize);
    bool allowPartB2PartA = (_partSize[1] - _minCellSize >= minPartSize && _partSize[0] + _minCellSize <= MaxPartSize);
    
    if (!allowPartA2PartB && !allowPartB2PartA) {
        // no cell can be moved
        return -1;
    }
    if (!allowPartA2PartB && allowPartB2PartA) {
        // check if there are cells in part B can move to part A
        for (auto it = _bList[1].rbegin(); it != _bList[1].rend(); ++it) {
            Node* node = it->second;
            if (node == nullptr) {
                continue;
            }
            Cell *cellptr = _cellArray[node->getId()];
            bool balance = (_partSize[1] - cellptr->getSize() >= minPartSize && _partSize[0] + cellptr->getSize() <= MaxPartSize);
            while (node != nullptr && !balance) {
                node = node->getNext();
                if (node == nullptr) {
                    break;
                }
                cellptr = _cellArray[node->getId()];
                balance = (_partSize[1] - cellptr->getSize() >= minPartSize && _partSize[0] + cellptr->getSize() <= MaxPartSize);
            }
            if (node != nullptr) {
                return node->getId();
            }

        }
        // no cell in part B can be moved to part A
        return -1;
    }
    if (allowPartA2PartB && !allowPartB2PartA) {
        // check if there are cells in part A can move to part B
        for (auto it = _bList[0].rbegin(); it != _bList[0].rend(); ++it) {
            Node* node = it->second;
            if (node == nullptr) {
                continue;
            }
            Cell *cellptr = _cellArray[node->getId()];
            bool balance = (_partSize[0] - cellptr->getSize() >= minPartSize && _partSize[1] + cellptr->getSize() <= MaxPartSize);
            while (node != nullptr && !balance) {
                node = node->getNext();
                if (node == nullptr) {
                    break;
                }
                cellptr = _cellArray[node->getId()];
                balance = (_partSize[0] - cellptr->getSize() >= minPartSize && _partSize[1] + cellptr->getSize() <= MaxPartSize);
            }
            if (node != nullptr) {
                return node->getId();
            }
        }
        // no cell in part A can be moved to part B
        return -1;
    }
    if (allowPartA2PartB && allowPartB2PartA) {
        // check both part A and part B in bList
        Node *node;
        while (it0 != _bList[0].rend() || it1 != _bList[1].rend()) {
            bool bothExist = (it0 != _bList[0].rend() && it1 != _bList[1].rend());
            if ((it1 == _bList[1].rend()) || (bothExist && it0->first > it1->first)) {
                // check if there are cells in part A can move to part B
                node = it0->second;
                while (node != nullptr) {
                    // check balance
                    if (_partSize[0] - _cellArray[node->getId()]->getSize() >= minPartSize && _partSize[1] + _cellArray[node->getId()]->getSize() <= MaxPartSize) {
                        return node->getId();
                    }
                    node = node->getNext();
                }
                //
                ++it0;
            }
            else if (it0 == _bList[0].rend() || (bothExist && it1->first > it0->first)) {
                // check if there are cells in part B can move to part A
                node = it1->second;
                while (node != nullptr) {
                    // check balance
                    if (_partSize[1] - _cellArray[node->getId()]->getSize() >= minPartSize && _partSize[0] + _cellArray[node->getId()]->getSize() <= MaxPartSize) {
                        return node->getId();
                    }
                    node = node->getNext();
                }
                ++it1;
            }
            else {
                // gain is the same
                // check bList[0] and bList[1] alternatively
                node = it0->second;
                Node* node2 = it1->second;
                while (node != nullptr || node2 != nullptr) {
                    if (node != nullptr) {
                        // check balance
                        if (_partSize[0] - _cellArray[node->getId()]->getSize() >= minPartSize && _partSize[1] + _cellArray[node->getId()]->getSize() <= MaxPartSize) {
                            return node->getId();
                        }
                        node = node->getNext();
                    }
                    if (node2 != nullptr) {
                        // check balance
                        if (_partSize[1] - _cellArray[node2->getId()]->getSize() >= minPartSize && _partSize[0] + _cellArray[node2->getId()]->getSize() <= MaxPartSize) {
                            return node2->getId();
                        }
                        node2 = node2->getNext();
                    }
                }
                ++it0;
                ++it1;
            }
        }
        return -1;
    }
    return -1;
}


void Partitioner::bucketListDebug(){
    bool is_wrong = false;
    for (int i=0; i<2; ++i){
        // cout << "part " << i << endl;
        for (auto it = _bList[i].begin(); it != _bList[i].end(); ++it){
            // cout << "gain: " << it->first << endl;
            // check if the gain is out of range
            if (it->first > _maxPinNum || it->first < -1 * _maxPinNum){
                cerr << "Error: gain out of range\n";
                cerr << "gain: " << it->first << "\n";
                is_wrong = true;
                break;
            }
            Node* node = it->second;
            // cout << "cell: ";
            while (node != nullptr){
                // cout << node->getId() << " ";
                if (_cellArray[node->getId()]->getPart() != i){
                    cerr << "Error: cell part not match\n";
                    cerr << "cell part: " << _cellArray[node->getId()]->getPart() << "\n";
                    cerr << "part: " << i << "\n";
                    is_wrong = true;
                }
                if (_cellArray[node->getId()]->getGain() != it->first){
                    cerr << "Error: cell gain not match\n";
                    cerr << "cell gain: " << _cellArray[node->getId()]->getGain() << "\n";
                    cerr << "gain: " << it->first << "\n";
                    is_wrong = true;
                }
                if (is_wrong){
                    break;
                }
                node = node->getNext();
            }
        }
        if (is_wrong){
            break;
        }
    }
    return;
}

void Partitioner::printSummary() const
{
    cout << endl;
    cout << "==================== Summary ====================" << endl;
    cout << " Cutsize: " << _cutSize << endl;
    cout << " Total cell number: " << _cellNum << endl;
    cout << " Total net number:  " << _netNum << endl;
    cout << " Cell Number of partition A: " << _partNum[0] << endl;
    cout << " Cell Number of partition B: " << _partNum[1] << endl;
    cout << "=================================================" << endl;
    cout << endl;
    return;
}


void Partitioner::writeResult(fstream& outFile)
{
    stringstream buff;
    buff << _cutSize;
    outFile << "Cutsize = " << buff.str() << '\n';
    buff.str("");
    buff << _partNum[0];
    outFile << "G1 " << buff.str() << '\n';
    for (size_t i = 0, end = _cellArray.size(); i < end; ++i) {
        if (_cellArray[i]->getPart() == 0) {
            outFile << _cellArray[i]->getName() << " ";
        }
    }
    outFile << ";\n";
    buff.str("");
    buff << _partNum[1];
    outFile << "G2 " << buff.str() << '\n';
    for (size_t i = 0, end = _cellArray.size(); i < end; ++i) {
        if (_cellArray[i]->getPart() == 1) {
            outFile << _cellArray[i]->getName() << " ";
        }
    }
    outFile << ";\n";
    return;
}

void Partitioner::clear()
{
    for (size_t i = 0, end = _cellArray.size(); i < end; ++i) {
        delete _cellArray[i];
    }
    for (size_t i = 0, end = _netArray.size(); i < end; ++i) {
        delete _netArray[i];
    }
    return;
}
