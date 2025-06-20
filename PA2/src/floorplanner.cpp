#include<fstream>
#include <iostream>
#include <unordered_map>
#include <cstdlib>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>
#include <queue>
#include <stack>
#include <numeric> // for std::accumulate
#include "floorplanner.h"
#include "module.h"
#include "contour.h"
using namespace std;

size_t Block::_maxX = 0;
size_t Block::_maxY = 0;

Floorplanner::Floorplanner(std::fstream& input_blk, std::fstream& input_net, double alpha)

{
    // initialize all variables
    _prevCost.setCost(1e10, 0.0, 0, 0, 0, false);
    _bestCost.setCost(1e10, 0.0, 0, 0, 0, false);
    _cost.setCost(1e10, 0.0, 0, 0, 0, false);

    // initialize all data members
    _numBlocks = 0;
    _numTerminals = 0;
    _numNets = 0;
    _outlineWidth = 0;
    _outlineHeight = 0;
    _givenAlpha = alpha;
    _bSTree = nullptr;

    _numIter = 0;
    _numIterPerTemp = 0;
    _temperature = 0.0;
    _coolTemp = 0.0;
    _coolRate = 0.0;
    _alpha = 0.0;
    _beta = 0.0;
    _gamma = 0.0;
    _normArea = 1.0;
    _normHPWL = 1.0;
    _normAspectRatio = 1.0;
    _normExceedArea = 1.0;

    // read block and net file
    readBlock(input_blk);
    readNet(input_net);

}

Floorplanner::~Floorplanner()
{
    // delete all blocks
    for (auto block : _blockVec) {
        delete block;
    }
    _blockVec.clear();

    // delete all nets
    for (auto net : _netVec) {
        delete net;
    }
    _netVec.clear();

}

void Floorplanner::readBlock(std::fstream& input_blk)
{
    string name, str;
    size_t i1, i2;
    bool isTerminal;

    // input file format:
    // Outline: 1326 1205
    // NumBlocks: 33                                                                   
    // NumTerminals: 40
    // <macro_name> <macro width> <macro height>
    // ....
    // <terminal_name> terminal <terminal x coordinate> <terminal y coordinate>
    // ....

    input_blk >> str; // Outline:
    input_blk >> i1 >> i2;
    _outlineWidth = i1;
    _outlineHeight = i2;
    input_blk >> str; // NumBlocks:
    input_blk >> _numBlocks;
    input_blk >> str; // NumTerminals:
    input_blk >> _numTerminals;

    _blockVec.reserve(_numBlocks + _numTerminals);

    for (int i=0; i<_numBlocks; i++) {
        input_blk >> name >> i1 >> i2;
        Block* block = new Block(name, i1, i2, false);
        _blockVec.push_back(block);
        _name2IntMap[name] = i;
    }
    for (int i=_numBlocks; i<_numTerminals+_numBlocks; i++) {
        input_blk >> name >> str >> i1 >> i2;
        Block* block = new Block(name, i1, i2, true);
        _blockVec.push_back(block);
        _name2IntMap[name] = i;
    }
    input_blk.close();
    return;
}

void Floorplanner::readNet(std::fstream& input_net)
{
    string str;
    size_t netDegreeNum = 0;

    // input file format:
    // NumNets: 121
    // NetDegree: 34
    // <NetName>
    // <NetName>
    // <NetName>
    // ....

    input_net >> str; // NumNets:
    input_net >> _numNets;
    _netVec.reserve(_numNets);
    
    for (int i=0; i<_numNets; i++) {

        _netVec.push_back(new Net());
        input_net >> str; // NetDegree:
        input_net >> netDegreeNum;
        for (int j=0; j<netDegreeNum; j++) {
            input_net >> str; // block name in the net
            int blockNum = _name2IntMap[str];
            _netVec[i]->addTerm(_blockVec[blockNum]);
        }
    }
    input_net.close();
    return;
}

void Floorplanner::floorplan()
{
    // initPlace();
    initPlace_2();
    
    initPerturbation();
    SA_basic();
    retrieveBestBSTree();
    while (!_bestCost.valid) {
        putBlockInDamnBox();
        retrieveBestBSTree();
    }
    finalTuning();
    retrieveBestBSTree();
    shiftTuning();

    calFinalCost();
    
}

// initial placement
// calculate the initial cost and assign to the bestcost
void Floorplanner::initPlace()
{
    sort(_blockVec.begin(), _blockVec.begin() + _numBlocks, [](Block* a, Block* b) {
        return a->getArea() > b->getArea();
    });

    // init place
    // build the B* tree using BFS-like technigue, and form a near balanced tree
    _bSTree = _blockVec[0];
    queue<Block*> Q;
    Q.push(_bSTree);

    for (int i=1; i<_numBlocks; i++) {
        Block* block = _blockVec[i];
        Block* parent = nullptr;
        int flag = (i-1) % 2; // 0: right, 1: top

        parent = Q.front();
        if (flag == TOP)
            Q.pop();
        
        if (flag == RIGHT) {
            parent->setRight(block);
            block->setParent(parent);
        }
        // flag == TOP
        else {
            parent->setTop(block);
            block->setParent(parent);
        }
        
        Q.push(block);

    }

    packing();
    
    for (int i=0; i<_numBlocks; i++) {
        _blockVec[i]->setBest();
    }
    calCost();
    _bestCost.setCost(_cost);
    _prevCost.setCost(_cost);
    _normArea = _cost.area;
    _normHPWL = _cost.hpwl;
}

// initial placement
// calculate the initial cost and assign to the bestcost
void Floorplanner::initPlace_2()
{
    sort(_blockVec.begin(), _blockVec.begin() + _numBlocks, [](Block* a, Block* b) {
        return a->getArea() > b->getArea();
    });

    // init place 2
    // put the block from left to right, once x coordinate exceed the outline width,
    // put the block in the next line
    size_t curr_x = 0;
    size_t curr_y = 0;
    _bSTree = _blockVec[0];
    _bSTree->setPos(curr_x, curr_y);
    _contour.clear();
    curr_y = _contour.getY(curr_x, _bSTree->getWidth(), _bSTree->getHeight());
    curr_x = curr_x + _bSTree->getWidth();
    curr_y = curr_y + _bSTree->getHeight();
    Block *block;
    Block *prevBlock = _bSTree;
    Block *currLeftestBlock = _bSTree;
    Block::setMaxX(_bSTree->getWidth());
    Block::setMaxY(_bSTree->getHeight());
    
    for (int i=1; i<_numBlocks; i++) {
        block = _blockVec[i];
        if (curr_x + block->getWidth() > _outlineWidth) {
            currLeftestBlock->setTop(block);
            block->setParent(currLeftestBlock);
            currLeftestBlock = block;
            curr_x = 0;
        }
        else {
            block->setParent(prevBlock);
            prevBlock->setRight(block);
        }
        curr_y = _contour.getY(curr_x, block->getWidth(), block->getHeight());
        block->setPos(curr_x, curr_y);
        if (curr_x + block->getWidth() > Block::getMaxX()) {
            Block::setMaxX(curr_x + block->getWidth());
        }
        if (curr_y + block->getHeight() > Block::getMaxY()) {
            Block::setMaxY(curr_y + block->getHeight());
        }
        curr_x = curr_x + block->getWidth();
        prevBlock = block;
    }


    // packing();
    
    for (int i=0; i<_numBlocks; i++) {
        _blockVec[i]->setBest();
    }
    calCost();
    _bestCost.setCost(_cost);
    _prevCost.setCost(_cost);
    _normArea = _cost.area;
    _normHPWL = _cost.hpwl;
}

void Floorplanner::packing()
{
    Block::setMaxX(0);
    Block::setMaxY(0);
    _contour.clear();
    // perform DFS search the B* tree
    // and build the contour while visiting each block
    // calculate the x coordinate of the left down corner of each block
    stack<Block*> S;
    Block* block = _bSTree;
    size_t curr_x = 0; // left down corner x coordinate
    while (!S.empty() || block != nullptr) {
        if (block == nullptr){
            // pop a block from the stack, and search for its top child
            block = S.top();
            S.pop();
            curr_x = block->getX();
            block = block->getTop();
            continue;
        }

        // update the contour
        size_t y = _contour.getY(curr_x, block->getWidth(), block->getHeight());
        
        // state the block position
        block->setPos(curr_x, y);
        if (block->getX() + block->getWidth() > Block::getMaxX()) {
            Block::setMaxX(block->getX() + block->getWidth());
        }
        if (y + block->getHeight() > Block::getMaxY()) {
            Block::setMaxY(y + block->getHeight());
        }
        curr_x += block->getWidth();
        
        // push the block to the stack
        S.push(block);
        block = block->getRight();
        
    }
}

void Floorplanner::updateBestBSTree()
{
    _bestBSTreeTop.clear();
    // perform DFS search the B* tree
    stack<Block*> S;
    Block* block = _bSTree;
    vector<Block*> rightChain;
    // rightChain.push_back(nullptr);
    while (!S.empty() || block != nullptr) {
        if (block == nullptr){
            // pop a block from the stack, and search for its top child
            _bestBSTreeTop.push_back(rightChain);
            rightChain.clear();
            block = S.top();
            S.pop();
            rightChain.push_back(block); // the block's top deirection connect to the next right chain
            block = block->getTop();
            continue;
        }
        rightChain.push_back(block);
        // push the block to the stack
        S.push(block);
        block = block->getRight();
        
    }
    if (!rightChain.empty()) {
        _bestBSTreeTop.push_back(rightChain);
        rightChain.clear();
    }
    return;
}

void Floorplanner::retrieveBestBSTree() 
{
    for (int i=0; i<_bestBSTreeTop.size(); i++) {
        Block *parentBlock = _bestBSTreeTop[i][0];
        if (i == 0) {
            _bSTree = parentBlock;
            parentBlock->setParent(nullptr);
            parentBlock->setRight(nullptr);
        }
        parentBlock->setTop(nullptr);
        for (int j=1; j<_bestBSTreeTop[i].size(); j++) {
            Block* block = _bestBSTreeTop[i][j];
            if (j == 1 && i != 0) {
                parentBlock->setTop(block);
            }
            else {
                parentBlock->setRight(block);
            }
            block->setParent(parentBlock);
            block->setRight(nullptr);
            block->setTop(nullptr);
            parentBlock = block;
        }
    }
    for (int i=0; i<_numBlocks; i++) {
        _blockVec[i]->setPos(_blockVec[i]->getBestCoordinate()[0], _blockVec[i]->getBestCoordinate()[1]);
        _blockVec[i]->setWidth(_blockVec[i]->getBestCoordinate()[2] - _blockVec[i]->getBestCoordinate()[0]);
        _blockVec[i]->setHeight(_blockVec[i]->getBestCoordinate()[3] - _blockVec[i]->getBestCoordinate()[1]);
    }
    return;
}

void Floorplanner::initPerturbation()
{
    _name2IntMap.clear();
    for (int i=0; i<_blockVec.size(); i++) {
        _name2IntMap[_blockVec[i]->getName()] = i;
    }
    // for delete and inserttion
    vector<InsertInfo> insertInfoVec, dumb1;
    vector<Block*> parentBlockVec, dumb2;

    vector<size_t> recentArea; // for calculate the normalized area
    vector<double> recentHPWL; // for calculate the normalized HPWL
    vector<double> recentAspectRatio; // for calculate the normalized aspect ratio
    vector<double> recentExceedArea; // for calculate the normalized exceed area
    // size_t initnum = min(size_t(100), _numBlocks); // pass
    size_t initnum = min(size_t(20), _numBlocks);
    
    for (int i=0; i<initnum; i++){
        Block * block1 = _blockVec[rand() % _numBlocks];
        Block * block2 = _blockVec[rand() % _numBlocks];
        while (block1 == block2) {
            block2 = _blockVec[rand() % _numBlocks];
        }

        int op = rand() % 3;
        int pos = rand() % 2;
        string posstr = (pos == RIGHT) ? "right" : "top";


        insertInfoVec.clear();
        parentBlockVec.clear();
        dumb1.clear();
        dumb2.clear();

        if (op == 0){
            block1->rotate();
        }
        else if (op == 1) {
            deleteBlock(block1, insertInfoVec, parentBlockVec);
            insertBlock(block1, block2, pos);
        }
        else {
            swapTwoBlocks(block1, block2);
        }
        packing();
        calCost();
        recentArea.push_back(_cost.area);
        recentHPWL.push_back(_cost.hpwl);
        recentAspectRatio.push_back(calAspectRatioTerm(_cost.height, _cost.width));
        recentExceedArea.push_back(calExceedArea(_cost.height, _cost.width));
        
        _prevCost.setCost(_cost);
        if (_cost.valid) {
            for (int i=0; i<_numBlocks; i++) {
                _blockVec[i]->setBest();
            }
            _bestCost.setCost(_cost);
        }
        // if (_cost < _bestCost) {
        //     for (int i=0; i<_numBlocks; i++) {
        //         _blockVec[i]->setBest();
        //     }
        // }
    }
    
    _normArea = accumulate(recentArea.begin(), recentArea.end(), 0) / (double)recentArea.size();
    _normHPWL = accumulate(recentHPWL.begin(), recentHPWL.end(), 0) / (double)recentHPWL.size();
    _normAspectRatio = accumulate(recentAspectRatio.begin(), recentAspectRatio.end(), 0) / (double)recentAspectRatio.size();
    _normExceedArea = accumulate(recentExceedArea.begin(), recentExceedArea.end(), 0) / (double)recentExceedArea.size();
    updateBestCost();
    updatePrevCost();
}

void Floorplanner::SA_basic()
{
    // set the initial temperature
    _temperature = 500.0;
    _coolTemp = 0.001;
    _coolRate = 0.98;

    // set the number of iterations
    _numIter = 0;
    _numIterPerTemp = 1000;
    
    // set the alpha
    _alpha = _givenAlpha;
    _beta = 1 - _givenAlpha;
    _gamma = 0.1;
    bool is_valid = false;
    int continueValid = 0;
    int continueNotValid = 0;

    vector<size_t> recentArea; // for calculate the normalized area
    vector<double> recentHPWL; // for calculate the normalized HPWL
    vector<double> recentAspectRatio; // for calculate the normalized aspect ratio
    vector<size_t> recentExceedArea; // for calculate the normalized exceed area
    // size_t preserveNum = min(size_t(100), _numBlocks); // pass
    size_t preserveNum = min(size_t(20), _numBlocks);
    recentArea.reserve(preserveNum);
    recentHPWL.reserve(preserveNum);
    recentAspectRatio.reserve(preserveNum);
    recentExceedArea.reserve(preserveNum);

    size_t perturbationNum = 0;
    size_t noImproveNum = 0;

    // for delete and insert operation
    vector<InsertInfo> insertInfoVec, dumbVec1;
    vector<Block*> parentBlockVec, dumbVec2;

    int calMaxIter = ceil(log(_coolTemp / _temperature) / log(_coolRate));
    // int calMaxIter = ceil(log(_coolTemp / _temperature) / (_coolRate));
    int twoThirdIter = calMaxIter * 2 / 3;
    int validNum = 0; // in the same termperature
    // double gammaUpperBound = 2.0;
    double gammaUpperBound = 1.0;
    double gammaLowerBound = 0.0;
    double exponent = log(gammaUpperBound / _gamma) / (double)calMaxIter;

    
    // start the SA
    while (_temperature > _coolTemp) {
        updateGamma(true, exponent, 1.0, gammaUpperBound, gammaLowerBound);
        updateBestCost();
        updatePrevCost();
        
        // perturbation in the same temperature
        for (int j=0; j<_numIterPerTemp; j++) {
            validNum = 0;
            int op = rand() % 3;
            // int op = 2;
            
            // for delete and insert
            insertInfoVec.clear();
            parentBlockVec.clear();
            dumbVec1.clear();
            dumbVec2.clear();
            int newPos = rand() % 2;
            
            Block * block1 = _blockVec[rand() % _numBlocks];
            Block * block2 = _blockVec[rand() % _numBlocks];
            while (block1 == block2) {
                block2 = _blockVec[rand() % _numBlocks];
            }
            // perturbation operation
            // 0: rotate, 1: delete and insert, 2: swap
            if (op == 0) {
                // cout << "rotate..." << endl;
                block1->rotate();
                // cout << "finish rotate" << endl;
            }
            else if (op == 1) {
                // cout << "delete and insert..." << endl;
                deleteBlock(block1, insertInfoVec, parentBlockVec);
                insertBlock(block1, block2, newPos);
            }
            else {
                // cout << "swap " << block1->getName() << " and " << block2->getName() << "..." << endl;
                swapTwoBlocks(block1, block2);
                // cout << "finish swap" << endl;
            }
            
            // calculate the cost
            packing();
            calCost();
            if (_cost.valid) {
                continueValid++;
                continueNotValid = 0;
                is_valid = true;
                validNum++;
            }
            else {
                continueValid = 0;
                continueNotValid++;
            }

            
            // calculate the normalized area and HPWL
            recentHPWL.push_back(_cost.hpwl);
            recentArea.push_back(_cost.area);
            recentAspectRatio.push_back(calAspectRatioTerm(_cost.height, _cost.width));
            recentExceedArea.push_back(calExceedArea(_cost.height, _cost.width));
            
            
            
            // if the cost is better than the previous cost, update the previous cost
            if (_cost < _prevCost) {
                noImproveNum = 0;
                _prevCost.setCost(_cost);
                // if the cost is better than the best cost, update the best cost
                if (_cost < _bestCost) {
                    _bestCost.setCost(_cost);
                    for (int k=0; k<_numBlocks; k++) {
                        _blockVec[k]->setBest();
                    }
                    updateBestBSTree();
                }
            }
            // if the cost is worse than the previous cost, accept the new cost with a probability
            else {
                noImproveNum++;
                double prob = exp((_prevCost.cost - _cost.cost) / _temperature);
                double randNum = (double)rand() / (double)RAND_MAX;
                if (randNum < prob) {
                    _prevCost.setCost(_cost);
                }
                else {
                    // undo the perturbation
                    switch (op) {
                    case 0:
                        block1->rotate();
                        break;
                    case 1:
                        // undo the delete and insert
                        deleteBlock(block1, dumbVec1, dumbVec2);
                        reInsertBlock(block1, insertInfoVec, parentBlockVec);
                        break;
                    case 2:
                        // undo the swap
                        swapTwoBlocks(block1, block2);
                        break;
                    } // end of switch
                } // end of undo the perturbation
            } // end of the cost is worse than the previous cost
            
            // update normalized area and HPWL
            if (recentArea.size() == preserveNum){
                _normArea = accumulate(recentArea.begin(), recentArea.end(), 0) / (double)recentArea.size();
                _normHPWL = accumulate(recentHPWL.begin(), recentHPWL.end(), 0) / (double)recentHPWL.size();
                _normAspectRatio = accumulate(recentAspectRatio.begin(), recentAspectRatio.end(), 0) / (double)recentAspectRatio.size();
                _normExceedArea = accumulate(recentExceedArea.begin(), recentExceedArea.end(), 0) / (double)recentExceedArea.size();
                if (_normAspectRatio <= 1e-6) _normAspectRatio = 1.0;
                if (_normExceedArea <= 1) _normExceedArea = 1;
                if (_normArea <= 0.0) {
                    cerr << "Error: normalized area <= 0" << endl;
                    cerr << "normArea: " << _normArea << endl;
                    return;
                }
                if (_normHPWL <= 0.0) {
                    cerr << "Error: normalized HPWL <= 0" << endl;
                    cerr << "normHPWL: " << _normHPWL << endl;
                    return;
                }
                if (_normAspectRatio <= 0.0) {
                    cerr << "Error: normalized aspect ratio <= 0" << endl;
                    cerr << "normAspectRatio: " << _normAspectRatio << endl;
                    return;
                }
                if (_normExceedArea <= 0) {
                    cerr << "Error: normalized exceed area <= 0" << endl;
                    cerr << "normExceedArea: " << _normExceedArea << endl;
                    return;
                }
                recentArea.clear();
                recentHPWL.clear();
                recentAspectRatio.clear();
                recentExceedArea.clear();
                
                // update _bestCost using new normalized area and HPWL
                updateBestCost();
                updatePrevCost();
            }
            
            if (continueValid > _numBlocks * 1) {
                _gamma = max(0.0, _gamma - 0.1);
                updateBestCost();
                updatePrevCost();
            }
            if (continueNotValid > _numBlocks * 1 && _numIter >= twoThirdIter) {
                _gamma += 0.1;
                updateBestCost();
                updatePrevCost();
            }
            
            perturbationNum++;
            // if (noImproveNum > 100) {
            //     cout << "no improvement for 100 iterations, break" << endl;
            //     break;
            // }
        } // end of the same temperature loop
        // if (noImproveNum > 100) {
        //     cout << "no improvement for 100 iterations, break" << endl;
        //     break;
        // }

        // set the temperature
        _temperature *= _coolRate;
        _numIter++;
    } // end of SA
}

void Floorplanner::putBlockInDamnBox()
{
    // set the initial temperature
    _temperature = 1.0;
    _coolTemp = 0.0001;
    _coolRate = 0.95;

    // set the number of iterations
    _numIter = 0;
    _numIterPerTemp = 1000;
    
    // set the alpha
    _alpha = 0.5;
    bool is_valid = false;
    int continueValid = 0;

    vector<size_t> recentArea; // for calculate the normalized area
    vector<double> recentHPWL; // for calculate the normalized HPWL
    vector<double> recentAspectRatio; // for calculate the normalized aspect ratio
    vector<size_t> recentExceedArea; // for calculate the normalized exceed area
    // size_t preserveNum = min(size_t(100), _numBlocks); // pass
    size_t preserveNum = min(size_t(20), _numBlocks);
    recentArea.reserve(preserveNum);
    recentHPWL.reserve(preserveNum);
    recentAspectRatio.reserve(preserveNum);
    recentExceedArea.reserve(preserveNum);

    size_t perturbationNum = 0;
    size_t noImproveNum = 0;
    // perturbationNum = 0;

    // for delete and insert operation
    vector<InsertInfo> insertInfoVec, dumbVec1;
    vector<Block*> parentBlockVec, dumbVec2;

    int calMaxIter = ceil(log(_coolTemp / _temperature) / log(_coolRate));
    int twoThirdIter = calMaxIter * 2 / 3;
    
    _prevCost.cost = 1e10;
    _bestCost.cost = 1e10;
    // start the SA
    while (_temperature > _coolTemp) {
        
        // perturbation in the same temperature
        for (int j=0; j<_numIterPerTemp; j++) {
            
            
            int op = rand() % 3;
            // int op = 2;
            
            // for delete and insert
            insertInfoVec.clear();
            parentBlockVec.clear();
            dumbVec1.clear();
            dumbVec2.clear();
            int newPos = rand() % 2;
            
            Block * block1 = _blockVec[rand() % _numBlocks];
            Block * block2 = _blockVec[rand() % _numBlocks];
            while (block1 == block2) {
                block2 = _blockVec[rand() % _numBlocks];
            }
            // perturbation operation
            // 0: rotate, 1: delete and insert, 2: swap
            if (op == 0) {
                // cout << "rotate..." << endl;
                block1->rotate();
                // cout << "finish rotate" << endl;
            }
            else if (op == 1) {
                // cout << "delete and insert..." << endl;
                deleteBlock(block1, insertInfoVec, parentBlockVec);
                insertBlock(block1, block2, newPos);
            }
            else {
                // cout << "swap " << block1->getName() << " and " << block2->getName() << "..." << endl;
                swapTwoBlocks(block1, block2);
                // cout << "finish swap" << endl;
            }
            
            packing();
            // calculate the cost
            // cost function1 : _alpha * exceedArea + (1-Alpha) * AspectRatio
            // cost function2 : exceedArea
            calCost();
            // _cost.cost = calExceedArea(_cost.height, _cost.width) * _alpha + calAspectRatioTerm(_cost.height, _cost.width) * (1 - _alpha);
            // _cost.cost = calExceedArea(_cost.height, _cost.width) ;
            _cost.cost = (double)calExceedArea(_cost.height, _cost.width) / _normExceedArea * _alpha + 
                (double)_cost.area / _normArea * (1 - _alpha);
            // end calculate the cost

            if (_cost.valid) {
                continueValid++;
                is_valid = true;
            }
            else {
                continueValid = 0;
            }

            
            // calculate the normalized area and HPWL
            recentHPWL.push_back(_cost.hpwl);
            recentArea.push_back(_cost.area);
            recentAspectRatio.push_back(calAspectRatioTerm(_cost.height, _cost.width));
            recentExceedArea.push_back(calExceedArea(_cost.height, _cost.width));
            
            
            
            // if the cost is better than the previous cost, update the previous cost
            if (_cost < _prevCost) {
                noImproveNum = 0;
                _prevCost.setCost(_cost);
                // if the cost is better than the best cost, update the best cost
                if (_cost < _bestCost) {
                    _bestCost.setCost(_cost);
                    for (int k=0; k<_numBlocks; k++) {
                        _blockVec[k]->setBest();
                    }
                    updateBestBSTree();
                }
            }
            // if the cost is worse than the previous cost, accept the new cost with a probability
            else {
                noImproveNum++;
                double prob = exp((_prevCost.cost - _cost.cost) / _temperature);
                double randNum = (double)rand() / (double)RAND_MAX;
                if (randNum < prob) {
                    _prevCost.setCost(_cost);
                }
                else {
                    // _cost.setCost(_prevCost);
                    // undo the perturbation
                    switch (op) {
                    case 0:
                        block1->rotate();
                        break;
                    case 1:
                        // undo the delete and insert
                        deleteBlock(block1, dumbVec1, dumbVec2);
                        reInsertBlock(block1, insertInfoVec, parentBlockVec);
                        break;
                    case 2:
                        // undo the swap
                        swapTwoBlocks(block1, block2);
                        break;
                    } // end of switch
                } // end of undo the perturbation
            } // end of the cost is worse than the previous cost
            
            // update normalized area and HPWL
            if (recentArea.size() == preserveNum){
                _normArea = accumulate(recentArea.begin(), recentArea.end(), 0) / (double)recentArea.size();
                _normHPWL = accumulate(recentHPWL.begin(), recentHPWL.end(), 0) / (double)recentHPWL.size();
                _normAspectRatio = accumulate(recentAspectRatio.begin(), recentAspectRatio.end(), 0) / (double)recentAspectRatio.size();
                _normExceedArea = accumulate(recentExceedArea.begin(), recentExceedArea.end(), 0) / (double)recentExceedArea.size();
                if (_normAspectRatio <= 1e-6) _normAspectRatio = 1.0;
                if (_normExceedArea <= 1) _normExceedArea = 1;
                if (_normArea <= 0.0) {
                    cerr << "Error: normalized area <= 0" << endl;
                    cerr << "normArea: " << _normArea << endl;
                    return;
                }
                if (_normHPWL <= 0.0) {
                    cerr << "Error: normalized HPWL <= 0" << endl;
                    cerr << "normHPWL: " << _normHPWL << endl;
                    return;
                }
                if (_normAspectRatio <= 0.0) {
                    cerr << "Error: normalized aspect ratio <= 0" << endl;
                    cerr << "normAspectRatio: " << _normAspectRatio << endl;
                    return;
                }
                if (_normExceedArea <= 0) {
                    cerr << "Error: normalized exceed area <= 0" << endl;
                    cerr << "normExceedArea: " << _normExceedArea << endl;
                    return;
                }
                recentArea.clear();
                recentHPWL.clear();
                recentAspectRatio.clear();
                recentExceedArea.clear();
            }
            // update _bestCost using new normalized area and HPWL
            // updateBestCost();
            // updatePrevCost();
            // _prevCost.cost = _alpha * calExceedArea(_prevCost.height, _prevCost.width) + 
            //     (1 - _alpha) * calAspectRatioTerm(_prevCost.height, _prevCost.width);
            // _bestCost.cost = _alpha * calExceedArea(_bestCost.height, _bestCost.width) + 
            //     (1 - _alpha) * calAspectRatioTerm(_bestCost.height, _bestCost.width);
            _prevCost.cost = _alpha * calExceedArea(_prevCost.height, _prevCost.width) / (double)_normExceedArea + 
                (1 - _alpha) * _prevCost.area / (double)_normArea;
            _bestCost.cost = _alpha * calExceedArea(_bestCost.height, _bestCost.width) / (double)_normExceedArea + 
                (1 - _alpha) * _bestCost.area / (double)_normArea;
            
            perturbationNum++;
        } // end of the same temperature loop

        // set the temperature
        _temperature *= _coolRate;
        _numIter++;
    } // end of SA
}


void Floorplanner::finalTuning()
{
    // set the initial temperature
    _temperature = 1.0;
    _coolTemp = 0.001;
    _coolRate = 0.95;

    // set the number of iterations
    _numIter = 0;
    _numIterPerTemp = 1000;
    
    // set the alpha
    _alpha = _givenAlpha;
    // _beta = 0.1;
    bool is_valid = false;
    int continueValid = 0;

    vector<size_t> recentArea; // for calculate the normalized area
    vector<double> recentHPWL; // for calculate the normalized HPWL
    vector<double> recentAspectRatio; // for calculate the normalized aspect ratio
    vector<size_t> recentExceedArea; // for calculate the normalized exceed area
    // size_t preserveNum = min(size_t(100), _numBlocks); // pass
    size_t preserveNum = min(size_t(20), _numBlocks);
    recentArea.reserve(preserveNum);
    recentHPWL.reserve(preserveNum);
    recentAspectRatio.reserve(preserveNum);
    recentExceedArea.reserve(preserveNum);

    size_t perturbationNum = 0;
    size_t noImproveNum = 0;
    // perturbationNum = 0;

    // for delete and insert operation
    vector<InsertInfo> insertInfoVec, dumbVec1;
    vector<Block*> parentBlockVec, dumbVec2;

    int calMaxIter = ceil(log(_coolTemp / _temperature) / log(_coolRate));
    int twoThirdIter = calMaxIter * 2 / 3;
    
    // _prevCost.cost = 1e10;
    // _bestCost.cost = 1e10;
    _prevCost.cost = _alpha * _prevCost.area / (double)_normArea + 
        (1 - _alpha) * _prevCost.hpwl / (double)_normHPWL;
    _bestCost.cost = _alpha * _bestCost.area / (double)_normArea + 
        (1 - _alpha) * _bestCost.hpwl / (double)_normHPWL;
    // start the SA
    while (_temperature > _coolTemp) {
        
        // perturbation in the same temperature
        for (int j=0; j<_numIterPerTemp; j++) {
            
            
            int op = rand() % 3;
            // int op = 2;
            
            // for delete and insert
            insertInfoVec.clear();
            parentBlockVec.clear();
            dumbVec1.clear();
            dumbVec2.clear();
            int newPos = rand() % 2;
            
            Block * block1 = _blockVec[rand() % _numBlocks];
            Block * block2 = _blockVec[rand() % _numBlocks];
            while (block1 == block2) {
                block2 = _blockVec[rand() % _numBlocks];
            }
            // perturbation operation
            // 0: rotate, 1: delete and insert, 2: swap
            if (op == 0) {
                block1->rotate();
            }
            else if (op == 1) {
                deleteBlock(block1, insertInfoVec, parentBlockVec);
                insertBlock(block1, block2, newPos);
            }
            else {
                swapTwoBlocks(block1, block2);
            }
            
            packing();
            // calculate the cost
            // cost function1 : _alpha * exceedArea + (1-Alpha) * AspectRatio
            // cost function2 : exceedArea
            calCost();
            // _cost.cost = calExceedArea(_cost.height, _cost.width) * _alpha + calAspectRatioTerm(_cost.height, _cost.width) * (1 - _alpha);
            // _cost.cost = calExceedArea(_cost.height, _cost.width) ;
            _cost.cost = (double)_cost.area / _normArea * _alpha + 
                (double)_cost.hpwl / _normHPWL * (1 - _alpha);
            // end calculate the cost

            if (_cost.valid) {
                continueValid++;
                is_valid = true;
            }
            else {
                continueValid = 0;
            }

            
            // calculate the normalized area and HPWL
            recentHPWL.push_back(_cost.hpwl);
            recentArea.push_back(_cost.area);
            recentAspectRatio.push_back(calAspectRatioTerm(_cost.height, _cost.width));
            recentExceedArea.push_back(calExceedArea(_cost.height, _cost.width));
            
            
            
            // if the cost is better than the previous cost, update the previous cost
            if (_cost < _prevCost) {
                noImproveNum = 0;
                _prevCost.setCost(_cost);
                // if the cost is better than the best cost, update the best cost
                if (_cost < _bestCost) {
                    _bestCost.setCost(_cost);
                    for (int k=0; k<_numBlocks; k++) {
                        _blockVec[k]->setBest();
                    }
                    updateBestBSTree();
                }
            }
            // if the cost is worse than the previous cost, accept the new cost with a probability
            else {
                noImproveNum++;
                double prob = exp((_prevCost.cost - _cost.cost) / _temperature);
                double randNum = (double)rand() / (double)RAND_MAX;
                if (randNum < prob) {
                    _prevCost.setCost(_cost);
                }
                else {
                    // _cost.setCost(_prevCost);
                    // undo the perturbation
                    switch (op) {
                    case 0:
                        block1->rotate();
                        break;
                    case 1:
                        // undo the delete and insert
                        /*TODO*/
                        deleteBlock(block1, dumbVec1, dumbVec2);
                        reInsertBlock(block1, insertInfoVec, parentBlockVec);
                        break;
                    case 2:
                        // undo the swap
                        swapTwoBlocks(block1, block2);
                        break;
                    } // end of switch
                } // end of undo the perturbation
            } // end of the cost is worse than the previous cost
            
            // update normalized area and HPWL
            if (recentArea.size() == preserveNum){
                _normArea = accumulate(recentArea.begin(), recentArea.end(), 0) / (double)recentArea.size();
                _normHPWL = accumulate(recentHPWL.begin(), recentHPWL.end(), 0) / (double)recentHPWL.size();
                _normAspectRatio = accumulate(recentAspectRatio.begin(), recentAspectRatio.end(), 0) / (double)recentAspectRatio.size();
                _normExceedArea = accumulate(recentExceedArea.begin(), recentExceedArea.end(), 0) / (double)recentExceedArea.size();
                if (_normAspectRatio <= 1e-6) _normAspectRatio = 1.0;
                if (_normExceedArea <= 1) _normExceedArea = 1;
                if (_normArea <= 0.0) {
                    cerr << "Error: normalized area <= 0" << endl;
                    cerr << "normArea: " << _normArea << endl;
                    return;
                }
                if (_normHPWL <= 0.0) {
                    cerr << "Error: normalized HPWL <= 0" << endl;
                    cerr << "normHPWL: " << _normHPWL << endl;
                    return;
                }
                if (_normAspectRatio <= 0.0) {
                    cerr << "Error: normalized aspect ratio <= 0" << endl;
                    cerr << "normAspectRatio: " << _normAspectRatio << endl;
                    return;
                }
                if (_normExceedArea <= 0) {
                    cerr << "Error: normalized exceed area <= 0" << endl;
                    cerr << "normExceedArea: " << _normExceedArea << endl;
                    return;
                }
                recentArea.clear();
                recentHPWL.clear();
                recentAspectRatio.clear();
                recentExceedArea.clear();
            }
            // update _bestCost using new normalized area and HPWL
            // updateBestCost();
            // updatePrevCost();
            // _prevCost.cost = _alpha * calExceedArea(_prevCost.height, _prevCost.width) + 
            //     (1 - _alpha) * calAspectRatioTerm(_prevCost.height, _prevCost.width);
            // _bestCost.cost = _alpha * calExceedArea(_bestCost.height, _bestCost.width) + 
            //     (1 - _alpha) * calAspectRatioTerm(_bestCost.height, _bestCost.width);
            _prevCost.cost = _alpha * _prevCost.area / (double)_normArea + 
                (1 - _alpha) * _prevCost.hpwl / (double)_normHPWL;
            _bestCost.cost = _alpha * _bestCost.area / (double)_normArea + 
                (1 - _alpha) * _bestCost.hpwl / (double)_normHPWL;
            
            perturbationNum++;
        } // end of the same temperature loop

        // set the temperature
        _temperature *= _coolRate;
        _numIter++;
    } // end of SA
}

void Floorplanner::shiftTuning()
{
    _cost.setCost(_bestCost);
    size_t minX = 0;
    size_t minY = 0;
    size_t maxX = _bestCost.width;
    size_t maxY = _bestCost.height;
    double hpwl = 0.0;
    
    size_t splitNumX = 100;
    size_t splitNumY = 100;
    
    size_t deltaX = max((_outlineWidth - maxX) / splitNumX, size_t(1));
    size_t deltaY = max((_outlineHeight - maxY) / splitNumY, size_t(1));

    splitNumX = min(splitNumX, (_outlineWidth - maxX) / deltaX);
    splitNumY = min(splitNumY, (_outlineHeight - maxY) / deltaY);

    for (size_t i=0; i<splitNumX; i++) {
        for (size_t j=0; j<splitNumY; j++) {
            Block::setMaxX(0);
            Block::setMaxY(0);
            if (i == 0 && j == 0) {
                continue;
            }
            hpwl = 0.0;
            for (auto block : _blockVec) {
                if (block->isTerminal())
                    break;
                if (j == 0)
                    block->shift(deltaX*i, 0);
                else
                    block->shift(0, deltaY*j);
                if (block->getX() + block->getWidth() > Block::getMaxX())
                    Block::setMaxX(block->getX() + block->getWidth());
                if (block->getY() + block->getHeight() > Block::getMaxY())
                    Block::setMaxY(block->getY() + block->getHeight());
            }
            if (Block::getMaxX() > _outlineWidth) return;
            if (Block::getMaxY() > _outlineHeight) break;
            for (auto net : _netVec) {
                hpwl += net->calcHPWL();
            }
            if (hpwl < _bestCost.hpwl) {
                _bestCost.hpwl = hpwl;
                for (auto block : _blockVec) {
                    if (block->isTerminal())
                        break;
                    block->setBest();
                }
            }
        }
        // move back to left bottom corner
        packing();
    }
}

void Floorplanner::updateGamma(bool increase_anyway, double exponent, double increase_ratio, double &upper_bound, double &lower_bound)
{
    if (increase_anyway) {
        _gamma *= exp(exponent);
    }
    // _gamma *= exp(increase_ratio); // higher score
    // _gamma *= exp(increase_ratio * exponent); // lower score
    if (_gamma > upper_bound) {
        _gamma = upper_bound;
    }
    if (_gamma < lower_bound) {
        _gamma = lower_bound;
    }
    return;
}

// update the current cost, _cost
void Floorplanner::calCost()
{
    double aspectRationTerm = calAspectRatioTerm(Block::getMaxY(), Block::getMaxX());
    double area = Block::getMaxX() * Block::getMaxY();
    double cost = 0.0;
    double hpwl = 0.0;
    bool valid = (Block::getMaxX() <= _outlineWidth) && (Block::getMaxY() <= _outlineHeight);
    for (auto net : _netVec) {
        hpwl += net->calcHPWL();
    }
    size_t exceedArea = calExceedArea(Block::getMaxY(), Block::getMaxX());

    // cost = _alpha * (area / _normArea) + (1 - _alpha) * (hpwl / _normHPWL);
    // cost = _alpha * (area / _normArea) + _beta * (hpwl / _normHPWL) + (1-_alpha-_beta) * (aspectRationTerm / _normAspectRatio);
    // cost = _alpha * (area / _normArea) + _beta * (hpwl / _normHPWL) + (1-_alpha-_beta) * ((double)exceedArea / _normExceedArea);
    cost = _alpha * ((double)area / _normArea) + _beta * ((double)hpwl / _normHPWL) + _gamma * ((double)exceedArea / _normExceedArea);
    _cost.setCost(cost, hpwl, area, Block::getMaxX(), Block::getMaxY(), valid);
    return;
}

void Floorplanner::updateBestCost()
{
    // cost 1
    // _bestCost.cost = _alpha * (_bestCost.area / _normArea) + (1 - _alpha) * (_bestCost.hpwl / _normHPWL);

    // cost 2
    // double aspectRationTerm = calAspectRatioTerm(_bestCost.height, _bestCost.width);
    // _bestCost.cost = _alpha * (_bestCost.area / _normArea) + _beta * (_bestCost.hpwl / _normHPWL)
    //     + (1 - _alpha - _beta) * (aspectRationTerm / _normAspectRatio);

    // cost 3
    // size_t exceedArea = calExceedArea(_bestCost.height, _bestCost.width);
    // _bestCost.cost = _alpha * (_bestCost.area / _normArea) + _beta * (_bestCost.hpwl / _normHPWL)
    //     + (1 - _alpha - _beta) * ((double)exceedArea / _normExceedArea);
    
    // cost 4
    size_t exceedArea = calExceedArea(_bestCost.height, _bestCost.width);
    _bestCost.cost = _alpha * ((double)_bestCost.area / _normArea) + _beta * (_bestCost.hpwl / _normHPWL)
        + _gamma * ((double)exceedArea / _normExceedArea);
    return;
}

void Floorplanner::updatePrevCost()
{
    // cost 1
    // _prevCost.cost = _alpha * (_prevCost.area / _normArea) + (1 - _alpha) * (_prevCost.hpwl / _normHPWL);

    // cost 2
    // double aspectRationTerm = calAspectRatioTerm(_prevCost.height, _prevCost.width);
    // _prevCost.cost = _alpha * (_prevCost.area / _normArea) + _beta * (_prevCost.hpwl / _normHPWL) + 
    //     (1 - _alpha - _beta) * (aspectRationTerm / _normAspectRatio);

    // cost 3
    // size_t exceedArea = calExceedArea(_prevCost.height, _prevCost.width);
    // _prevCost.cost = _alpha * (_prevCost.area / _normArea) + _beta * (_prevCost.hpwl / _normHPWL) + 
    //     (1 - _alpha - _beta) * ((double)exceedArea / _normExceedArea);
    
    // cost 4
    size_t exceedArea = calExceedArea(_prevCost.height, _prevCost.width);
    _prevCost.cost = _alpha * ((double)_prevCost.area / _normArea) + _beta * (_prevCost.hpwl / _normHPWL) + 
        _gamma * ((double)exceedArea / _normExceedArea);
    return;
}

size_t Floorplanner::calExceedArea(size_t height, size_t width)
{
    size_t exceedArea = 0;
    if ((width > _outlineWidth) && (height > _outlineHeight)) {
        exceedArea = width * height - _outlineWidth * _outlineHeight;
    }
    else if (width > _outlineWidth) {
        exceedArea = (width - _outlineWidth) * height;
    }
    else if (height > _outlineHeight) {
        exceedArea = (height - _outlineHeight) * width;
    }
    return exceedArea;
}

double Floorplanner::calAspectRatioTerm(size_t height, size_t width)
{
    double outlineRatio =  (double)_outlineHeight / (double)_outlineWidth;
    double myRatio = (double)height / (double)width;
    double aspectRationTerm = (myRatio - outlineRatio) * (myRatio - outlineRatio);
    return aspectRationTerm;
}

void Floorplanner::calFinalCost()
{
    // calculate the final cost
    _cost.setCost(_bestCost);
    _cost.cost = _givenAlpha * (double)(_cost.area) + (1.0 - _givenAlpha) * (double)(_cost.hpwl);
    return;
}

void Floorplanner::debugPrint()
{
    cout << "debug print" << endl;
    cout << "block info" << endl;
    for (auto block : _blockVec) {
        cout << block->getName() << " " << block->getWidth() << " " << block->getHeight() << endl;
    }
    cout << endl << endl;
    cout << "net info" << endl;
    for (auto net : _netVec) {
        cout << "net size: " << net->getSize() << endl;
        for (auto block : net->getTermList()) {
            cout << block->getName() << " ";
        }
        cout << endl;
    }
    cout << endl << endl;
}

void Floorplanner::swapTwoBlocks(Block* block1, Block* block2)
{
    if (block1 == block2) {
        return;
    }
    if (block1 == nullptr || block2 == nullptr) {
        cerr << "Error: block1 or block2 is null" << endl;
        return;
    }
    
    // if one of the block is the parent of the other, make sure block2 is parent
    if (block2->getParent() == block1) {
        Block * tmp = block1;
        block1 = block2;
        block2 = tmp;
    }
    // swap the position in b*tree
    Block *parent1 = block1->getParent();
    Block *top1 = block1->getTop();
    Block *right1 = block1->getRight();
    
    Block *parent2 = block2->getParent();
    Block *top2 = block2->getTop();
    Block *right2 = block2->getRight();

    int block1Pos = (parent1 == nullptr) ? ROOT : (block1 == parent1->getTop()) ? TOP : RIGHT;
    int block2Pos = (parent2 == nullptr) ? ROOT : (block2 == parent2->getTop()) ? TOP : RIGHT;

    
    //  parent child relationship
    if (parent1 == block2) {
        
        // parent2 -> block1
        block1->setParent(parent2);
        if (block2Pos == TOP)  parent2->setTop(block1);
        else if (block2Pos == RIGHT) parent2->setRight(block1);
        else _bSTree = block1; // ROOT
        
        // block1 -> block2
        if (block1Pos == TOP) {
            block1->setTop(block2);
            block2->setParent(block1);
            // block -> children
            block1->setRight(right2);
            block2->setRight(right1);
            block2->setTop(top1);
            // children -> block
            if (right2 != nullptr) right2->setParent(block1);
            if (right1 != nullptr) right1->setParent(block2);
            if (top1 != nullptr) top1->setParent(block2);
        }
        else if (block1Pos == RIGHT) {
            block1->setRight(block2);
            block2->setParent(block1);
            // block to children
            block1->setTop(top2);
            block2->setTop(top1);
            block2->setRight(right1);
            // children to block
            if (top2 != nullptr) top2->setParent(block1);
            if (right1 != nullptr) right1->setParent(block2);
            if (top1 != nullptr) top1->setParent(block2);
        }
        else{
            cerr << "something wrong, in parent child relationship, child always has parent" << endl;
            return;
        }
    } // end of (parent1 == block2)
    else {
        // block to children
        block1->setTop(top2);
        block1->setRight(right2);
        block2->setTop(top1);
        block2->setRight(right1);
        // children to block
        if (top2 != nullptr) top2->setParent(block1);
        if (right2 != nullptr) right2->setParent(block1);
        if (top1 != nullptr) top1->setParent(block2);
        if (right1 != nullptr) right1->setParent(block2);
        // parent to block
        switch (block1Pos) {
            case ROOT:
            _bSTree = block2;
            break;
            case TOP:
                parent1->setTop(block2);
                break;
            case RIGHT:
                parent1->setRight(block2);
                break;
        }
        switch (block2Pos) {
            case ROOT:
                _bSTree = block1;
                break;
            case TOP:
                parent2->setTop(block1);
                break;
            case RIGHT:
                parent2->setRight(block1);
                break;
        }
        // block to parent
        block1->setParent(parent2);
        block2->setParent(parent1);
    }
    return;
}

// delete the block from the B* tree
// the input vectors need to be cleared before using
// three types
// 1. leaf node, delete the block
// 2. one child node, delete the block and insert the child
// 3. two child node, swap the block with one of its child node, and repeat until reach leaf or one child node
void Floorplanner::deleteBlock(Block* block, vector<InsertInfo>& insertInfoVec, vector<Block*>& parentBlockVec)
{
    if (block == nullptr) {
        cerr << "Error: block is null" << endl;
        return;
    }

    Block *parent = block->getParent();
    Block *top = block->getTop();
    Block *right = block->getRight();
    
    int type = (top != nullptr && right != nullptr) ? TWOCHILD : (top == nullptr && right == nullptr) ? LEAF : ONECHILD;
    int p2bpos; // parent to block position
    int b2cpos; // block to child position

    while (type == TWOCHILD) {
        // swap to one of its child node until reach LEAF or ONECHILD type
        int b2cpos = (rand() % 2 == 0) ? TOP : RIGHT;
        Block *child = (b2cpos == TOP) ? top : right;
        swapTwoBlocks(block, child);
        parentBlockVec.push_back(child);
        
        parent = block->getParent();
        top = block->getTop();
        right = block->getRight();
        type = (top != nullptr && right != nullptr) ? TWOCHILD : (top == nullptr && right == nullptr) ? LEAF : ONECHILD;
    }
    
    if (type == LEAF) {
        // leaf node
        // insertInfovec has one element
        p2bpos = (parent == nullptr) ? ROOT : (block == parent->getTop()) ? TOP : RIGHT;
        insertInfoVec.push_back(InsertInfo(parent, p2bpos));

        // delete the block
        block->setParent(nullptr);
        if (p2bpos == ROOT) {
            _bSTree = nullptr;
        }
        else if (p2bpos == TOP) {
            parent->setTop(nullptr);
        }
        else if (p2bpos == RIGHT) {
            parent->setRight(nullptr);
        }
    }
    else if (type == ONECHILD) {
        // one child node
        // insertInfoVec has two elements
        p2bpos = (parent == nullptr) ? ROOT : (block == parent->getTop()) ? TOP : RIGHT;
        b2cpos = (top != nullptr) ? TOP : RIGHT;
        Block *replacedBlock = (b2cpos == TOP) ? top : right;
        insertInfoVec.push_back(InsertInfo(parent, p2bpos));
        insertInfoVec.push_back(InsertInfo(replacedBlock, b2cpos));
        
        // delete the block
        block->setParent(nullptr);
        if (p2bpos == ROOT) {
            _bSTree = replacedBlock;
        }
        else if (p2bpos == TOP) {
            parent->setTop(replacedBlock);
        }
        else if (p2bpos == RIGHT) {
            parent->setRight(replacedBlock);
        }
        replacedBlock->setParent(parent);
        block->setTop(nullptr);
        block->setRight(nullptr);
    }
    else {
        cerr << "Error: block type is not LEAF or ONECHILD" << endl;
        return;
    }
    return;
}

// insert the block to the B* tree
// flag = 0: insert to the right of hostBlock
// flag = 1: insert to the top of hostBlock
void Floorplanner::insertBlock(Block* block, Block* hostBlock, int flag)
{
    if (block == nullptr || hostBlock == nullptr) {
        cerr << "Error: block or hostBlock is null" << endl;
        return;
    }
    
    Block *child = (flag == RIGHT) ? hostBlock->getRight() : hostBlock->getTop();
    int childpos = rand() % 2; // right or top of the insert block
    // note that the childpos won't affect the undo operation (delete and reinsert)
    // after insertion, the block will always be a leaf node or one child node
    
    // insert the block to the hostBlock
    if (flag == RIGHT) {
        block->setParent(hostBlock);
        hostBlock->setRight(block);
        if (childpos == RIGHT) block ->setRight(child);
        else block->setTop(child);
        if (child != nullptr) child->setParent(block);
    }
    else if (flag == TOP) {
        block->setParent(hostBlock);
        hostBlock->setTop(block);
        if (childpos == RIGHT) block ->setRight(child);
        else block->setTop(child);
        if (child != nullptr) child->setParent(block);
    }
    else {
        cerr << "Error: flag is not RIGHT or TOP" << endl;
        return;
    }
}

void Floorplanner::reInsertBlock(Block* block, vector<InsertInfo>& insertInfoVec, vector<Block*>& parentBlockVec)
{
    if (block == nullptr) {
        cerr << "Error: block is null" << endl;
        return;
    }
    
    if (insertInfoVec.empty()) {
        cerr << "Error: insertInfoVec is empty" << endl;
        return;
    }
    if (insertInfoVec.size() > 2) {
        cerr << "Error: insertInfoVec size is greater than 2" << endl;
        return;
    }

    // parent and target block connection
    Block *parent = insertInfoVec[0].block;
    int pos = insertInfoVec[0].pos;
    block->setParent(parent);
    if (pos == ROOT) {
        _bSTree = block;
    }
    else if (pos == TOP) {
        parent->setTop(block);
    }
    else if (pos == RIGHT) {
        parent->setRight(block);
    }
    
    // target block and child connection
    if (insertInfoVec.size() == ONECHILD) {
        Block *child = insertInfoVec[1].block;
        int childpos = insertInfoVec[1].pos;
        if (childpos == TOP) {
            block->setTop(child);
        }
        else if (childpos == RIGHT) {
            block->setRight(child);
        }
        else {
            cerr << "Error: childpos is not TOP or RIGHT" << endl;
            return;
        } 
        child->setParent(block);
    }
    
    // swap back the block to its original position
    // TWOCHILD case
    for (int i=parentBlockVec.size()-1; i>=0; i--) {
        if (parentBlockVec[i] == block) {
            cerr << "Error: In reInsertBlock, parentBlock is the same as block" << endl;
            return;
        }
        swapTwoBlocks(block, parentBlockVec[i]);
    }
    return;
    
}

void Floorplanner::printTree(ostream& outs)
{
    cout << "print tree" << endl;
    unordered_map<Block*, bool> blockVisited;
    stack<Block*> S;
    Block* block = _bSTree;
    outs << "root: ";
    while (!S.empty() || block != nullptr) {
        if (block == nullptr){
            // pop a block from the stack, and search for its top child
            outs << "- " << endl << S.top()->getName() << ": ";
            block = S.top();
            S.pop();
            block = block->getTop();
            continue;
        }
        if (blockVisited[block]) {
            outs << "tree acyclic, revisited " << block->getName() << endl;
            break;
        }
        outs << block->getName() << " ";
        // block visited
        blockVisited[block] = true;
        // push the block to the stack
        S.push(block);
        block = block->getRight();
        
    }
    outs << endl;
}

void Floorplanner::printBestTreeTopology()
{
    cout << "print best tree topology" << endl;
    cout << "root: ";
    for (int i=0; i<_bestBSTreeTop.size(); i++) {
        if (i != 0){
            cout << _bestBSTreeTop[i][0]->getName() << ": ";
        }
        else{
            cout << _bestBSTreeTop[i][0]->getName() << " ";
        }
        for (int j=1; j<_bestBSTreeTop[i].size(); j++) {
            cout << _bestBSTreeTop[i][j]->getName() << " ";
        }
        cout << endl;
    }
}


void printFloorplanResult(std::fstream& output, Floorplanner* fp, chrono::duration<double>& duration) {
    // <final cost>
    output << fp->_cost.cost << endl;
    // <total wirelength>
    output << fp->_cost.hpwl << endl;
    // <chip area>
    output << fp->_cost.area << endl;
    // <chip width> <chip height>
    output << fp->_cost.width << " " << fp->_cost.height << endl;
    // <program running time>
    output << duration.count() << endl;
    // <block name> <x1> <y1> <x2> <y2>
    for (auto block : fp->_blockVec) {
        if (block->isTerminal()) {
            break;
        }
        output << block->getName() << " "
                << block->getBestCoordinate()[0] << " "
                << block->getBestCoordinate()[1] << " "
                << block->getBestCoordinate()[2] << " "
                << block->getBestCoordinate()[3] << endl;
    }
    output.close();
}
