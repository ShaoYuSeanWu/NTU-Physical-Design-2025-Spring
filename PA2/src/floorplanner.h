#ifndef FLOORPLANNER_H
#define FLOORPLANNER_H

#include <vector>
#include <fstream>
#include <iostream>
#include <unordered_map>
#include <string>
#include <chrono>
#include "module.h"
#include "contour.h"
using namespace std;

// for B* tree
#define RIGHT 0
#define TOP  1
#define ROOT 2

// for operation delete and insert
#define TWOCHILD 0
#define LEAF 1
#define ONECHILD 2

// cost package
// cost = alpha * hpwl + (1 - alpha) * area
class Cost{
    public:
        double cost;
        double hpwl;
        size_t area;
        size_t width;
        size_t height;
        bool valid; // valid floorplan or not
    
        Cost() : cost(0), hpwl(0), area(0), width(0), height(0), valid(false) { }
        void setCost(double c, double h, size_t a, size_t w, size_t hgt, bool v) {
            cost = c;
            hpwl = h;
            area = a;
            width = w;
            height = hgt;
            valid = v;
        }
        void setCost(const Cost& other) {
            cost = other.cost;
            hpwl = other.hpwl;
            area = other.area;
            width = other.width;
            height = other.height;
            valid = other.valid;
        }
        bool operator<(const Cost& other) const {
            if (valid && !other.valid) return true;
            if (!valid && other.valid) return false;
            return cost < other.cost;
        }
        bool operator>(const Cost& other) const {
            if (valid && !other.valid) return false;
            if (!valid && other.valid) return true;
            return cost > other.cost;
        }
        bool operator==(const Cost& other) const {
            return (cost == other.cost) && (valid == other.valid);
        }
        void print() const {
            string validStr = valid ? "true" : "false";
            cout << "cost: " << cost << ", hpwl: " << hpwl << ", area: " << area
                 << ", width: " << width << ", height: " << height
                 << ", valid: " << validStr << endl;
        }
};

// for insert back
class InsertInfo{
    public:
        InsertInfo() : block(nullptr), pos(-1) { }
        InsertInfo(Block* block, int pos) : block(block), pos(pos) { }
        Block *block;
        int pos; // 0: right, 1: top 2: root
};

class Floorplanner
{
    public:
        friend void printFloorplanResult(std::fstream& output, Floorplanner* fp, chrono::duration<double>& duration);
        Floorplanner(std::fstream& input_blk, std::fstream& input_net, double alpha);
        ~Floorplanner();
        void floorplan();

        void SA_basic();
        void putBlockInDamnBox();
        void finalTuning();
        void shiftTuning();

    private:
        // read file
        void readBlock(std::fstream& input_blk);
        void readNet(std::fstream& input_net);
        
        void initPlace();
        void initPlace_2();
        void initPerturbation(); 
        void calCost();
        void updateBestCost(); // update best code when normalized area and hpwl are changed
        void updatePrevCost(); // update prev code when normalized area and hpwl are changed
        void calFinalCost();
        double calAspectRatioTerm(size_t height, size_t width);
        size_t calExceedArea(size_t height, size_t width);
        void updateGamma(bool increase_anyway, double exponent, double increase_ratio, double &upper_bound, double &lower_bound);

        // B* tree
        void packing();
        void updateBestBSTree();
        void retrieveBestBSTree();
        // op1 random rotate
        // void randomRotate(Block *block);
        // op2 random delete and insert
        void deleteBlock(Block* block, vector<InsertInfo>& insertInfoVec, vector<Block*>& parentBlockVec);
        void insertBlock(Block* block, Block* hostBlock, int flag); // flag means the block is inserted to the right or top of hostBlock
        void reInsertBlock(Block* block, vector<InsertInfo>& insertInfoVec, vector<Block*>& parentBlockVec); // undo
        // op3 random swap
        void swapTwoBlocks(Block* block1, Block* block2);

        void debugPrint();
        void printBestTreeTopology();
        void printTree(ostream& outs);

    private:
        // floorlan information
        size_t _numBlocks; // number of blocks
        size_t _numTerminals; // number of terminals
        size_t _numNets;   // number of nets
        size_t _outlineWidth; // outline width
        size_t _outlineHeight; // outline height
        double _givenAlpha; // user given alpha
        std::vector<Block*> _blockVec; // vector of blocks
        std::vector<Net*>   _netVec; // vector of nets
        unordered_map<string, int> _name2IntMap; // map of block name to block num in _blockVec
        
        // for B* tree
        Block* _bSTree; // b* tree root
        Contour _contour; // contour of the b* tree
        vector<vector<Block*>> _bestBSTreeTop; // best b* tree topology

        // for SA
        size_t _numIter; // number of iterations for SA
        size_t _numIterPerTemp; // number of iterations per temperature
        double _temperature; // temperature for SA
        double _coolTemp; // cooling temperature for SA
        double _coolRate; // cooling rate for SA
        double _alpha; // alpha for SA
        double _beta;
        double _gamma;
        
        size_t _normArea; // normalized area for SA
        double _normHPWL; // normalized HPWL for SA
        double _normAspectRatio; // normalized aspect ratio for SA (R* - R)^2
        size_t _normExceedArea; // normalized exceed area for SA

        Cost _prevCost; // previous cost for SA
        Cost _cost; // cost for SA
        Cost _bestCost; // best cost for SA

};

#endif // FLOORPLANNER_H