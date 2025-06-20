#ifndef MODULE_H
#define MODULE_H

#include <vector>
#include <string>
using namespace std;

// Block or Terminal
class Block
{
    public:
        // constructor and destructor
        Block(string& name, size_t w, size_t h, bool isTerminal) :
            _name(name), _isTerminal(isTerminal), _parent(nullptr), _top(nullptr), _right(nullptr){ 
                if (isTerminal) {
                    _w = 0; _h = 0;
                    _x = w; _y = h;
                } else {
                    _w = w; _h = h;
                    _x = 0; _y = 0;
                }
                _bestCoordinate[0] = 0;
                _bestCoordinate[1] = 0;
                _bestCoordinate[2] = 0;
                _bestCoordinate[3] = 0;
            }
        ~Block() { }

        // basic access methods
        const size_t getWidth()  { return _w; }
        const size_t getHeight() { return _h; }
        const size_t getArea()  { return _h * _w; }
        const size_t getX() { return _x; }
        const size_t getY() { return _y; }
        const bool isTerminal() { return _isTerminal; }
        const string getName() { return _name; }
        Block* getParent() { return _parent; }
        Block* getTop() { return _top; }
        Block* getRight() { return _right; }
        const size_t* getBestCoordinate() { return _bestCoordinate; }
        static size_t getMaxX() { return _maxX; }
        static size_t getMaxY() { return _maxY; }

        // set functions
        void setWidth(size_t w)         { _w = w; }
        void setHeight(size_t h)        { _h = h; }
        static void setMaxX(size_t x)   { _maxX = x; }
        static void setMaxY(size_t y)   { _maxY = y; }
        void setPos(size_t x, size_t y) {
            _x = x;   _y = y;
        }
        void setName(string& name) { _name = name; }
        void setParent(Block* parent) { _parent = parent; }
        void setTop(Block* top) { _top = top; }
        void setRight(Block* right) { _right = right; }
        
        // midify methods
        void rotate() {
            size_t temp = _w;
            _w = _h;
            _h = temp;
        }
        void setBest(){
            _bestCoordinate[0] = _x;
            _bestCoordinate[1] = _y;
            _bestCoordinate[2] = _x + _w;
            _bestCoordinate[3] = _y + _h;
        }
        void shift(size_t dx, size_t dy) {
            if (!_isTerminal){
                _x += dx;
                _y += dy;
            }
        }

    private:
        string         _name;      // module name
        size_t          _w;         // width of the block
        size_t          _h;         // height of the block
        size_t         _x;        // min x coordinate of the block
        size_t         _y;        // min y coordinate of the block
        bool         _isTerminal; // true if the block is a terminal
        
        Block * _parent; // parent block (block not terminal)
        Block * _top;
        Block * _right;
        
        size_t _bestCoordinate[4]; // best coordinate for SA, 
                                   // 0: x0, 1: y0, 2: x1, 3: y1

        static size_t   _maxX;      // maximum x coordinate for all blocks
        static size_t   _maxY;      // maximum y coordinate for all blocks
};


class Net
{
public:
    // constructor and destructor
    Net()   { }
    ~Net()  { }

    // basic access methods
    const vector<Block*> getTermList()   { return _blockList; }
    const size_t getSize() { return _blockList.size(); }

    // modify methods
    void addTerm(Block* term) { _blockList.push_back(term); }

    // other member functions

    double calcHPWL() {
        if (_blockList.size() == 0) return 0.0;
        
        double minX, minY, maxX, maxY;
        if (_blockList[0]->isTerminal()) {
            minX = _blockList[0]->getX();
            minY = _blockList[0]->getY();
        }
        else{
            minX = _blockList[0]->getX() + _blockList[0]->getWidth() / 2.0;
            minY = _blockList[0]->getY() + _blockList[0]->getHeight() / 2.0;
        }
        maxX = minX;
        maxY = minY;
        
        for (auto block : _blockList) {
            if (block->isTerminal()) {
                if (block->getX() < minX) minX = block->getX();
                if (block->getY() < minY) minY = block->getY();
                if (block->getX() > maxX) maxX = block->getX();
                if (block->getY() > maxY) maxY = block->getY();
            }
            else{
                double x = block->getX() + block->getWidth() / 2.0;
                double y = block->getY() + block->getHeight() / 2.0;
                if (x < minX) minX = x;
                if (y < minY) minY = y;
                if (x > maxX) maxX = x;
                if (y > maxY) maxY = y;
            }
        }
        double hpwl = (maxX - minX) + (maxY - minY);
        return hpwl;
    }

private:
    vector<Block*>   _blockList;  // list of terminals the net is connected to
};

#endif  // MODULE_H
