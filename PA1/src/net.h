#ifndef NET_H
#define NET_H

// #include <unordered_set>
#include <unordered_set>
using namespace std;

class Net
{
public:
    // constructor and destructor
    Net(string& name) :
        _name(name) {
        _partCount[0] = 0; _partCount[1] = 0;
    }
    ~Net()  { }

    // basic access methods
    string getName()           const { return _name; }
    int getPartCount(int part) const { return _partCount[part]; }
    const unordered_set<int>& getCellList()  const { return _cellList; }
    int getSize() const { return _cellList.size(); }

    // set functions
    void setName(const string name) { _name = name; }
    void setPartCount(int part, const int count) { _partCount[part] = count; }
    void setCellList(const unordered_set<int>& cellList) { _cellList = cellList; }

    // modify methods
    void incPartCount(int part)     { ++_partCount[part]; }
    void decPartCount(int part)     { --_partCount[part]; }
    void addCell(const int cellId)  { _cellList.insert(cellId); }
    void eraseCell(const int cellId) { _cellList.erase(cellId); }

private:
    int             _partCount[2];  // Cell number in partition A(0) and B(1)
    string          _name;          // Name of the net
    unordered_set<int>     _cellList;      // List of cells the net is connected to
};

#endif  // NET_H
