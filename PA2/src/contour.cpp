#include <iostream>
#include "contour.h"

using namespace std;

Contour::~Contour()
{
    clear();
    segNode* node = _trash;
    while (node != nullptr) {
        segNode* nextNode = node->_next;
        delete node;
        node = nextNode;
    }
}

size_t Contour::getY(size_t x, size_t w, size_t h)
{
    // find the segment that contains x
    segNode* fnode = findSegNode(x); // fnode and bnode are the frontest and backest segNode which form the segment containing [x, x+w]
    segNode* bnode = nullptr; // fnode and bnode are the frontest and backest segNode which form the segment containing [x, x+w]
    if (fnode == nullptr){
        insert(x, x + w, h, _tail, nullptr);
        // _last = _tail;
        return 0;
        // return h;
    }
    // the segment is found,
    // calculate the max y coordinate of the segments that constain [x, x+w]
    // delete the segments that are contained in [x, x+w]
    // insert new segment [x, x+w] with the max y coordinate
    bnode = fnode;
    size_t maxY = fnode->_y;
    while (x+w > bnode->_x1 && bnode->_next != nullptr) {
        if (bnode->_y > maxY) {
            maxY = bnode->_y;
        }
        bnode = bnode->_next;
    }
    if (bnode->_x1 >= x+w) {
        // bnode is the last node
        if (bnode->_y > maxY) {
            maxY = bnode->_y;
        }
    }
    else {
        if (bnode != nullptr && bnode->_y > maxY) {
            maxY = bnode->_y;
        }
        bnode = nullptr;
    }
    // segments adjustment
    // normal cases
    // [          ]
    // [        ]
    //    [     ]
    // |---|--|---|  segments
    //   [        ]
    // [               ]   exceed 1
    //   [               ] exceed 2

    // extreme case
    // [     ]
    //   [ ]
    // |-----|  only one segment
    //    [  ]
    // [   ]
    // check if the frontNode is the same as the backNode
    if (fnode == bnode) {
        // four cases
        if (fnode->_x0 == x && fnode->_x1 == x + w) {
            // case 1: the segment is the same as [x, x+w]
            fnode->_y = maxY + h;
        }
        else if (fnode->_x0 < x && fnode->_x1 > x + w) {
            // case 2: the segment contains [x, x+w]
            size_t originalX0 = fnode->_x0;
            size_t originalX1 = fnode->_x1;
            size_t originalY = fnode->_y;
            fnode->_x0 = x;
            fnode->_x1 = x + w;
            fnode->_y = maxY + h;
            insert(originalX0, x, originalY, fnode->_prev, fnode);
            insert(x + w, originalX1, originalY, fnode, fnode->_next);
        }
        else if (fnode->_x0 < x && fnode->_x1 == x + w) {
            // case 3: the segment's right end is the same as x+w
            fnode->_x1 = x;
            insert(x, x + w, maxY + h, fnode, fnode->_next);
        }
        else if (fnode->_x0 == x && fnode->_x1 > x + w) {
            // case 4: the segment's left edge is the same as x
            fnode->_x0 = x + w;
            insert(x, x + w, maxY + h, fnode->_prev, fnode);
        }
        else {
            cerr << "Missing case in getY\n";
            cerr << "one segment case\n";
            cerr << "fnode: " << fnode->_x0 << ", " << fnode->_x1 << ", " << fnode->_y << endl;
            cerr << "bnode: " << bnode->_x0 << ", " << bnode->_x1 << ", " << bnode->_y << endl;
            cerr << "x: " << x << ", w: " << w << ", h: " << h << endl;
            cerr << "maxY: " << maxY << endl;
        }
    }
    // normal cases
    else {
        // 6 cases
        segNode *nodeToDelete = nullptr;
        // case 1: exceed 1
        if (fnode->_x0 == x && bnode == nullptr){
            fnode->_x1 = x + w;
            fnode->_y = maxY + h;
            nodeToDelete = fnode->_next;
        }
        // case 2: exceed 2
        else if (fnode->_x0 < x && bnode == nullptr){
            fnode->_x1 = x;
            insert(x, x + w, maxY + h, fnode, fnode->_next);
            nodeToDelete = fnode->_next->_next;

        }
        // case 3: the segment is the same as [x, x+w]
        else if (fnode->_x0 == x && bnode->_x1 == x + w) {
            fnode->_x1 = x + w;
            fnode->_y = maxY + h;
            nodeToDelete = fnode->_next;
        }
        // case 4: the segment's left edge is the same as x
        else if (fnode->_x0 == x && bnode->_x1 > x + w) {
            fnode->_x1 = x + w;
            fnode->_y = maxY + h;
            bnode->_x0 = x + w;
            nodeToDelete = fnode->_next;
        }
        // case 5: the segment's right end is the same as x+w
        else if (fnode->_x0 < x && bnode->_x1 == x + w) {
            fnode->_x1 = x;
            fnode = fnode->_next;
            fnode->_x0 = x;
            fnode->_x1 = x + w;
            fnode->_y = maxY + h;
            nodeToDelete = fnode->_next;
        }
        // case 6: the segment contains [x, x+w]
        else if (fnode->_x0 < x && bnode->_x1 > x + w) {
            // case 6: the segment contains [x, x+w]
            fnode->_x1 = x;
            bnode->_x0 = x + w;
            insert(x, x + w, maxY + h, fnode, fnode->_next);
            nodeToDelete = fnode->_next->_next;
        }
        else {
            cerr << "Missing case in getY\n";
            cerr << "multiple segments case\n";
            cerr << "fnode: " << fnode->_x0 << ", " << fnode->_x1 << ", " << fnode->_y << endl;
            cerr << "bnode: " << bnode->_x0 << ", " << bnode->_x1 << ", " << bnode->_y << endl;
            cerr << "x: " << x << ", w: " << w << ", h: " << h << endl;
            cerr << "maxY: " << maxY << endl;
        }
        // delete the segments that are contained in [x, x+w]
        while (nodeToDelete != nullptr && nodeToDelete->_x0 < x + w && nodeToDelete->_x0 >= x && nodeToDelete->_x1 > x && nodeToDelete->_x1 <= x + w) {
            segNode* nextNode = nodeToDelete->_next;
            erase(nodeToDelete);
            nodeToDelete = nextNode;
        }

    }
    
    return maxY;
    // return maxY + h;
}

segNode* Contour::findSegNode(size_t x)
{
    // find the segment that contains x
    // find from the segNode _last
    // if _last is nullptr, start from the head
    // segNode* node = (_last == nullptr) ? _head : _last; // currently have bug
    segNode* node = _head;
    while (node != nullptr) {
        if (node->_x0 <= x && node->_x1 > x) {
            return node;
        }
        if (node->_x0 > x) {
            node = node->_prev;
        }
        else{
            // node->_x1 < x
            node = node->_next;
        }
    }
    return nullptr;
}

// insert a segment into the linked list
// revise the head and tail pointer if needed
void Contour::insert(size_t x0, size_t x1, size_t y, segNode* frontNode, segNode* backNode)
{
    if (_trash != nullptr){
        segNode *newN = _trash;
        _trash = _trash->_next;
        
        newN->_prev = frontNode;
        newN->_next = backNode;
        newN->_x0 = x0;
        newN->_x1 = x1;
        newN->_y = y;
        if (_trash != nullptr)
            _trash->_prev = nullptr;
        
        // if the frontNode and backNode are both nullptr
        // means the size of the list is 0
        if (frontNode == nullptr && backNode == nullptr) {
            _head = newN;
            _tail = newN;
        }
        else if (backNode == nullptr) {
            // if the backNode is nullptr
            // means insert the new node at the end of the list
            _tail = newN;
            frontNode->_next = newN;
        }
        else if (frontNode == nullptr) {
            // if the frontNode is nullptr
            // insert to the head
            _head = newN;
            backNode->_prev = newN;
        }
        else {
            frontNode->_next = newN;
            backNode->_prev = newN;
        }
    }
    else {
        segNode* newNode = new segNode(frontNode, backNode, x0, x1, y);
        // if the frontNode and backNode are both nullptr
        // means the size of the list is 0
        if (frontNode == nullptr && backNode == nullptr) {
            _head = newNode;
            _tail = newNode;
        }
        else if (backNode == nullptr) {
            // if the backNode is nullptr
            // means insert the new node at the end of the list
            _tail = newNode;
            frontNode->_next = newNode;
        }
        else if (frontNode == nullptr) {
            // if the frontNode is nullptr
            // insert to the head
            _head = newNode;
            backNode->_prev = newNode;
        }
        else {
            frontNode->_next = newNode;
            backNode->_prev = newNode;
        }
    }
    _size++;
    return;
}

void Contour::print()
{
    cout << "Contour: ";
    cout << "size: " << _size << endl;
    segNode* node = _head;
    while (node != nullptr) {
        cout << "[" << node->_x0 << ", " << node->_x1 << ", " << node->_y << "] ";
        node = node->_next;
        cout << endl;
    }
    cout << endl;
}

// move a single segNode to the trash
// if the trash is empty, set the trash to the node
void Contour::moveTrash(segNode* node) {
    if (node == nullptr) {
        cerr << "Error: move nullptr to trash\n";
        return;
    }
    if (_trash == nullptr) {
        _trash = node;
        node->_next = nullptr;
        node->_prev = nullptr;
    }
    else {
        node->_next = _trash;
        _trash->_prev = node;
        node->_prev = nullptr;
        _trash = node;
    }
}

// link the whole linked list to the trash
void Contour::clear()
{
    if (_tail != nullptr && _trash != nullptr) {
        _trash->_prev = _tail;
        _tail->_next = _trash;
        _trash = _head;
    }
    else if (_tail != nullptr) {
        _trash = _head;
        // _tail->_next = _trash;
    }
    _head = nullptr;
    _tail = nullptr;
    // _last = nullptr;
    _size = 0;
}

// erase a segment from the linked list
// put the deleted segment into the trash
// revise the head and tail pointer if needed
// return the deleted segment
void Contour::erase(segNode* node)
{
    if (node == nullptr) return;
    
    node->_x0 = 0;
    node->_x1 = 0;
    node->_y = 0;
    if (_size == 1){
        _head = nullptr;
        _tail = nullptr;
    }
    else if (node == _head){
        _head = node->_next;
        _head->_prev = nullptr;
        
    }
    else if (node == _tail){
        _tail = node->_prev;
        _tail->_next = nullptr;
        
    }
    else {
        // if the node is in the middle of the list
        node->_prev->_next = node->_next;
        node->_next->_prev = node->_prev;
    }
    
    moveTrash(node);
    _size--;
    // return node;
}