#ifndef CONTOUR_H
#define CONTOUR_H

#include <cstddef> // for size_t

// segment
class segNode
{
    public:
        // constructor and destructor
        segNode(): _prev(nullptr), _next(nullptr), _x0(0), _x1(0), _y(0) { }
        segNode(size_t x0, size_t x1, size_t y): _prev(nullptr), _next(nullptr), _x0(x0), _x1(x1), _y(y) { }
        segNode(segNode* prev, segNode* next, size_t x0, size_t x1, size_t y): _prev(prev), _next(next), _x0(x0), _x1(x1), _y(y) { }
        void clear(){
            _prev = nullptr;
            _next = nullptr;
            _x0 = 0;
            _x1 = 0;
            _y = 0;
        }
        ~segNode() { }
        friend class Contour;
    
    private:
        segNode* _prev;
        segNode* _next;
        size_t _x0;
        size_t _x1;
        size_t _y;
};

// doubly linked-list of segments
class Contour
{
    public:
        // constructor and destructor
        Contour(): _head(nullptr), _tail(nullptr), _last(nullptr), _trash(nullptr), _size(0) { };
        ~Contour();

        size_t getY(size_t x, size_t w, size_t h);
        void clear();
        void print();
        
    private:
        
        segNode* findSegNode(size_t x);
        void insert(size_t x0, size_t x1, size_t y, segNode* frontNode, segNode* backNode);
        void erase(segNode* node);
        void moveTrash(segNode* node);
    
        segNode* _head; // head of the linked list
        segNode* _tail; // tail of the linked list
        segNode* _last; // last visited segNode
        segNode* _trash;
        size_t _size; // size of the linked list
};

#endif // CONTOUR_H