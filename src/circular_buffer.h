#ifndef _CIRCULAR_BUFFER_
#define _CIRCULAR_BUFFER_

#include <stdio.h>
#include <iostream>
#include <vector>

using std::vector;

template <class T>
class CircularBuffer { 
public: 

   /**
    *
    */
   CircularBuffer ();

   /**
    *
    */
   void resize(unsigned int size);

   /**
    *
    */
   void reset();
   
   /**
    *
    */
   void push(T elem);

   /**
    *
    */
   T* pull();

   /**
    *
    */
   void rewind (){ head = 0;}

   /**
    *
    */
   unsigned int get_last_elem_pos(){return tail;}
   
private:
   unsigned int head, tail;
   unsigned int oldest_pos;
   unsigned int elem_nb;
   vector<T> container;
};


template <class T>
CircularBuffer<T>::CircularBuffer(){ 
   head = 0; 
   tail = 0;
   oldest_pos=0;
   container.clear();
} 

template <class T>
void CircularBuffer<T>::resize(unsigned int size)
{ 
   container.resize (size);
} 

template <class T>
void CircularBuffer<T>::push(T elem)
{
   tail++;
   if (tail == container.size())
      tail = 0;

   container[tail] = elem;
}

template <class T>
void CircularBuffer<T>::reset()
{
   head = tail = 0;
   oldest_pos=0;

   for(unsigned int i=0; i<container.size(); i++)
   {
      container[i].init();
   }
}

template <class T>
T* CircularBuffer<T>::pull()
{
   T* elem = NULL;

   elem = &container[head];
   head++;
   if (head == container.size())
      head = 0;
   
   return elem;
}

#endif
