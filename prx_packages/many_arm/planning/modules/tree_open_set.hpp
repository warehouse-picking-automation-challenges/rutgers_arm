/**
 * @file tree_open_set.hpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_CHUPLES_OPEN_SET_HPP
#define PRX_CHUPLES_OPEN_SET_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/boost/hash.hpp"
#include <vector>

namespace prx
{
    namespace util
    {

        /**
         *
         */
        template < class element_type_t >
        class open_set_t
        {
          public:

            /**
             *
             */
            open_set_t()
            {
                heap.clear();
                heap.push_back(new element_type_t());
                heap_size = 1;
            }

            /**
             *
             */
            ~open_set_t()
            {
                //Delete the held element
                delete heap[0];
                heap_size = 0;
            }

            /**
             *
             */
            void insert(element_type_t* element)
            {
                if( finder.find(element) != finder.end() )
                {
                    update( element, element );
                    return;
                }

                //Make sure we can actually put the element into the heap
                if( heap.size() > heap_size )
                {
                    heap[heap_size] = element;
                }
                else
                {
                    heap.push_back(element);
                }
                //Increment the size
                ++heap_size;

                finder[element] = heap_size - 1;
                reheap_up(heap_size - 1);
            }

            /**
             *
             */
            element_type_t* remove_min()
            {
                element_type_t* return_val = NULL;

                if( heap_size > 1 )
                {
                    return_val = heap[1];
                    heap[1] = heap[heap_size - 1];
                    finder[heap[1]] = 1;
                    finder.erase(return_val);

                    heap_size--;
                    reheap_down(1);
                }

                return return_val;
            }

            /**
             *
             */
            void update( element_type_t* element, element_type_t* new_element )
            {
                //Debug: make sure the element we want to replace is actually in here
                PRX_ASSERT(finder.find(element) != finder.end());

                //Get the index of the element to replace
                unsigned index = finder[element];
                //Then, if we are changing the element memory,
                if( element != new_element )
                {
                    //Update the finder to use the new element for indexing
                    finder[new_element] = index;
                    finder.erase(element);
                    //And make the heap point to the new memory
                    heap[index] = new_element;
                }

                //Then, fix the heap
                reheap_up(index);
                reheap_down(index);
            }

            /**
             *
             */
            void remove(element_type_t* element)
            {
                //If the element is indeed in here
                if( finder.find(element) != finder.end() )
                {
                    //Get the index
                    unsigned i = finder[element];
                    //Then, move it up in the heap
                    while( i != 1 )
                    {
                        swap(i, parent(i));
                        i = parent(i);
                    }

                    //And remove the minimum, we can do debugging too
                    element_type_t* node = remove_min();
                    PRX_ASSERT( node == element );
                }
            }

            /**
             *
             */
            bool contains(element_type_t* element)
            {
                return finder.find(element) != finder.end();
            }

            /**
             *
             */
            element_type_t* peek_min() const
            {
                return heap[1];
            }

            /**
             *
             */
            int size() const
            {
                return heap_size - 1;
            }

            /**
             *
             */
            bool empty() const
            {
                return size() == 0;
            }

            /**
             *
             */
            void clear()
            {
                heap_size = 1;
                finder.clear();
            }

            /**
             *
             */
            void get_items(std::vector<element_type_t*>& items)
            {
                items.assign(heap.begin() + 1, heap.begin() + heap_size);
            }


          private:

            /**
             *
             */
            void reheap_up(unsigned index)
            {
                unsigned parent_index = parent(index);

                if( parent_index != 0 && *heap[index] < *heap[parent_index] )
                {
                    swap(index, parent_index);
                    reheap_up(parent_index);
                }
            }

            /**
             *
             */
            void reheap_down(unsigned index)
            {
                unsigned child1_index = child1(index);
                unsigned child2_index = child2(index);

                if( child1_index != 0 )
                {
                    if( child2_index != 0 )
                    {
                        if( !(*heap[index] < *heap[child1_index]) || !(*heap[index] < *heap[child2_index]) )
                        {
                            if( *heap[child1_index] < *heap[child2_index] )
                            {
                                swap(child1_index, index);
                                reheap_down(child1_index);
                            }
                            else
                            {
                                swap(child2_index, index);
                                reheap_down(child2_index);
                            }
                        }
                    }
                    else
                    {
                        if( *heap[child1_index] < *heap[index] )
                        {
                            swap(index, child1_index);
                            reheap_down(child1_index);
                        }
                    }
                }
            }

            /**
             *
             */
            unsigned parent(unsigned index)
            {
                return index / 2;
            }

            /**
             *
             */
            unsigned child1(unsigned index)
            {
                unsigned val = index * 2;

                if( val < heap_size )
                    return val;
                return 0;
            }

            /**
             *
             */
            unsigned child2(unsigned index)
            {
                unsigned val = index * 2 + 1;

                if( val < heap_size )
                    return val;
                return 0;
            }

            /**
             *
             */
            void swap(unsigned val1, unsigned val2)
            {
                finder[heap[val1]] = val2;
                finder[heap[val2]] = val1;
                element_type_t* temp = heap[val1];
                heap[val1] = heap[val2];
                heap[val2] = temp;
            }

            std::vector<element_type_t*> heap;
            hash_t<element_type_t*, unsigned> finder;
            unsigned heap_size;
        };
    }
}



#endif
