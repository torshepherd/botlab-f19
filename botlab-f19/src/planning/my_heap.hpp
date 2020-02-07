#ifndef MY_HEAP_HPP
#define MY_HEAP_HPP

#include <iostream>
#include <stdio.h>
#include <vector>
#include <unordered_map>

#define MyHeapDouble MyHeap<double>
#define MyHeapFloat MyHeap<float>
#define MyHeapInt MyHeap<int>

#define HeapNodeDouble HeapNode<double>
#define HeapNodeFloat HeapNode<float>
#define HeapNodeInt HeapNode<int>

template<class T>
class HeapNode{
public:
	int id_;
	T key_;
	HeapNode(int id, T key){
		id_ = id;
		key_ = key;
	}
};

template<class T>
class MyHeap{
private:
	int max_size_, now_size_;
	HeapNode<T> **node_address_;

	
	std::unordered_map<int, int> id2loc_;
	bool swap(int loc1, int loc2);
	
public:
	MyHeap(int array_size_set){
		const int array_size = array_size_set; //2097152; //16384;
		node_address_ = new HeapNode<T>*[array_size];

		max_size_ = array_size;
		now_size_ = 0;
	}
	~MyHeap(){
		for (int i = 0; i < now_size_; i++){
			delete node_address_[i];
		}
		delete[] node_address_;
	}
	bool       insert(HeapNode<T>* new_address);
	bool       update(int id, T key);
	HeapNode<T>*  findMin();
	HeapNode<T>*  deleteMin();
	bool       deleteNode(int id);
	int        parent(int loc);
	int        left(int loc);
	int        right(int loc);
	int        reorder(int loc);
	bool       isEmpty();
	bool       isFull();
	int        size();
	bool       hasId(int id);
	void       extendHeap();
	void       printHeap();
	//bool make_heap();

};

template<class T>
bool MyHeap<T>::isFull(){
	return (now_size_ >= max_size_);
}

template<class T>
bool MyHeap<T>::isEmpty(){
	return (now_size_ == 0);
}

template<class T>
int MyHeap<T>::size(){
	return now_size_;
}

template<class T>
bool MyHeap<T>::hasId(int id){
	return (id2loc_.find(id) != id2loc_.end());
}

template<class T>
int MyHeap<T>::parent(int loc){
	int parent_loc;
	if (loc >= 1){
		parent_loc = (loc - 1) / 2;
	}
	else{
		parent_loc = -1;
	}
	return parent_loc;
}

template<class T>
int MyHeap<T>::left(int loc){
	int left_loc = 2 * loc + 1;
	if (left_loc < now_size_){
		return left_loc;
	}
	else{
		return -1;
	}
}

template<class T>
int MyHeap<T>::right(int loc){
	int right_loc = 2 * loc + 2;
	if (right_loc < now_size_){
		return right_loc;
	}
	else{
		return -1;
	}
}

template<class T>
bool MyHeap<T>::swap(int loc1, int loc2){
	int id1 = node_address_[loc1]->id_;
	int id2 = node_address_[loc2]->id_;
	id2loc_[id1] = loc2;
	id2loc_[id2] = loc1;

	HeapNode<T> * temp = node_address_[loc1];
	node_address_[loc1] = node_address_[loc2];
	node_address_[loc2] = temp;

	return true;
}

template<class T>
int MyHeap<T>::reorder(int loc){
	int left_loc = left(loc);
	int right_loc = right(loc);
	int parent_loc = parent(loc);
	bool flag_up = false;
	if (parent_loc != -1){
		flag_up = node_address_[loc]->key_ < node_address_[parent_loc]->key_;
	}
	if (flag_up){
		while(parent_loc != -1){
			// If smaller than the parent, swap up
			if( node_address_[loc]->key_ < node_address_[parent_loc]->key_ ){
				swap(loc, parent_loc);
				loc = parent_loc;
				parent_loc = parent(loc);
			}
			else{
				break;
			}
		}
	}
	else{
		while(left_loc != -1){
			// Find the smallest child
			T min_key = node_address_[left_loc]->key_;
			int min_loc = left_loc;
			if(right_loc != -1){
				if(node_address_[right_loc]->key_ < min_key){
					min_key = node_address_[right_loc]->key_;
					min_loc = right_loc;
				}
			}
			// If larger than the smallest child, swap down
			if(node_address_[loc]->key_ > min_key){
				//printf("self:%d,%d\tleft:%d,%d\tright:%d,%d\t\n", loc, node_address_[loc]->key_, left_loc, node_address_[left_loc]->key_, right_loc, node_address_[right_loc]->key_);			
				swap(loc, min_loc);
				loc = min_loc;
				left_loc = left(loc);
				right_loc = right(loc);	
			}
			else{
				break;
			}
		}
	}
	return loc;
}

template<class T>
void MyHeap<T>::extendHeap(){
	const int new_size = 2 * max_size_;

	HeapNode<T>** new_node_address = new HeapNode<T>*[new_size];
	for(int i = 0; i < max_size_; i++){
		new_node_address[i] = node_address_[i];
	}

	delete[] node_address_;
	node_address_ = new_node_address;
	max_size_ = new_size;
	//printf("extended: %d\n", max_size_);
}

template<class T>
void MyHeap<T>::printHeap(){
	int max_number_in_level = 1;
	int now_level = 0;
	printf("heap:");
	for(int i = 0; i < now_size_; i++){
		if(i >= max_number_in_level - 1){
			max_number_in_level *= 2;
			now_level++;
			printf("\nLevel%d\t",now_level);
		}
		printf("(%d: [%d], %d)\t", (int) node_address_[i]->key_, node_address_[i]->id_, i);
	}
	printf("\n\ntable:\n");
	for ( auto &x: id2loc_ ){
    	printf("([%d]:%d)\n", x.first, x.second);
	}
	printf("\n\n");
}

template<class T>
bool MyHeap<T>::insert(HeapNode<T>* new_address){
	if(isFull()){
		extendHeap();
	}
	node_address_[now_size_] = new_address;
	id2loc_[new_address->id_] = now_size_;
	now_size_++;
	reorder(now_size_-1);

	return true;
}

template<class T>
bool MyHeap<T>::update(int id, T key){
	int loc = id2loc_[id];
	node_address_[loc]->key_ = key;
	reorder(loc);

	return true;
}

template<class T>
bool MyHeap<T>::deleteNode(int id){
	int loc = id2loc_[id];
	swap(loc, now_size_-1);
	//delete node_address_[now_size_-1];
	id2loc_.erase(id);
	now_size_ -= 1;
	reorder(loc);

	return true;
}

template<class T>
HeapNode<T>* MyHeap<T>::findMin(){
	return node_address_[0];
}

template<class T>
HeapNode<T>* MyHeap<T>::deleteMin(){
	if(!isEmpty()){
		HeapNode<T>* return_address = node_address_[0];
		deleteNode( node_address_[0]->id_ );
		return return_address;		
	}
	else{
		return nullptr;
	}

}

#endif // MY_HEAP_HPP