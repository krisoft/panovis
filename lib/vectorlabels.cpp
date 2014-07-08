#include "vectorlabels.h"


void VectorLabels::allocate(const int length, LabelRef refs[] ){
  int ref_index = 0;
  LabelRef ref_candidate = 0;
  std::map<LabelRef,int>::iterator it=this->label_map.begin();
  while( ref_index<length ){
    if( (it == this->label_map.end()) || (it->first!=ref_candidate) ){
      refs[ ref_index ] = ref_candidate;
      this->label_map[ ref_candidate ] = this->label_map.size();
      ++ref_candidate;
      ++ref_index;
    } else {
      ++it;
      ++ref_candidate;
    }
  }
}

void VectorLabels::release(const int length, LabelRef refs[] ){
  int index = 0;
  std::map<LabelRef,int>::iterator it=this->label_map.begin();
  while( it!=this->label_map.end() ){
    bool found = false;
    for(int ref_index=0; ref_index<length; ref_index++){
      if( refs[ref_index]==it->first){
        found = true;
        break;
      }
    }
    if( found ){
      this->label_map.erase( it++ );
    } else {
      it->second = index++;
      ++it;
    }
  }
}


int VectorLabels::index(const LabelRef ref ){
  return this->label_map.at( ref );
}