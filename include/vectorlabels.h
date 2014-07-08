#ifndef RVISION_VECTORLABELS_H_
#define RVISION_VECTORLABELS_H_

#include <list>
#include <map>


typedef int LabelRef;

class VectorLabels {
    std::map<LabelRef,int> label_map;
  public:
    void allocate(const int length, LabelRef refs[] );
    void release(const int length, LabelRef refs[] );
    int index( const LabelRef ref );
};

#endif  // RVISION_VECTORLABELS_H_