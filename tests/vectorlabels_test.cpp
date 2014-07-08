#include "gtest/gtest.h"
#include "vectorlabels.h"

TEST(vectorlabels_test, simple_allocation)
{
  VectorLabels labels;
  LabelRef references1[1];
  labels.allocate( 1, references1 );
  EXPECT_EQ(0, references1[0]);

  EXPECT_EQ(0, labels.index( references1[0] ));

  LabelRef references2[5];
  labels.allocate( 5, references2 );
  EXPECT_EQ(1, references2[0]);
  EXPECT_EQ(2, references2[1]);
  EXPECT_EQ(3, references2[2]);
  EXPECT_EQ(4, references2[3]);
  EXPECT_EQ(5, references2[4]);

  EXPECT_EQ(0, labels.index( references1[0] ));
  EXPECT_EQ(1, labels.index( references2[0] ));
  EXPECT_EQ(2, labels.index( references2[1] ));
  EXPECT_EQ(3, labels.index( references2[2] ));
  EXPECT_EQ(4, labels.index( references2[3] ));
  EXPECT_EQ(5, labels.index( references2[4] ));
}


TEST(vectorlabels_test, simple_release)
{
  VectorLabels labels;
  LabelRef references1[3];
  labels.allocate( 3, references1 );
  EXPECT_EQ(0, references1[0]);
  EXPECT_EQ(1, references1[1]);
  EXPECT_EQ(2, references1[2]);

  EXPECT_EQ(0, labels.index( references1[0] ));
  EXPECT_EQ(1, labels.index( references1[1] ));
  EXPECT_EQ(2, labels.index( references1[2] ));

  labels.release( 3, references1 );

  LabelRef references2[5];
  labels.allocate( 5, references2 );
  EXPECT_EQ(0, references2[0]);
  EXPECT_EQ(1, references2[1]);
  EXPECT_EQ(2, references2[2]);
  EXPECT_EQ(3, references2[3]);
  EXPECT_EQ(4, references2[4]);

  EXPECT_EQ(0, labels.index( references2[0] ));
  EXPECT_EQ(1, labels.index( references2[1] ));
  EXPECT_EQ(2, labels.index( references2[2] ));
  EXPECT_EQ(3, labels.index( references2[3] ));
  EXPECT_EQ(4, labels.index( references2[4] ));

  LabelRef references3[2];
  labels.allocate( 2, references3 );
  EXPECT_EQ(5, references3[0]);
  EXPECT_EQ(6, references3[1]);

  EXPECT_EQ(0, labels.index( references2[0] ));
  EXPECT_EQ(1, labels.index( references2[1] ));
  EXPECT_EQ(2, labels.index( references2[2] ));
  EXPECT_EQ(3, labels.index( references2[3] ));
  EXPECT_EQ(4, labels.index( references2[4] ));
  EXPECT_EQ(5, labels.index( references3[0] ));
  EXPECT_EQ(6, labels.index( references3[1] ));

  LabelRef references4[1];
  references4[0] = references2[2];
  labels.release( 1, references4 );

  EXPECT_EQ(0, labels.index( references2[0] ));
  EXPECT_EQ(1, labels.index( references2[1] ));
  EXPECT_EQ(2, labels.index( references2[3] ));
  EXPECT_EQ(3, labels.index( references2[4] ));
  EXPECT_EQ(4, labels.index( references3[0] ));
  EXPECT_EQ(5, labels.index( references3[1] ));

  LabelRef references5[3];
  labels.allocate( 3, references5 );
  EXPECT_EQ(2, references5[0]);
  EXPECT_EQ(7, references5[1]);
  EXPECT_EQ(8, references5[2]);

  EXPECT_EQ(0, labels.index( references2[0] ));
  EXPECT_EQ(1, labels.index( references2[1] ));
  EXPECT_EQ(2, labels.index( references2[3] ));
  EXPECT_EQ(3, labels.index( references2[4] ));
  EXPECT_EQ(4, labels.index( references3[0] ));
  EXPECT_EQ(5, labels.index( references3[1] ));
  EXPECT_EQ(6, labels.index( references5[0] ));
  EXPECT_EQ(7, labels.index( references5[1] ));
  EXPECT_EQ(8, labels.index( references5[2] ));

}