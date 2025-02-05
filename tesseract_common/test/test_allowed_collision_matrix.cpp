#include <gtest/gtest.h>
#include <tesseract_common/allowed_collision_matrix.h>

using namespace tesseract_common;

class AllowedCollisionMatrixTest : public ::testing::Test
{
protected:
  AllowedCollisionMatrix acm;
};

TEST_F(AllowedCollisionMatrixTest, AddAndCheckCollision) // NOLINT
{
  acm.addAllowedCollision("link1", "link2", "test reason");
  EXPECT_TRUE(acm.isCollisionAllowed("link1", "link2"));
  EXPECT_TRUE(acm.isCollisionAllowed("link2", "link1")); // Order should not matter
}

TEST_F(AllowedCollisionMatrixTest, RemoveSpecificCollision) // NOLINT
{
  acm.addAllowedCollision("link1", "link2", "test reason");
  acm.removeAllowedCollision("link1", "link2");
  EXPECT_FALSE(acm.isCollisionAllowed("link1", "link2"));
}

TEST_F(AllowedCollisionMatrixTest, RemoveCollisionsByLink) // NOLINT
{
  acm.addAllowedCollision("link1", "link2", "reason1");
  acm.addAllowedCollision("link1", "link3", "reason2");
  acm.removeAllowedCollision("link1");
  EXPECT_FALSE(acm.isCollisionAllowed("link1", "link2"));
  EXPECT_FALSE(acm.isCollisionAllowed("link1", "link3"));
  EXPECT_TRUE(acm.getAllAllowedCollisions().empty());
}

TEST_F(AllowedCollisionMatrixTest, ClearAllowedCollisions) // NOLINT
{
  acm.addAllowedCollision("link1", "link2", "reason");
  acm.addAllowedCollision("link3", "link4", "reason");
  acm.clearAllowedCollisions();
  EXPECT_TRUE(acm.getAllAllowedCollisions().empty());
}

TEST_F(AllowedCollisionMatrixTest, InsertAllowedCollisionMatrix) // NOLINT
{
  AllowedCollisionMatrix other_acm;
  other_acm.addAllowedCollision("link1", "link2", "reason");
  other_acm.addAllowedCollision("link3", "link4", "reason");

  acm.insertAllowedCollisionMatrix(other_acm);

  EXPECT_TRUE(acm.isCollisionAllowed("link1", "link2"));
  EXPECT_TRUE(acm.isCollisionAllowed("link3", "link4"));
}

TEST_F(AllowedCollisionMatrixTest, ReserveSpace) // NOLINT
{
  // This test is for API coverage; reserve does not alter functionality directly.
  acm.reserveAllowedCollisionMatrix(10);
  EXPECT_TRUE(acm.getAllAllowedCollisions().empty());
}

TEST_F(AllowedCollisionMatrixTest, EqualityOperators) // NOLINT
{
  AllowedCollisionMatrix acm1;
  AllowedCollisionMatrix acm2;

  acm1.addAllowedCollision("link1", "link2", "reason1");
  acm2.addAllowedCollision("link1", "link2", "reason1");

  EXPECT_TRUE(acm1 == acm2);
  EXPECT_FALSE(acm1 != acm2);

  acm2.addAllowedCollision("link3", "link4", "reason2");
  EXPECT_FALSE(acm1 == acm2);
  EXPECT_TRUE(acm1 != acm2);
}

TEST_F(AllowedCollisionMatrixTest, StreamOutputOperator) // NOLINT
{
  acm.addAllowedCollision("link1", "link2", "reason");
  std::ostringstream oss;
  oss << acm;
  EXPECT_FALSE(oss.str().empty());
  EXPECT_NE(oss.str().find("link1"), std::string::npos);
  EXPECT_NE(oss.str().find("link2"), std::string::npos);
  EXPECT_NE(oss.str().find("reason"), std::string::npos);
}
