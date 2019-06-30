#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <algorithm>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_geometry/geometries.h>

TEST(TesseractGeometryUnit, Instantiation)
{
  auto box = std::make_shared<tesseract_geometry::Box>(1, 1, 1);
  auto cone = std::make_shared<tesseract_geometry::Cone>(1, 1);
  auto cylinder = std::make_shared<tesseract_geometry::Cylinder>(1, 1);
  auto plane = std::make_shared<tesseract_geometry::Plane>(1, 1, 1, 1);
  auto sphere = std::make_shared<tesseract_geometry::Shere>(1);
  auto convex_mesh = std::make_shared<tesseract_geometry::ConvexMesh>(nullptr, nullptr);
  auto mesh = std::make_shared<tesseract_geometry::Mesh>(nullptr, nullptr);
  auto sdf_mesh = std::make_shared<tesseract_geometry::SDFMesh>(nullptr, nullptr);
  auto octree = std::make_shared<tesseract_geometry::Octree>(nullptr, tesseract_geometry::Octree::SubType::BOX);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
