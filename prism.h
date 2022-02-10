#include <fmt/core.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>

#include <random>
using std::vector;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> Cloud;

void filterPrism(Cloud::Ptr cloud);
Cloud::Ptr createRing(float rMin, float rMax);
Cloud::Ptr createTwoParts();
Cloud::Ptr createCube();