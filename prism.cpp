#include "prism.h"

int main(void) {
	 auto cloud = createRing(0.25f, 0.5f);
	// auto cloud = createTwoParts();
	filterPrism(cloud);

	return 0;
}

Cloud::Ptr createRing(float rMin, float rMax) {
	fmt::print("create a ring cloud (1000 points)\n");
	Cloud::Ptr ring(new Cloud); // r ∈ [rMin, rMax], z ∈ [-0.01, 0.01]
	{
		// random seed
		std::random_device rd;	// obtain a random number from hardware
		std::mt19937 gen(rd()); // seed the generator
		std::uniform_real_distribution<float> radiusDist(rMin, rMax);
		std::uniform_real_distribution<float> radianDist(-M_PI, M_PI);
		std::uniform_real_distribution<float> zDist(-0.01f, 0.01f);
		for (size_t i = 0; i < 1000; i++) {
			float radius = radiusDist(gen);
			float angle = radianDist(gen);
			float z = zDist(gen);
			PointT point(
				cos(angle) * radius,
				sin(angle) * radius,
				z);
			ring->push_back(point);
		}
	}

	Cloud::Ptr cube = createCube();

	fmt::print("merging the ring and cube (2000 points)\n"); // cloud = ring ∪ cube
	Cloud::Ptr cloud(new Cloud);
	{
		Cloud::concatenate(*cloud, *ring);
		Cloud::concatenate(*cloud, *cube);
	}
	return cloud;
}

Cloud::Ptr createTwoParts() {
	Cloud::Ptr disk = createRing(0, 0.2f);
	Cloud::Ptr cloud(new Cloud);
	for (auto& point : disk->points) {
		auto left = point;
		auto right = point;
		left.x -= 0.5;
		right.x += 0.5;
		cloud->points.push_back(left);
		cloud->points.push_back(right);
	}
	return cloud;
}

Cloud::Ptr createCube() {
	fmt::print("create a cube cloud (1000 points)\n");
	Cloud::Ptr cube(new Cloud); // [-1, 1] at all axes
	{
		// random seed
		std::random_device rd;	// obtain a random number from hardware
		std::mt19937 gen(rd()); // seed the generator
		std::uniform_real_distribution<float> dist(-1.f, 1.f);

		for (size_t i = 0; i < 1000; i++) {
			PointT point(dist(gen), dist(gen), dist(gen));
			cube->push_back(point);
		}
	}
	return cube;
}

void filterPrism(Cloud::Ptr cloud) {
	fmt::print("finding the ring plane using RANSAC\n");
	pcl::ModelCoefficients::Ptr planeCoefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr planeInliers(new pcl::PointIndices);
	{
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(0.05);
		seg.setInputCloud(cloud);
		seg.segment(*planeInliers, *planeCoefficients);
		fmt::print("\t the ring plane has {} points\n", planeInliers->indices.size());
	}

	fmt::print("filtering plane outliers (same plane, far from the ring)\n");
	pcl::StatisticalOutlierRemoval<PointT> sor;
	{
		sor.setInputCloud(cloud);
		sor.setIndices(planeInliers);
		sor.setMeanK(50); // these are the default values from the tutorial, and they kinda work
		sor.setStddevMulThresh(1.0);
		sor.filter(planeInliers->indices);
		fmt::print("\t now the ring plane has {} points\n", planeInliers->indices.size());
	}

	fmt::print("finding the concave hull\n");
	Cloud::Ptr hullCloud(new Cloud);
	vector<pcl::Vertices> polygons;
	pcl::ConcaveHull<PointT> concaveHull;
	{
		concaveHull.setIndices(planeInliers);
		concaveHull.setInputCloud(cloud);
		concaveHull.setAlpha(0.1);
		concaveHull.reconstruct(*hullCloud, polygons);
		fmt::print("\tconcave hull has {} points, and {} vectors\n", hullCloud->size(), polygons.size());
	}

	fmt::print("filtering about ring prism\n");
	pcl::ExtractPolygonalPrismData<PointT> pp;
	pcl::PointIndices::Ptr pointsInsidePolygons(new pcl::PointIndices);
	{
		pp.setInputCloud(cloud);
		pp.setInputPlanarHull(hullCloud);
		pp.setHeightLimits(-1, 1);
		pp.segment(*pointsInsidePolygons);

		fmt::print("\tprism has {} points\n", pointsInsidePolygons->indices.size());
	}

	pcl::io::savePLYFile(fmt::format("c:/temp/cloud.ply"), *cloud);
	pcl::io::savePLYFile(fmt::format("c:/temp/plane.ply"), *cloud, planeInliers->indices);
	pcl::io::savePLYFile(fmt::format("c:/temp/hull.ply"), *hullCloud);
	pcl::io::savePLYFile(fmt::format("c:/temp/prism.ply"), *cloud, pointsInsidePolygons->indices);
}
