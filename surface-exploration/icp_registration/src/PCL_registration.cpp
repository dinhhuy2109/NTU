#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>

// int
// main(int argc, char** argv)
// {// registration raw.pcd test.pcd
//     //	Objects for storing the point clouds.
// 	pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud(new pcl::PointCloud<pcl::PointXYZ>);
// 	pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud(new pcl::PointCloud<pcl::PointXYZ>);
// 	pcl::PointCloud<pcl::PointXYZ>::Ptr finalCloud(new pcl::PointCloud<pcl::PointXYZ>);
// 	pcl::PointCloud<pcl::PointXYZ>::Ptr tftargetCloud(new pcl::PointCloud<pcl::PointXYZ>);
// 	pcl::PointCloud<pcl::PointXYZ>::Ptr spsourceCloud(new pcl::PointCloud<pcl::PointXYZ>);
// 	pcl::PointCloud<pcl::PointXYZ>::Ptr tfspsourceCloud(new pcl::PointCloud<pcl::PointXYZ>);

// 	pcl::PointCloud<pcl::PointXYZ>::Ptr estimateCloud(new pcl::PointCloud<pcl::PointXYZ>);

// 	//Read two PCD files from disk.
// 	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *sourceCloud) != 0)
// 	{
// 		return -1;
// 	}
// 	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], *targetCloud) != 0)
// 	{
// 		return -1;
// 	}

// 	if (pcl::io::loadPCDFile<pcl::PointXYZ>("sparse_data.pcd", *spsourceCloud) != 0)
// 	{
// 		return -1;
// 	}

// 	//	Transformation matrix object, initialized to the identity matrix
// 	//	(a null transformation).
// 	Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
// 	Eigen::Matrix4f transformation2 = Eigen::Matrix4f::Identity();

// 	//	Set a rotation around the Z axis.
// 	float theta = M_PI / 15;
// 	transformation(0, 0) = cos(theta);
// 	transformation(0, 1) = -sin(theta);
// 	transformation(1, 0) = sin(theta);
// 	transformation(1, 1) = cos(theta);
 
// 	//	Set a translation on the X axis.
// 	transformation(0, 3) = 0.002f; //meter (positive direction).
// 	transformation(1, 3) = 0.007f;
// 	transformation(2, 3) = 0.008f;
// 	pcl::transformPointCloud(*targetCloud, *tftargetCloud, transformation);
// 	pcl::transformPointCloud(*sourceCloud, *tfspsourceCloud, transformation);

// 	//	ICP object.
// 	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
// 	icp.setInputSource(sourceCloud);
// 	icp.setInputTarget(tftargetCloud);

// 	// Set the max correspondence distance to 0.05cm (e.g., correspondences with higher distances will be ignored)
// 	icp.setMaxCorrespondenceDistance (0.05);
// 	// Set the maximum number of iterations (criterion 1)
// 	//	icp.setMaximumIterations (20000);
// 	// Set the transformation epsilon (criterion 2)
// 	//	icp.setTransformationEpsilon (1e-10);
// 	// Set the euclidean distance difference epsilon (criterion 3)
// 	//	icp.setEuclideanFitnessEpsilon (1);

// 	icp.setRANSACOutlierRejectionThreshold(0.01);

// 	icp.align(*finalCloud);

// 	if (icp.hasConverged())
// 	{
// 		std::cout << "ICP converged." << std::endl
// 				  << "The score is " << icp.getFitnessScore() << std::endl;
// 		std::cout << "Transformation matrix:" << std::endl;
// 		std::cout << icp.getFinalTransformation() << std::endl;
// 		transformation2 = icp.getFinalTransformation();
// 	}
// 	else std::cout << "ICP did not converge." << std::endl;

// 	std::cout << "Real transformation: " << std::endl;
// 	std::cout << transformation << std::endl;


// 	//	Visualize both the original and the test and final pointcloud.
// 	pcl::visualization::PCLVisualizer viewer(argv[1]);
// 	//	The sparse final pointcloud 
// 	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler4(finalCloud,0,255,0);
// 	viewer.addPointCloud(finalCloud, colorHandler4, "Final cloud");

    
// 	// // original cloud
// 	// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler1(sourceCloud, 0, 0, 255);
// 	// viewer.addPointCloud(sourceCloud, colorHandler1, "Transformed cloud");
// 	// //test point on original cloud
// 	// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler2(targetCloud, 255, 0, 0);
// 	// viewer.addPointCloud(targetCloud, colorHandler2, "Transformed target cloud");
// 	//real surface
// 	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler5(tfspsourceCloud,153,51,255);
// 	viewer.addPointCloud(tfspsourceCloud, colorHandler5, "Transformed source cloud");

// 	//target (obtained data)
// 	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler3(tftargetCloud, 255,0,0);
// 	viewer.addPointCloud(tftargetCloud, colorHandler3, "Target cloud");

	
// 	while (!viewer.wasStopped())
// 	{
// 		viewer.spinOnce();
// 	}

	
// }






int
main(int argc, char** argv)
{ // registration test.pcd raw.pcd
	// Objects for storing the point clouds.
	pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr tfsourceCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr tftargetCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr finalCloud(new pcl::PointCloud<pcl::PointXYZ>);
	// Read two PCD files from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *sourceCloud) != 0)
	{
		return -1;
	}
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], *targetCloud) != 0)
	{
		return -1;
	}



	// Transformation matrix object, initialized to the identity matrix
	// (a null transformation).
	Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f transformation2 = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f transformation3 = Eigen::Matrix4f::Identity();
	// Set a rotation around the Z axis.
	float theta = M_PI / 30;
	transformation(0, 0) = cos(theta);
	transformation(0, 1) = -sin(theta);
	transformation(1, 0) = sin(theta);
	transformation(1, 1) = cos(theta);
 
	// Set a translation on the X axis.
	transformation(0, 3) = 0.005f; // meter (positive direction).
	transformation(1, 3) = 0.007f;
	transformation(2, 3) = 0.002f;
	pcl::transformPointCloud(*sourceCloud, *tfsourceCloud, transformation);
	pcl::transformPointCloud(*targetCloud, *tftargetCloud, transformation);	

	// ICP object.
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> registration;
	registration.setInputSource(tfsourceCloud);
	registration.setInputTarget(targetCloud);

	// Set the max correspondence distance to 0.05cm (e.g., correspondences with higher distances will be ignored)
	registration.setMaxCorrespondenceDistance (0.05);
	// Set the maximum number of iterations (criterion 1)
	registration.setMaximumIterations (20000);
	// Set the transformation epsilon (criterion 2)
	//	registration.setTransformationEpsilon (1e-10);
	// Set the euclidean distance difference epsilon (criterion 3)
	//	registration.setEuclideanFitnessEpsilon (1);

	registration.setRANSACOutlierRejectionThreshold(0.001);


	registration.align(*finalCloud);
	if (registration.hasConverged())
	{
		std::cout << "ICP converged." << std::endl
				  << "The score is " << registration.getFitnessScore() << std::endl;
		std::cout << "Transformation matrix:" << std::endl;
		std::cout << registration.getFinalTransformation().inverse() << std::endl;
		transformation2 = registration.getFinalTransformation();
		transformation3 = registration.getFinalTransformation().inverse();
	}
	else std::cout << "ICP did not converge." << std::endl;

	std::cout << "Real transformation: " << std::endl;
	std::cout << transformation << std::endl;
 
	// pcl::transformPointCloud(*tfsourceCloud, *finalCloud, transformation2);
	pcl::transformPointCloud(*targetCloud, *finalCloud, transformation3);



	// Visualize both the original and the test and final pointcloud.
	pcl::visualization::PCLVisualizer viewer(argv[1]);

	//

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler4(tftargetCloud, 0, 0, 255);
	viewer.addPointCloud(tftargetCloud, colorHandler4, "Transformed target cloud");

	// 
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler3(targetCloud, 0,0,255);
	viewer.addPointCloud(targetCloud, colorHandler3, "Target cloud");

	
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler2(finalCloud, 51,255,51);
	viewer.addPointCloud(finalCloud, colorHandler2, "Final cloud");
	
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler0(sourceCloud, 255, 0,0);
	viewer.addPointCloud(sourceCloud,colorHandler0, "source Cloud");
	//
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler1(tfsourceCloud, 255, 0, 0);
	viewer.addPointCloud(tfsourceCloud, colorHandler1, "Transformed cloud");

	// Add 3D colored axes to help see the transformation.
	//	viewer.addCoordinateSystem(1.0, "reference", 0);
 
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}

}
