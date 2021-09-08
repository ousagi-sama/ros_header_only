/******************************************
 * VoxelGridの実装
 * 
 * version 1.00              by ousagi-sama
 ******************************************/
#ifndef H_ROS_PCL_VOXEL_GRID
#define H_ROS_PCL_VOXEL_GRID

/* PCLを利用するためのヘッダ */
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/* ダウンサンプリングに必要 */
#include <pcl/filters/voxel_grid.h>

/**
 * 空間をボクセルで区切り、重心のみ残すダウンサンプリング
 * [例] cloud_ptr = VoxelGrid<pcl::PointXYZ>(cloud_ptr, 0.1, 0.1, 0.1);
 * in_cloud: 対象点群ポインタ
 * x, y, z : ボクセルサイズ
 * 返り値  : ダウンサンプリング後の点群ポインタ
 */
template<class pointT>
typename pcl::PointCloud<pointT>::Ptr VoxelGrid(
    typename pcl::PointCloud<pointT>::Ptr in_cloud, float x, float y, float z){
    
    typename pcl::PointCloud<pointT>::Ptr cloud_filtered (new pcl::PointCloud<pointT>);
    
    pcl::VoxelGrid<pointT> sor;
    sor.setInputCloud(in_cloud);
    sor.setLeafSize(x, y, z);
    sor.filter (*cloud_filtered);

    return cloud_filtered;
}

#endif
