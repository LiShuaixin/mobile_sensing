#include <iostream>
#include <math.h>
#include <vector>
#include <time.h>
#include <string>
#include <stdio.h>  
#include <stdlib.h>  

#include <pcl/point_cloud.h>                        
#include <pcl/filters/voxel_grid.h>            
#include <pcl/filters/filter.h>      
#include <pcl/kdtree/kdtree_flann.h> // 此头文件不能置于opencv之后
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/point_types.h>
#include <pcl/point_representation.h>
#include <pcl/registration/transforms.h> 
#include <pcl/io/pcd_io.h>                    
#include <pcl/io/ply_io.h>
#include <pcl/registration/icp.h>           
#include <pcl/registration/icp_nl.h>  
#include <pcl/registration/gicp.h>


#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include "Thirdparty/g2o/g2o/core/base_vertex.h"
#include "Thirdparty/g2o/g2o/core/base_unary_edge.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include <pangolin/pangolin.h>

#include <chrono>

#include "mcgicpIntensity.h"

using namespace std;
using namespace cv;

// 找两帧点云的最邻近匹配点
void find_correspondence_intensity(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr source, 
    const pcl::PointCloud<pcl::PointXYZI>::Ptr target,
    vector<int>& indecis1,
    vector<int>& indecis2
);

void find_correspondence_rgb(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr source, 
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr target,
    vector<int>& indecis1,
    vector<int>& indecis2
);

// pcl库的icp实现
void pcl_icp(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr pts1, 
    const pcl::PointCloud<pcl::PointXYZI>::Ptr pts2,
    Eigen::Matrix4f& T
);

// 基于svd分解的icp实现
void icp_intensity_svd(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr pts1, 
    const pcl::PointCloud<pcl::PointXYZI>::Ptr pts2,
    Mat& R, Mat& t
);

void icp_rgb_svd(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pts1, 
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pts2,
    Mat& R, Mat& t
);

// 基于非线性优化的icp实现
void icp_rgb_ba(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pts1, 
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pts2,
    Mat& R, Mat& t
);

void icp_intensity_ba(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr pts1, 
    const pcl::PointCloud<pcl::PointXYZI>::Ptr pts2,
    Mat& R, Mat& t
);

// mcgicp实现
void mcgicp_rgb_svd(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pts1, 
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pts2,
    Mat& R, Mat& t
);

void mcgicp_intensity_svd(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr pts1, 
    const pcl::PointCloud<pcl::PointXYZI>::Ptr pts2,
    Mat& R, Mat& t
);

void mcgicp_rgb_ba(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pts1, 
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pts2,
    Mat& R, Mat& t
);

void mcgicp_intensity_ba(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr pts1, 
    const pcl::PointCloud<pcl::PointXYZI>::Ptr pts2,
    Mat& R, Mat& t
);

// 读取velodyne的二进制格式数据
void load_velodyne( int argc, char** argv, std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& vpts );

// 点云可视化
void showPointCloud(const string &window_name, const pcl::PointCloud<pcl::PointXYZI>::Ptr src, const pcl::PointCloud<pcl::PointXYZI>::Ptr tgt);
void showPointCloud(const string &window_name, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr src, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt);

// g2o edge
class EdgeProjectXYZRGBDPoseOnly : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeProjectXYZRGBDPoseOnly( const Eigen::Vector3d& point ) : _point(point) {}

    virtual void computeError()
    {
        const g2o::VertexSE3Expmap* pose = static_cast<const g2o::VertexSE3Expmap*> ( _vertices[0] );
        // measurement is p, point is p'
        _error = _measurement - pose->estimate().map( _point );
    }
    
    virtual void linearizeOplus()
    {
        g2o::VertexSE3Expmap* pose = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
        g2o::SE3Quat T(pose->estimate());
        Eigen::Vector3d xyz_trans = T.map(_point);
        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double z = xyz_trans[2];
        
        _jacobianOplusXi(0,0) = 0;
        _jacobianOplusXi(0,1) = -z;
        _jacobianOplusXi(0,2) = y;
        _jacobianOplusXi(0,3) = -1;
        _jacobianOplusXi(0,4) = 0;
        _jacobianOplusXi(0,5) = 0;
        
        _jacobianOplusXi(1,0) = z;
        _jacobianOplusXi(1,1) = 0;
        _jacobianOplusXi(1,2) = -x;
        _jacobianOplusXi(1,3) = 0;
        _jacobianOplusXi(1,4) = -1;
        _jacobianOplusXi(1,5) = 0;
        
        _jacobianOplusXi(2,0) = -y;
        _jacobianOplusXi(2,1) = x;
        _jacobianOplusXi(2,2) = 0;
        _jacobianOplusXi(2,3) = 0;
        _jacobianOplusXi(2,4) = 0;
        _jacobianOplusXi(2,5) = -1;
    }

    bool read ( istream& in ) {}
    bool write ( ostream& out ) const {}
protected:
    Eigen::Vector3d _point;
};

int main ( int argc, char** argv )
{   
    pcl::PointCloud<pcl::PointXYZI>::Ptr pts1(new pcl::PointCloud<pcl::PointXYZI>()), pts2(new pcl::PointCloud<pcl::PointXYZI>());
    std::vector< pcl::PointCloud<pcl::PointXYZI>::Ptr > vpts;
    
    load_velodyne(argc, argv, vpts);
    
    if(vpts.empty())
    {
        cout<<"load data failed!"<<endl;
        return 1;
    }
    
    pts1 = vpts[0]; pts2 = vpts[1];
    
    showPointCloud("before alignment", pts1, pts2);

    cv::Mat R = cv::Mat::eye(3,3,CV_32F), t = cv::Mat::zeros(3,1,CV_32F);
    
    // icp_intensity_svd ( pts1, pts2, R, t );  
    // icp_intensity_ba( pts1, pts2, R, t );    
    
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    pcl_icp(pts1, pts2, T);
    
    /*
     * Eigen::Matrix3f R_eigen; Eigen::Vector3f t_eigen;
    cv::cv2eigen(R, R_eigen); cv::cv2eigen(t, t_eigen);
    T.topLeftCorner<3,3>() = R_eigen; T.topRightCorner<3,1>() = t_eigen;
    std::cout << "T_final = \n" << T << std::endl;
    */
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>()); 
    pcl::transformPointCloud(*pts2, *tmp, T);  
    
    showPointCloud("after alignment", pts1, tmp);
}

void find_correspondence_intensity(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr source, 
    const pcl::PointCloud<pcl::PointXYZI>::Ptr target,
    vector<int>& indecis1,
    vector<int>& indecis2
)
{     
    int N1 = source->size(), N2 = target->size();
    int K = source->size() < target->size()? source->size() : target->size(); // 取点云个数小的量
    
    std::vector<int> indecies; indecies.reserve (K);
    std::vector<float> dist_sq; dist_sq.reserve (K);
    
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    
    // 最近邻对应点搜索
    if(N1 < N2)
    {
        std::vector<int> nn_indecies; nn_indecies.reserve (1);
        std::vector<float> nn_dist_sq; nn_dist_sq.reserve (1);
        kdtree->setInputCloud(target);
	
	pcl::PointCloud<pcl::PointXYZI>::const_iterator points_iterator = source->begin ();
	
	for(int i = 0; points_iterator != source->end(); ++points_iterator, ++i)
	{
	    const pcl::PointXYZI &query_point = *points_iterator;
	    
	    kdtree->nearestKSearch(query_point, 1, nn_indecies, nn_dist_sq); // 在target中找到与source点对应的最近邻点
	    
	    if(nn_dist_sq[0] < 2.25)
	    {
		indecies.push_back(nn_indecies[0]);
		dist_sq.push_back(nn_dist_sq[0]);
		indecis1.push_back(i);
		indecis2.push_back(nn_indecies[0]);
	    }
	} 
	
	
    }
    else
    {
        std::vector<int> nn_indecies; nn_indecies.reserve (1);
        std::vector<float> nn_dist_sq; nn_dist_sq.reserve (1);
        kdtree->setInputCloud(source);
	
	pcl::PointCloud<pcl::PointXYZI>::const_iterator points_iterator = target->begin ();
	
	for(int i = 0; points_iterator != target->end(); ++points_iterator, ++i)
	{
	    const pcl::PointXYZI &query_point = *points_iterator;
	    
	    kdtree->nearestKSearch(query_point, 1, nn_indecies, nn_dist_sq); // 在target中找到与source点对应的最近邻点
	    
	    if(nn_dist_sq[0] < 2.25)
	    {
		indecies.push_back(nn_indecies[0]);
		dist_sq.push_back(nn_dist_sq[0]);
		indecis2.push_back(i);
		indecis1.push_back(nn_indecies[0]);
	    }
	}
    }   
    
}

void pcl_icp(
    const pcl::PointCloud< pcl::PointXYZI >::Ptr pts1, 
    const pcl::PointCloud< pcl::PointXYZI >::Ptr pts2, 
    Eigen::Matrix4f& T
)
{
    // PCL的ICP
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity();
    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> reg;
    // pcl::IterativeClosestPointNonLinear<pcl::PointXYZI, pcl::PointXYZI> reg;
    // pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> reg;
    // pcl::GeneralizedIterativeClosestPoint4D reg(0.5, 120);

    // 设置收敛判断条件，越小精度越高，收敛速度越慢
    reg.setTransformationEpsilon(0.001);
    
    // 设置临近点判断距离,距离为m
    double d = 1.5;

    reg.setMaxCorrespondenceDistance(d);
    
    reg.setInputSource(pts2);
    reg.setInputTarget(pts1);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr reg_result = pts2;
    Eigen::Matrix4f prev;
    
    reg.setMaximumIterations(2);
    
    for(int i = 0; i < 25; i++)
    {
	*pts2 = *reg_result;
	
	reg.setInputSource(pts2);
	reg.align(*reg_result);
	
	// 旋转矩阵累计
	Ti = reg.getFinalTransformation() * Ti;
	//如果转换矩阵增量的变换量小于阈值(说明配准精度较高)，则缩小correspondence距离(追求更高精度)
	if( fabs((reg.getLastIncrementalTransformation()-prev).sum())<reg.getTransformationEpsilon() )
	    reg.setMaxCorrespondenceDistance(0.8 * reg.getMaxCorrespondenceDistance());
	
	// 将prev设置为上一时刻坐标变换增量
	prev = reg.getLastIncrementalTransformation(); 
    }
    
    T = Ti;
    std::cout << "T_final = \n" << T << std::endl;
}


// T is the transform from C2 to C1
// R and t are rotation and translation matrix that transform pts2 to pts1
void icp_intensity_svd (
    const pcl::PointCloud<pcl::PointXYZI>::Ptr pts1, 
    const pcl::PointCloud<pcl::PointXYZI>::Ptr pts2,
    Mat& R, Mat& t
)
{
    // vector< Eigen::Matrix4d > T_intermediate(100);
    Eigen::Matrix4d T_ = Eigen::Matrix4d::Identity();
    
    double mse = 100000, mse_pre = 100000;
    int iter = 0;
    bool not_convergence = true;
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp = pts2;
    
    while(not_convergence)
    {	
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
	Eigen::Matrix3d R_eigen; Eigen::Vector3d t_eigen;
	cv::cv2eigen(R, R_eigen); cv::cv2eigen(t, t_eigen);
	T.topLeftCorner<3,3>() = R_eigen; T.topRightCorner<3,1>() = t_eigen;		
		
	pcl::transformPointCloud(*tmp, *tmp, T); // 根据给定的变换矩阵将点云统一至同一坐标系下
	
	vector<int> indecis1, indecis2;
	find_correspondence_intensity(pts1, tmp, indecis1, indecis2 );
	
	int N = indecis1.size();
	cerr << N << " correspondences have been found." << endl;
	if( N != indecis2.size() )
	    std::cout << " the size of indecis1 is not equal to the size of indecis2. " << std::endl;
	
	Eigen::Vector3d p1,p2; // center of point cloud
	for(int i = 0; i< N; i++)
	{
	    p1[0] += pts1->points[indecis1[i]].x;
	    p1[1] += pts1->points[indecis1[i]].y;
	    p1[2] += pts1->points[indecis1[i]].z;
	    
	    p2[0] += tmp->points[indecis2[i]].x;
	    p2[1] += tmp->points[indecis2[i]].y;
	    p2[2] += tmp->points[indecis2[i]].z;
	}
	
	p1 /= static_cast<double>(N); 
	p2 /= static_cast<double>(N);

	vector<Eigen::Vector3d> q1(N), q2(N); // remove the center
	
	for ( int i=0; i<N; i++ )
	{
	    Eigen::Vector3d pt1(pts1->points[indecis1[i]].x, pts1->points[indecis1[i]].y, pts1->points[indecis1[i]].z);
	    q1[i] = pt1 - p1;
	    
	    Eigen::Vector3d pt2(tmp->points[indecis2[i]].x, tmp->points[indecis2[i]].y, tmp->points[indecis2[i]].z);
	    q2[i] = pt2 - p2;
	}
	
	// compute q1*q2^T
	Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
	for ( int i=0; i<N; i++ )
	{
	    W += Eigen::Vector3d ( q1[i][0], q1[i][1], q1[i][2] ) * 
		Eigen::Vector3d ( q2[i][0], q2[i][1], q2[i][2] ).transpose();
	}
	// cout<<"W="<<W<<endl;

	// SVD on W
	Eigen::JacobiSVD<Eigen::Matrix3d> svd ( W, Eigen::ComputeFullU|Eigen::ComputeFullV );
	Eigen::Matrix3d U = svd.matrixU();
	Eigen::Matrix3d V = svd.matrixV();
	// cout<<"U="<<U<<endl;
	// cout<<"V="<<V<<endl;

	Eigen::Matrix3d R_ = U* ( V.transpose() );
	Eigen::Vector3d t_ = Eigen::Vector3d ( p1[0], p1[1], p1[2] ) - R_ * Eigen::Vector3d ( p2[0], p2[1], p2[2] );

	// 计算收敛条件
	double cos_angle = 0.5 * (R_(0, 0) + R_(1, 1) + R_(2, 2) - 1);
        double translation_sqr = t_(0) * t_(0) + t_(1) * t_(1) + t_(2) * t_(2);
	
	for( int i = 0; i<N; i++ )
	{
	    mse = 0;
	    mse += ( Eigen::Vector3d(pts1->points[indecis1[i]].x, pts1->points[indecis1[i]].y, pts1->points[indecis1[i]].z) -
		    (R_ * Eigen::Vector3d(tmp->points[indecis2[i]].x, tmp->points[indecis2[i]].y, tmp->points[indecis2[i]].z) + t_) ).norm();
	}
	mse /= N;
	// std::cout << "mse = " << mse << std::endl;
	
	if( iter>100 || (cos_angle >= 1-1e-6 && translation_sqr <= 1e-6) || 
	    fabs(mse - mse_pre) < 1e-12 || fabs(mse - mse_pre)/mse_pre < 1e-5)
	    not_convergence = false;
	
	mse_pre = mse;
	
	// 累加变换矩阵
	T.topLeftCorner<3,3>() = R_; T.topRightCorner<3,1>() = t_;
	T_ = T * T_;
	
	cerr << "T_cur = " << T_ << endl;
	// convert to cv::Mat
	R = ( Mat_<double> ( 3,3 ) <<
	      R_ ( 0,0 ), R_ ( 0,1 ), R_ ( 0,2 ),
	      R_ ( 1,0 ), R_ ( 1,1 ), R_ ( 1,2 ),
	      R_ ( 2,0 ), R_ ( 2,1 ), R_ ( 2,2 )
	    );
	t = ( Mat_<double> ( 3,1 ) << t_ ( 0,0 ), t_ ( 1,0 ), t_ ( 2,0 ) );	
	
	iter++;
    }   
    
    // convert to cv::Mat
    R = ( Mat_<double> ( 3,3 ) <<
	  T_ ( 0,0 ), T_ ( 0,1 ), T_ ( 0,2 ),
	  T_ ( 1,0 ), T_ ( 1,1 ), T_ ( 1,2 ),
	  T_ ( 2,0 ), T_ ( 2,1 ), T_ ( 2,2 )
	);
    t = ( Mat_<double> ( 3,1 ) << T_ ( 0,3 ), T_ ( 1,3 ), T_ ( 2,3 ) );
    
    // verify p1 = R*p2 + t
    /*for ( int i=0; i<N; i++ )
    {
	cout<<"p1 = "<<pts1->points[indecis1[i]]<<endl;
	cout<<"p2 = "<<pts2->points[indecis2[i]]<<endl;
	cout<<"(R*p2+t) = "<< 
	    R * (Mat_<double>(3,1)<<pts2->points[indecis2[i]].x, pts2->points[indecis2[i]].y, pts2->points[indecis2[i]].z) + t
	    <<endl;
	cout<<endl;
    } */
}

void icp_intensity_ba(
    const pcl::PointCloud< pcl::PointXYZI >::Ptr pts1, 
    const pcl::PointCloud< pcl::PointXYZI >::Ptr pts2, 
    Mat& R, Mat& t
)
{   
    // 设置初值
    Eigen::Matrix4d T_cur = Eigen::Matrix4d::Identity(), T_last = Eigen::Matrix4d::Identity(), T_incre = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d R_eigen; Eigen::Vector3d t_eigen;
    cv::cv2eigen(R, R_eigen); cv::cv2eigen(t, t_eigen);
    cv::cv2eigen(R, R_eigen); cv::cv2eigen(t, t_eigen);
    T_cur.topLeftCorner<3,3>() = R_eigen; T_cur.topRightCorner<3,1>() = t_eigen;
    
    // 根据初值变换点云  
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>()); // 用于找临近点
    pcl::transformPointCloud(*pts2, *tmp, T_cur); 
		     
    double mse = 100000, mse_pre = 100000;
    int iter = 0;
    bool not_convergence = true;
    
    // 主循环
    while(not_convergence)
    {   
	// 搜索匹配点
	vector<int> indecis1, indecis2;
	find_correspondence_intensity(pts1, tmp, indecis1, indecis2 );
	
	int N = indecis1.size();
	cerr <<  N << " correspondences have been found" << endl;
	if( N != indecis2.size() )
	    std::cout << " the size of indecis1 is not equal to the size of indecis2. " << std::endl;
	
	// G2O的准备
	// 线性方程求解器linearSolver->矩阵块求解器solver_ptr->设置算法L-M->稀疏求解器optimizer
	typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,3> > Block;
	Block::LinearSolverType* linearSolver = new g2o::LinearSolverEigen<Block::PoseMatrixType>();
	Block* solver_ptr = new Block(linearSolver);
	g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( solver_ptr );
	g2o::SparseOptimizer optimizer;
	optimizer.setAlgorithm(solver);
	
	// 设置节点	
	g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
	pose->setId(0);
	pose->setEstimate( g2o::SE3Quat(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()) );
	optimizer.addVertex(pose);
	
	// 设置边
	int index = 1;
	vector<EdgeProjectXYZRGBDPoseOnly*> edges;
	for(size_t i = 0; i < indecis1.size(); ++i)
	{
	    EdgeProjectXYZRGBDPoseOnly* edge = new EdgeProjectXYZRGBDPoseOnly( 
	        Eigen::Vector3d(tmp->points[indecis2[i]].x, tmp->points[indecis2[i]].y, tmp->points[indecis2[i]].z) );
	    edge->setId(index);
	    edge->setVertex(0, dynamic_cast<g2o::VertexSE3Expmap*> (optimizer.vertex(0)));
	    edge->setMeasurement( Eigen::Vector3d(pts1->points[indecis1[i]].x, pts1->points[indecis1[i]].y, pts1->points[indecis1[i]].z) );
	    edge->setInformation( Eigen::Matrix3d::Identity()*1e4 );
	    edge->setRobustKernel( new g2o::RobustKernelHuber() );
	    optimizer.addEdge(edge);
	    edges.push_back(edge);
	    index++;
	}
	
	// 优化计算
	optimizer.setVerbose( true );
	optimizer.initializeOptimization();
	optimizer.optimize(5);
	
	// 将优化计算得到的变换矩阵转为旋转矩阵和平移向量	
	g2o::VertexSE3Expmap* v = dynamic_cast<g2o::VertexSE3Expmap*>( optimizer.vertex(0) );
	g2o::SE3Quat T = v->estimate();
	Eigen::Matrix3d R_ = T.rotation().toRotationMatrix();
	Eigen::Vector3d t_ = T.translation();
	
	T_last = T_cur;
	T_incre.topLeftCorner<3,3>() = R_; T_incre.topRightCorner<3,1>() = t_;
	
	pcl::transformPointCloud(*tmp, *tmp, T_incre);
	
	T_cur = T_incre * T_cur;
	
	// 计算收敛条件
	double cos_angle = 0.5 * (R_(0, 0) + R_(1, 1) + R_(2, 2) - 1);
        double translation_sqr = t_(0) * t_(0) + t_(1) * t_(1) + t_(2) * t_(2);
	
	for( int i = 0; i<N; i++ )
	{
	    mse = 0;
	    mse += ( Eigen::Vector3d(pts1->points[indecis1[i]].x, pts1->points[indecis1[i]].y, pts1->points[indecis1[i]].z) -
		    (R_ * Eigen::Vector3d(tmp->points[indecis2[i]].x, tmp->points[indecis2[i]].y, tmp->points[indecis2[i]].z) + t_) ).norm();
	}
	mse /= N;
	std::cout << "mse = " << mse << std::endl;

	if( iter>100 || (cos_angle >= 1-1e-8 && translation_sqr <= 1e-8) || fabs(mse - mse_pre) < 1e-12 || fabs(mse - mse_pre)/mse_pre < 1e-5 )
	    not_convergence = false;
	
	mse_pre = mse;
	
	cerr << "T_cur = \n" << T_cur << endl;	
	
	iter++;
    }
    
    // convert to cv::Mat
    R = ( Mat_<double> ( 3,3 ) <<
	  T_cur ( 0,0 ), T_cur ( 0,1 ), T_cur ( 0,2 ),
	  T_cur ( 1,0 ), T_cur ( 1,1 ), T_cur ( 1,2 ),
	  T_cur ( 2,0 ), T_cur ( 2,1 ), T_cur ( 2,2 )
	);
    t = ( Mat_<double> ( 3,1 ) << T_cur ( 0,3 ), T_cur ( 1,3 ), T_cur ( 2,3 ) );
}

void load_velodyne(int argc, char** argv, std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& vpts)
{
    if ( argc != 3 )
    {
        cout<<"usage: pose_estimation_3d3d pts1 pts2"<<endl;
        return;
    }
    
    for(int i = 1; i<argc; i++)
    {
        FILE *pf;
	
        /* allocate 4 MB buffer */
	int32_t numPoints = 1000000; // the number of point clouds
	float *data = (float*)malloc(numPoints * sizeof(float));
	
	/* pointers */
	float *px = data + 0; // point to the first entry of data
	float *py = data + 1;
	float *pz = data + 2;
	float *pr = data + 3;
	
	// cerr << "file name = " << argv[i] << endl;
	std::string fn(argv[i]);
	if((pf = fopen(fn.c_str(), "rb")) == NULL )
	    std::cout << "Load binary file failed!" << std::endl;
	
	numPoints = fread(data, sizeof(float), numPoints, pf)/4; // fread从pf数据流中读取numPoints个数据，每个数据sizeof(float)个字节， 返回实际读到的项个数
	// cerr << "number of points in binary file = " << numPoints << endl;
	// cerr << "begin to add points into the vector..." << endl;
	
	pcl::PointCloud<pcl::PointXYZI>::Ptr pts(new pcl::PointCloud<pcl::PointXYZI>());
	
	for(int32_t i = 0; i < numPoints; i++)
	{
	    pcl::PointXYZI point;
	    point.x = *px;
	    point.y = *py;
	    point.z = *pz;
	    point.intensity = *pr;
	    
	    //从点云中移除NAN点也就是无效点
	    std::vector<int> indices;
	    pcl::removeNaNFromPointCloud(*pts,*pts, indices);
	    
	    pts->points.push_back(point);
				
	    px += 4;
	    py += 4;
	    pz += 4;
	    pr += 4;
	}
	std::fclose(pf);
	
	vpts.push_back(pts);
	// cerr << "size of vpts = " << vpts.size() << endl;
    }
}


void showPointCloud(const string &window_name, const pcl::PointCloud<pcl::PointXYZI>::Ptr src, const pcl::PointCloud<pcl::PointXYZI>::Ptr tgt ) 
{

    if (src->empty() || tgt->empty()) 
    {
        cerr << "Point cloud is empty!" << endl;
        return;
    }

    pangolin::CreateWindowAndBind(window_name, 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt( 20,20,50, 0,0,0, pangolin::AxisZ )
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glPointSize(2);
        glBegin(GL_POINTS);
	
	pcl::PointCloud<pcl::PointXYZI>::const_iterator points_iterator = src->begin ();
	
	for( ; points_iterator != src->end(); ++points_iterator )
	{
	    const pcl::PointXYZI &p = *points_iterator;
	    
	    glColor3f(0.0, 1.0, 0.0);
	    glVertex3d(p.x, p.y, p.z);
	}
	
	points_iterator = tgt->begin ();
	
	for( ; points_iterator != tgt->end(); ++points_iterator )
	{
	    const pcl::PointXYZI &p = *points_iterator;
	    
	    glColor3f(1.0, 0.0, 0.0);
	    glVertex3d(p.x, p.y, p.z);
	}
        
        glEnd();
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
    return;
}

void showPointCloud(const string &window_name, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr src, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt ) 
{

    if (src->empty() || tgt->empty()) 
    {
        cerr << "Point cloud is empty!" << endl;
        return;
    }

    pangolin::CreateWindowAndBind(window_name, 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glPointSize(2);
        glBegin(GL_POINTS);
	
	pcl::PointCloud<pcl::PointXYZRGB>::const_iterator points_iterator = src->begin ();
	
	for( ; points_iterator != src->end(); ++points_iterator )
	{
	    const pcl::PointXYZRGB &p = *points_iterator;
	    
	    glColor3f(p.r, p.g, p.b);
	    glVertex3d(p.x, p.y, p.z);
	}
	
	points_iterator = tgt->begin ();
	
	for( ; points_iterator != tgt->end(); ++points_iterator )
	{
	    const pcl::PointXYZRGB &p = *points_iterator;
	    
	    glColor3f(p.r, p.g, p.b);
	    glVertex3d(p.x, p.y, p.z);
	}

        glEnd();
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
    return;
}