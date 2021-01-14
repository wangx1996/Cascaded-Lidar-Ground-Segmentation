#include <iostream>
#include <fstream>
#include <vector>
#include <ctime>
#include <stdlib.h>
#include <time.h> 
#include <algorithm>
#include <deque>
#include <unordered_set>
#include <math.h>
//PCL
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h> 

//Eigen
#include <Eigen/Dense>
#include <queue> 

#include <unordered_map>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
using namespace std;

struct PointXYZIR{
PCL_ADD_POINT4D;
float intensity;
uint16_t ring;
EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIR,
(float,x,x)
(float,y,y)
(float,z,z)
(float,intensity,intensity)
(uint16_t,ring,ring)
)

static double pi = 3.1415926;

static int64_t gtm() {
	struct timeval tm;
	gettimeofday(&tm, 0);
	// return ms
	int64_t re = (((int64_t) tm.tv_sec) * 1000 * 1000 + tm.tv_usec);
	return re;
}


template<typename T> string toString(const T& t) {
	ostringstream oss;
	oss << t;
	return oss.str();
}

float Polar_angle_cal(float x, float y) {
	float temp_tangle = 0;
	if (x == 0 && y == 0) {
		temp_tangle = 0;
	} else if (y >= 0) {
		temp_tangle = (float) atan2(y, x);
	} else if (y <= 0) {
		temp_tangle = (float) atan2(y, x) + 2 * pi;
	}
	return temp_tangle;
}

double calRadius(double theta, double alpha, double beta){
	double r = (1.0/(tan(theta)+ tan(beta))) - (1.0/(tan(alpha+theta) + tan(beta)));
	return r;
}
	
Eigen::MatrixXd Radius_table(int model, double height, double slope){

	double step = 0.0;
	double initial_angle = 0.0;

	if(model == 64){
		step = 1.0/3.0;
		initial_angle = -2.0;
	}

	Eigen::MatrixXd rtable(model,2);
	rtable.fill(0.0);
	
	double alpha = step * pi/180;
	double beta = slope * pi/180;

	for(int i=0; i<model; ++i){
		double theta = (i * step + initial_angle);
		if(i==31){
			rtable(i,0) = height * calRadius(theta* pi/180, -alpha, beta);
			if(theta !=0) rtable(i,1)= height/tan(theta* pi/180);
			step = 0.5;
			alpha = step * pi/180;
			initial_angle = -15.0 + 8.83;
		}else{
			rtable(i,0) = height * calRadius(theta* pi/180, alpha, beta);
			if(theta !=0) rtable(i,1) = height/tan(theta* pi/180);
		}
	}

	return rtable;
}

double cal_range(PointXYZIR& p){
	return sqrt(p.x*p.x + p.y*p.y);
}

vector<vector<double>> depth_image(pcl::PointCloud<PointXYZIR>& cloud_in, int model){
	double width = 2083;
	cv::Mat bvimage = cv::Mat::zeros(model, (int)width, CV_8UC1);
	vector<vector<double>> depthimg((int)width, vector<double>(model,-1));
	for(int i=0; i<cloud_in.points.size(); ++i){
		double u = Polar_angle_cal(cloud_in.points[i].x,cloud_in.points[i].y);
		int col = width - (int)(width*(u *180/pi)/360.0) - 1;
		int ind = model - cloud_in.points[i].ring -1;
		depthimg[col][ind] = i;
		bvimage.at<uchar>(ind,col) = (cal_range(cloud_in.points[i])/100)*255;
	}
	cv::Mat image_ColorMap;
	cv::applyColorMap(bvimage, image_ColorMap, cv::COLORMAP_JET);
	cv::resize(image_ColorMap, image_ColorMap, cv::Size(1000, 100));
	//cv::imshow("test",image_ColorMap);
	//cv::waitKey(0);
	return depthimg;
}
	 


void cascased_ground_seg(pcl::PointCloud<PointXYZIR>& cloud_in, int model, double height, double slope){
   	int64_t tm0 = gtm();

	Eigen::MatrixXd rtable = Radius_table(model, height, slope);
	vector<vector<double>> depthimg = depth_image(cloud_in, model);
	double vertical_thresh = 0.08;

	pcl::PointCloud<PointXYZIR>::Ptr ground(
			new pcl::PointCloud<PointXYZIR>);

	pcl::PointCloud<PointXYZIR>::Ptr nground(
			new pcl::PointCloud<PointXYZIR>);


	pcl::PointCloud<PointXYZIR>::Ptr finalground(
			new pcl::PointCloud<PointXYZIR>);

	pcl::PointCloud<PointXYZIR>::Ptr finalnground(
			new pcl::PointCloud<PointXYZIR>);


	//####################################### inner ring segmentaiton #################################
	for(int i=0; i<depthimg.size(); ++i){
		deque<int> ring;
		deque<int> index;
		pcl::PointCloud<PointXYZIR>::Ptr candidate(
			new pcl::PointCloud<PointXYZIR>);

		for(int j=0; j<depthimg[0].size(); ++j){
			if(depthimg[i][j] == -1)
				continue;
			else{
				ring.push_back(j);
				index.push_back(depthimg[i][j]);
			}
		}

		PointXYZIR p0 = cloud_in.points[index[0]];
		double zmax = p0.z;
		double zmin = p0.z;
		candidate->points.push_back(cloud_in.points[index[0]]);
		//cout<<"########################### "<<index.size()<<" "<<rtable<<"\n "<<endl;
		for(int id=0; id<index.size(); ++id){
			if(id != (index.size()-1)){

				double thresh = 0.0;
				for(int ti = ring[id]; ti < ring[id+1]; ++ti){
					thresh += rtable(ti,0);	
				}

				double range = cal_range(cloud_in.points[index[id]]);
				double rangen = cal_range(cloud_in.points[index[id+1]]);
				if(fabs(rangen - range) < thresh){
					candidate->points.push_back(cloud_in.points[index[id+1]]);
					if(cloud_in.points[index[id+1]].z > zmax)
						zmax = cloud_in.points[index[id+1]].z;
					if(cloud_in.points[index[id+1]].z < zmin)
						zmin = cloud_in.points[index[id+1]].z;
				}else{
					if(candidate->points.size()>1 && (zmax - zmin) > vertical_thresh){
						*nground += *candidate;
						candidate->points.clear();
						candidate->points.swap(candidate->points);
					}else{
						*ground += *candidate;
						candidate->points.clear();
						candidate->points.swap(candidate->points);
					}
					candidate->points.push_back(cloud_in.points[index[id+1]]);
					zmax = cloud_in.points[index[id+1]].z;
					zmin = cloud_in.points[index[id+1]].z;			
				}
			}else{

				if(candidate->points.size()>1 && (zmax - zmin) > vertical_thresh){
					*nground += *candidate;
					candidate->points.clear();
					candidate->points.swap(candidate->points);
				}else{
					*ground += *candidate;
					candidate->points.clear();
					candidate->points.swap(candidate->points);
				}
			}

		}

	}
	//################################### multi plane segmeantation #################################### 
	
	int section = 4;
	int width = 2083;
	double seg_section = (int)(ceil((2083.0)/8.0));
	vector<double> angle_boundary = {width -seg_section, seg_section, 3*seg_section, 5*seg_section, width - seg_section};
	
	int ring_section = model/4;
	vector<double> ring_boundary = {0, rtable(63-ring_section,1),rtable(63- 2*ring_section,1), rtable(63-3*ring_section,1), rtable(7,1)};	


	vector<vector<double>> uint = {{1,0},{0,-1},{-1,0},{0,1}};
	vector<vector<int>> plane_section(4);

	for(int i=0; i < ground->points.size(); ++i){
		double u = Polar_angle_cal(ground->points[i].x,ground->points[i].y);
		int col = width - (int)(width*(u *180/pi)/360.0) - 1;
		if(col< angle_boundary[1]){
			plane_section[0].push_back(i);

		}else if(col >= angle_boundary[1] && col<angle_boundary[2]){
			plane_section[1].push_back(i);

		}else if(col >= angle_boundary[2] && col<angle_boundary[3]){
			plane_section[2].push_back(i);
		
		}else if(col >= angle_boundary[3]){
			plane_section[3].push_back(i);
	
		}
	}


	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
			new pcl::visualization::PCLVisualizer("pcd")); //PCLVisualizer 可视化类
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1);
        cv::RNG rng(12345);

	for(int i=0; i<4; ++i){
		vector<vector<int>> ratio_section(4);
		vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> section_point;
		vector<pcl::PointCloud<PointXYZIR>::Ptr> section_point_origin;
		for(int k=0; k<4; ++k){
			pcl::PointCloud<pcl::PointXYZ>::Ptr x(new pcl::PointCloud<pcl::PointXYZ>);
			section_point.push_back(x);
			pcl::PointCloud<PointXYZIR>::Ptr xt(new pcl::PointCloud<PointXYZIR>);
			section_point_origin.push_back(xt);
		}
		for(int j=0; j<plane_section[i].size(); ++j){
			double u = cal_range(ground->points[plane_section[i][j]]);
			if(u >= ring_boundary[0] && u<ring_boundary[1]){
				pcl::PointXYZ p;
				p.x = ground->points[plane_section[i][j]].x;
				p.y = ground->points[plane_section[i][j]].y;
				p.z = ground->points[plane_section[i][j]].z;
				section_point[0]->points.push_back(p);
	
				PointXYZIR po;
				po = ground->points[plane_section[i][j]];
				section_point_origin[0]->points.push_back(po);

			}else if(u >= ring_boundary[1] && u<ring_boundary[2]){
				pcl::PointXYZ p;
				p.x = ground->points[plane_section[i][j]].x;
				p.y = ground->points[plane_section[i][j]].y;
				p.z = ground->points[plane_section[i][j]].z;
				section_point[1]->points.push_back(p);

				PointXYZIR po;
				po = ground->points[plane_section[i][j]];
				section_point_origin[1]->points.push_back(po);

			}else if(u >= ring_boundary[2] && u<ring_boundary[3]){
				pcl::PointXYZ p;
				p.x = ground->points[plane_section[i][j]].x;
				p.y = ground->points[plane_section[i][j]].y;
				p.z = ground->points[plane_section[i][j]].z;
				section_point[2]->points.push_back(p);

				PointXYZIR po;
				po = ground->points[plane_section[i][j]];
				section_point_origin[2]->points.push_back(po);
		
			}else if(u >= ring_boundary[3] && u<ring_boundary[4]){
				pcl::PointXYZ p;
				p.x = ground->points[plane_section[i][j]].x;
				p.y = ground->points[plane_section[i][j]].y;
				p.z = ground->points[plane_section[i][j]].z;
				section_point[3]->points.push_back(p);

				PointXYZIR po;
				po = ground->points[plane_section[i][j]];
				section_point_origin[3]->points.push_back(po);
	
			}
		}

		for(int ci =0; ci<4; ++ci){
			int r = rng.uniform(0, 255) ;
			int b = rng.uniform(0, 255) ;
			int g = rng.uniform(0, 255) ;
			pcl::visualization::PointCloudColorHandlerCustom < pcl::PointXYZ> color2(section_point[ci], (b), (g),(r));
			viewer->addPointCloud(section_point[ci],color2, "cloud" + toString(ci + 4* i));
		}

		pcl::ModelCoefficients::Ptr precoefficients (new pcl::ModelCoefficients);

		//######################### plane fitting ############################
		for(int pid=0; pid<4; ++pid){
			if(section_point[pid]->points.size()>0){
				pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm;  
				//创建法向量计算对象	
				norm.setInputCloud (section_point[pid]);                  //设置输入点云	//
   				pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>());
				norm.setSearchMethod (tree); //设置搜索方法	
				norm.setKSearch(50); //设置半径邻域搜索	
				pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>); //创建法向量点云	
				norm.compute (*normals); //计算法向量

         			pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
          			pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
          			// Create the segmentation object
          			//pcl::SACSegmentation<pcl::PointXYZ> seg;
       				pcl::SACSegmentationFromNormals<pcl::PointXYZ,pcl::Normal> seg;    //分割对象

          			// Optional
         	 		seg.setOptimizeCoefficients (true);
          			// Mandatory
          			seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
          			seg.setMethodType (pcl::SAC_RANSAC);
          			seg.setDistanceThreshold (0.3);
    				seg.setNormalDistanceWeight(0.5);
    				seg.setMaxIterations(100);
         			seg.setInputCloud (section_point[pid]);
    				seg.setInputNormals(normals);

          			seg.segment (*inliers, *coefficients);
				pcl::PointCloud<PointXYZIR>::Ptr ground_temp(
					new pcl::PointCloud<PointXYZIR>);
				pcl::PointCloud<PointXYZIR>::Ptr nground_temp(
					new pcl::PointCloud<PointXYZIR>);

				unordered_set<int> hash;
				for (size_t j = 0; j < inliers->indices.size (); ++j){
					hash.insert(inliers->indices[j]);
				}

				if(pid==0){
					precoefficients = coefficients;
					for(int li =0; li<section_point_origin[pid]->points.size(); ++li){
						if(hash.count(li)){
							finalground->points.push_back(section_point_origin[pid]->points[li]);
						}else{
							finalnground->points.push_back(section_point_origin[pid]->points[li]);
						}
					}
				}else{
					
					double x = ring_boundary[pid] * uint[i][0];
					double y = ring_boundary[pid] * uint[i][1];
					Eigen::VectorXd pre(2);
					Eigen::VectorXd now(2);
					pre<<precoefficients->values[0],precoefficients->values[1];
					now<<coefficients->values[0],coefficients->values[1];
					double angle_diff = acos(pre.dot(now));

					Eigen::VectorXd preh(4);
					Eigen::VectorXd nowh(4);
					preh<<precoefficients->values[0],precoefficients->values[1],precoefficients->values[2],precoefficients->values[3];
					nowh<<coefficients->values[0],coefficients->values[1],coefficients->values[2],coefficients->values[3];


					Eigen::VectorXd temp = nowh/nowh(2) - preh/preh(2);
					double height_diff = fabs(temp(0) * x + temp(1) * y + temp(3));

					if(height_diff >=  0.3 || angle_diff >= 5*pi/180){
						for(int li =0; li<section_point_origin[pid]->points.size(); ++li){
							if(hash.count(li)){
								finalground->points.push_back(section_point_origin[pid]->points[li]);
							}else{
								finalnground->points.push_back(section_point_origin[pid]->points[li]);
							}
							precoefficients = coefficients;
						}
					}else{
						for(int li =0; li<section_point_origin[pid]->points.size(); ++li){
							double px(0),py(0),pz(0);
							px = section_point_origin[pid]->points[li].x;
							py = section_point_origin[pid]->points[li].y;
							pz = section_point_origin[pid]->points[li].z;
							double distance = fabs(preh(0) * px + preh(1)*py + preh(2)*pz + \
										preh(3))/sqrt(preh(0)*preh(0) +preh(1)*preh(1)+preh(2)*preh(2));
							if(distance < 0.3){
								finalground->points.push_back(section_point_origin[pid]->points[li]);
							}else{
								finalnground->points.push_back(section_point_origin[pid]->points[li]);
							}

						}
					}
				}

			}
				
				
		}

	}


	pcl::PointCloud<pcl::PointXYZI>::Ptr show(
			new pcl::PointCloud<pcl::PointXYZI>);

	for(int si = 0; si<finalground->points.size(); ++si){
		pcl::PointXYZI p;
		p.x = finalground->points[si].x;
		p.y = finalground->points[si].y;
		p.z = finalground->points[si].z;
		p.intensity = finalground->points[si].intensity;
		show->points.push_back(p);
	}


	pcl::PointCloud<pcl::PointXYZI>::Ptr showob(
			new pcl::PointCloud<pcl::PointXYZI>);

	for(int si = 0; si<finalnground->points.size(); ++si){
		pcl::PointXYZI p;
		p.x = finalnground->points[si].x;
		p.y = finalnground->points[si].y;
		p.z = finalnground->points[si].z;
		p.intensity = finalnground->points[si].intensity;
		showob->points.push_back(p);
	}

	*finalnground = *finalnground + *nground;
   	int64_t tm1 = gtm();
  	printf("[INFO]seg cast time:%ld us\n",  tm1-tm0);

	pcl::visualization::PointCloudColorHandlerCustom < pcl::PointXYZI> color2(show, (255), (0),(0));
	string cs = "cloudfinal";
	//viewer->addPointCloud(show,color2, cs);

	pcl::visualization::PointCloudColorHandlerCustom < pcl::PointXYZI> colorob(showob, (0), (255),(0));
	string csob = "cloudfinalob";
	//viewer->addPointCloud(showob,colorob, csob);



	while (!viewer->wasStopped()) {
		viewer->spin();
	}

}
	

int main(int argc, char** argv){
	int model = 64;
	double height = 2.0;
	double slope = 10.0;
    	pcl::PointCloud<PointXYZIR>::Ptr cloud(new pcl::PointCloud<PointXYZIR>);//初始化点云 
    	pcl::io::loadPCDFile<PointXYZIR>(argv[1], *cloud);//加载pcd点云并放入cloud中
    	cloud->height = 1;
    	cloud->width = cloud->points.size();
    	cloud->is_dense = false;//最终优化结果
	cascased_ground_seg(*cloud, model,height, slope);

	return 0;
}
	
		
