#include "euclidean_cluster_core.h"

//写个用于对曲率进行排序
bool sortFunCur(const vector<double> &v1, const vector<double> &v2)
{   
    //v_angle,rho,x,y,z,cur
	return v1[5] > v2[5]; 
}

//判断图像中的一个点是否在矩形框内px,py
//矩形框的参数：左下角xy 长宽lw
bool point_in_rect(double p[],double rect[])
{
    double px=p[0],py=p[1];
    double x=rect[0],y=rect[1],l=rect[2],w=rect[3];
    if((px>x)&&(px<x+l)&&(py>y)&&(py<y+w))
    {
        return  true;
    }
    else
    {
        return false;
    }
}

//构造函数
EuClusterCore::EuClusterCore(ros::NodeHandle &nh)
{

    //这里是point_cb,所以之后才执行point_cb
    sub_point_cloud_ = nh.subscribe("/filtered_points_no_ground", 10, &EuClusterCore::point_cb, this);
    // sub_point_cloud_ = nh.subscribe("/low_pt", 10, &EuClusterCore::point_cb, this);

    //try to sub the high points to see the how the result is 
    // sub_point_cloud_ = nh.subscribe("/high_roi_pt", 10, &EuClusterCore::point_cb, this);

    // pub_bounding_boxs_ = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/detected_bounding_boxs", 10);
    //发布信息用
    pub_rs_msgs = nh.advertise<rs_perception::PerceptionListMsg>("/rs_percept_result", 10);
    //形式上一样，先订阅，然后发布
    //markerArrayPub--发布marker序列
    markerArrayPub = nh.advertise<visualization_msgs::MarkerArray>("/SpeedArrowArray", 10);

    markerArrayPub_v2 = nh.advertise<visualization_msgs::MarkerArray>("/SpeedArrowArray_v2", 10);

    gridArrayPub = nh.advertise<visualization_msgs::MarkerArray>("/GridArray", 10);
    markerCboxPub = nh.advertise<visualization_msgs::MarkerArray>("/CboxArray", 10);
    //use the maximal length and width to draw the box
    markerCboxV2Pub = nh.advertise<visualization_msgs::MarkerArray>("/CboxV2Array", 10);
    //kalman bouding box
    kalmanBoxPub = nh.advertise<visualization_msgs::MarkerArray>("/KalmanBoxArray", 10);
    //从相机到雷达的方法
    markerBoxPub = nh.advertise<visualization_msgs::MarkerArray>("/BoxArray", 10);

    //将可通行区域包围盒子改成markerArray
    markerPAboxPub = nh.advertise<visualization_msgs::MarkerArray>("/PABoxArray", 10);
    //创建一个显示编号用的markerArray
    markerIndexPub = nh.advertise<visualization_msgs::MarkerArray>("/IndexArray", 10);
    //发布边缘点
    pub_edge_points = nh.advertise<sensor_msgs::PointCloud2>("/edge_points", 10);
    //for pub box edge points
    pub_box_edge_pts = nh.advertise<sensor_msgs::PointCloud2>("/box_edge_pt", 10);
    //pub all points xy
    pub_all_pts_xy   = nh.advertise<sensor_msgs::PointCloud2>("/all_pts_xy", 10); 

    nh.getParam("work_mode", work_mode_);
    ROS_INFO("work_mode: %f", work_mode_);
    switch(int(work_mode_))
    {
        case 0:
        {
            cout << "跟踪匹配估计模式" << endl;
            break;
        }
        case 1:
        {
            cout << "朱哥模式" << endl;
            break;
        }
        case 2:
        {
            cout << "马老师模式" << endl;
            break;
        }
    }

    ros::spin();
}

//析构函数
EuClusterCore::~EuClusterCore() {}

//自己编写的聚类函数
void EuClusterCore::Euclidean_cluster_DIY(pcl::PointCloud<pcl::PointXYZI>::Ptr in_pc) 
{   
    // cout << "使用自己的方法编写的欧式聚类" << endl;
    //用来标记这个点是否已经被搜索过了
    //用于聚类的点云所具有的点的数量
    int size=in_pc->points.size();
    // cout << "size=" << size << endl;
    std::vector<int> index(size);
    local_indices.clear();

    //手写聚类算法  
    //创建一个kdtree用于进行最近点搜索
    // pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_XYZ(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*in_pc, *cloud_XYZ);
    // cout << "cloud_XYZ->points.size()" << cloud_XYZ->points.size() << endl;

    //FLANN不能用XYZI
    //必须先进行一次点云格式转换
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud_XYZ);

    //创建一个容器用于存储每个类别的索引
    vector<vector<int> > cluster_index;

    //设置聚类半径5m以内用0.5m，5到10用0.75，大于10用1
    double cluster_distance;
    double cluster_num=0;   //当前所具有的类别数量
    
    for(size_t i = 0;i<size;i++)
    {   
        //索引不为0表示已经被分类了
        if(index[i]!=0)
        {
            continue;
        }
        //类别数加一
        cluster_num++;
        index[i]=cluster_num;
        // std::vector<int> Q;//当前聚集成一类的点的索引
        pcl::PointIndices indices_cluster; 
        //每一次都初始化了，不需要特意去清空
        // Q.push_back(i);
        indices_cluster.indices.push_back(i);
        int pQ=0;            //表示此时搜索到Q中的哪一个位置
        //这个聚类计算结束的标志，这个类别的点的数量不变
        // while(pQ<Q.size())
        while(pQ<indices_cluster.indices.size())
        {   
            // int index_search=Q[pQ];
            // pcl::PointXYZ searchPoint; 
            // searchPoint.x=in_pc->points[index_search].x;
            // searchPoint.y=in_pc->points[index_search].y;
            // searchPoint.z=in_pc->points[index_search].z;
            //不用Q的方式
            pcl::PointXYZ searchPoint; 
            auto pit=indices_cluster.indices.begin()+pQ;
            searchPoint.x=in_pc->points[*pit].x;
            searchPoint.y=in_pc->points[*pit].y;
            searchPoint.z=in_pc->points[*pit].z;
            
            //根据与原点的距离确定聚类半径
            float rho=sqrt(searchPoint.x*searchPoint.x+searchPoint.y*searchPoint.y);

            //分段阈值确定聚类半径
            // if(rho<5)
            //     cluster_distance=0.5;
            // else if(rho<10)
            //     cluster_distance=0.75;
            // else
            //     cluster_distance=1;
            //用一个函数确定聚类半径
            cluster_distance=0.1*rho+0.5;

            // cout << "cluster_distance=" << cluster_distance << endl;
            //先假设用一样的聚类半径
            // cluster_distance=CLUSTER_DISTANCE;
            //半径邻域搜索
            std::vector<int> pointIdxRadiusSearch;          //索引
            std::vector<float> pointRadiusSquaredDistance;      //距离
            kdtree.radiusSearch(searchPoint, cluster_distance, pointIdxRadiusSearch, pointRadiusSquaredDistance);
            for(int j=0;j<pointIdxRadiusSearch.size();j++)
            {   
                int index_temp=pointIdxRadiusSearch[j];
                //不等于0表示已经搜索过了
                if(index[index_temp]!=0)
                {   
                    continue;
                }
                index[index_temp]=cluster_num;
                // Q.push_back(index_temp);
                indices_cluster.indices.push_back(index_temp);
            }
            pQ++;
        }
        //结束时pQ=Q.size()
        // if(Q.size()<MIN_CLUSTER_SIZE)
        // {   
        //     continue;
        // }
        if(indices_cluster.indices.size()<MIN_CLUSTER_SIZE)
        {
            continue;
        }
        // cluster_index.push_back(Q);
        local_indices.push_back(indices_cluster);
    }
    // cout << "cluster_size的形式" << endl;
    // for(int i=0;i<cluster_index.size();i++)
    // {
    //     cout << cluster_index[i].size() << "  ";
    // }
    // cout << endl;
    
    //验证结果部分
    // cout << "转化为local_indices的形式" << endl;
    // cout << "-----------总类别数量=" << local_indices.size() << endl;
    // for(int i=0;i<local_indices.size();i++)
    // {
    //     cout << local_indices[i].indices.size() << "  ";
    // }
    // cout << endl;
}

//系统自带的欧式聚类函数,
void EuClusterCore::Euclidean_cluster_function(pcl::PointCloud<pcl::PointXYZI>::Ptr in_pc) 
{   
    cout << "直接调用欧式聚类的函数来计算的结果" << endl;
    local_indices.clear();
    cout << "in_pc->points.size()" << in_pc->points.size() << endl;
    // cout << "计算前,local_indices.size()=" << local_indices.size() << endl;
    // std::vector<pcl::PointIndices> local_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> euclid;
    euclid.setInputCloud(in_pc);
    euclid.setClusterTolerance(CLUSTER_DISTANCE);
    euclid.setMinClusterSize(MIN_CLUSTER_SIZE);
    euclid.setMaxClusterSize(MAX_CLUSTER_SIZE);
    euclid.extract(local_indices);
    cout << "-----------总类别数量=" << local_indices.size() << endl;
    for(int i=0;i<local_indices.size();i++)
    {   
        int num = int(local_indices[i].indices.size()); 
        cout << num << "  ";
    }
    cout << endl;
}

void EuClusterCore::publish_cloud(const ros::Publisher &in_publisher,
                                  const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_to_publish_ptr,
                                  const std_msgs::Header &in_header)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
    cloud_msg.header = in_header;
    in_publisher.publish(cloud_msg);
}


void EuClusterCore::voxel_grid_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr in, pcl::PointCloud<pcl::PointXYZI>::Ptr out, double leaf_size)
{   
    //将滤波后的点的数量限制在4000以内
    if(in->points.size()>8000)
    {
        leaf_size=0.3;
    }
    else if(in->points.size()>5000)
    {
        leaf_size=0.15;
    }
    else
    {
        leaf_size=0.1;
    }
    cout << "位置2the num of in is " << in->size() << endl;
    pcl::VoxelGrid<pcl::PointXYZI> filter;
    filter.setInputCloud(in);
    cout << "leaf_size=" << leaf_size << endl;
    filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    filter.filter(*out);
    cout << "位置3the num of out is " << out->size() << endl;
}

// 欧拉角度转化为四元数的实现部分
EuClusterCore::Quaternion EuClusterCore::ToQuaternion(double yaw, double pitch, double roll) 
{   
    //按照先z轴，后y轴，后x轴的顺序  弧度
    //z-y-x  yaw-pitch-roll
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
 
    Quaternion q;
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;

    // std::cout << "函数调用成功" << std::endl;
 
    return q;
}

//测试传参数的情况
void EuClusterCore::test_fcn(std::vector<pcl::PointIndices> local_indices,int i)
{   
    cout << "test function " << endl;
    std::cout << "yes" << std::endl;

}

//计算特征点，传入参数，存储有所有点的xy坐标的容器
//返回一个数组，四个特征点  LINE_NUM-线的数量
void EuClusterCore::feature_points(vector<vector<double> > points,
                                   vector<vector<double> >* feature_array)
{   
    // cout << "feature_points函数内部" << endl;
    int num=points.size();     //这个类的点的数量
    // vector<vector<vector<double>> > scan_lines(16);    //16线激光雷达，从-15到+15
    vector<vector<vector<double>> > scan_lines(LINE_NUM);    //16线激光雷达，从-15到+15
    // cout << "FPA" << endl;
    // cout << "num=" << num << endl;
    for(int i=0;i<num;i++)
    {   
        // cout << "i=" << i << endl;
        double x=points[i][0],y=points[i][1],z=points[i][2];
        //水平扫描角度,与x轴正方向的夹角
        double h_angle=atan2(y,x);
        h_angle=h_angle*180/pi;
        //垂直方向扫描角度
        double v_angle=atan2(z,sqrt(x*x+y*y));
        int v_omega=int(v_angle*180/pi);
        //距离rho
        double rho=sqrt(x*x+y*y+z*z);
        //10Hz 水平分辨率0.18 垂直角度范围-15到+15,分辨率2，线数为16
        // cout << "i=" << i << endl;
        // cout << "x=" << x << "  y=" << y << "  z=" << z << endl;
        // cout << "h_angle=" << h_angle << "  v_omega=" << v_omega << "   rho=" << rho << endl;
        //根据v_omega判断其应该放在那个容器中,xyz也存进去，之后取方便些
        //v_angle,rho,x,y,z,cur 
        vector<double> p_temp(6);
        p_temp[0]=h_angle;
        p_temp[1]=rho;
        p_temp[2]=x;
        p_temp[3]=y;
        p_temp[4]=z;
        p_temp[5]=-1; //曲率初始化为-1表示没有曲率
        //扫描线索引号
        int line_index=(v_omega+15)/2;
        // int line_index=(v_omega+(LINE_NUM-1)))/2;
        // cout << "line_index=" << line_index << endl;
        scan_lines[line_index].push_back(p_temp);
    }
    //所有点已经放进了扫描线当中
    //对每根扫描线中的点按照水平扫描角度进行排序,从小到大排序
    int N=10;        //一根扫描线中的点多于这个值才需要排序
    // cout << "进行排序" << endl;
    // cout << "B" << endl;
    // for(int i=0;i<16;i++)
    for(int i=0;i<LINE_NUM;i++)
    {
        // cout << "第" << i << "条扫描线中有" << scan_lines[i].size() << "个点" << endl;
        //扫描线中点的数量大于10个点才能计算
        if(scan_lines[i].size()>N)
        {
            //默认会按照升序排序
            sort(scan_lines[i].begin(),scan_lines[i].end());
        }
    }
    //打印验证一下是否真的进行了排序
    // cout << "验证排序的效果" << endl;
    // for(int i=0;i<16;i++)
    // {
    //     cout << "第" << i << "条扫描线中有" << scan_lines[i].size() << "个点" << endl;
    //     for(int j=0;j<scan_lines[i].size();j++)
    //     {
    //         cout << "j=" << j << "  h_angle=" << scan_lines[i][j][0] 
    //              << "  rho=" << scan_lines[i][j][1] << endl;
    //     }
    // }
    //排序过后计算相应的点的曲率
    // cout << "计算相应的点的曲率" << endl;
    // cout << "C" << endl;
    // for(int i=0;i<16;i++)
    for(int i=0;i<LINE_NUM;i++)
    {   
        //这根线上有多少个点
        int num=scan_lines[i].size();
        // cout << "线数为" << i << " 点数为" << num << endl;
        if(num>N)
        {   
            // cout << "正在计算第" << i << "根线" << endl;
            //创建一个容器来存储曲率和索引
            vector<double> curvature;
            vector<int> lp_index;       //line_points_index
            //根据前两个点和后两个点和自身一共5个点计算曲率
            for(int j=2;j<num-2;j++)
            {
                double diff_x=scan_lines[i][j-2][2]+scan_lines[i][j-1][2]
                            +scan_lines[i][j+1][2]+scan_lines[i][j+2][2]
                            -4*scan_lines[i][j][2];
                double diff_y=scan_lines[i][j-2][3]+scan_lines[i][j-1][3]
                            +scan_lines[i][j+1][3]+scan_lines[i][j+2][3]
                            -4*scan_lines[i][j][3];
                double diff_z=scan_lines[i][j-2][4]+scan_lines[i][j-1][4]
                            +scan_lines[i][j+1][4]+scan_lines[i][j+2][4]
                            -4*scan_lines[i][j][4];
                double cur=diff_x*diff_x+diff_y*diff_y+diff_z*diff_z;
                scan_lines[i][j][5]=cur;        //这个点的第五个属性为cur
                curvature.push_back(cur);
                lp_index.push_back(j);
                // cout << "点的编号j=" << j << "   cur=" << cur << endl;
            }
            //按照曲率从大到小进行排序
            sort(scan_lines[i].begin(),scan_lines[i].end(),sortFunCur);
            //验证一下是否按照了曲率进行排序
            // cout << "这是第" << i << "根扫描线，有" << num << "个点" << endl;
            // for(int j=0;j<num;j++)
            // {
            //     for(int k=0;k<6;k++)
            //     {
            //         cout << scan_lines[i][j][k] << "  ";
            //     }
            //     cout << endl;
            // }
            //每根线确实按照了曲率进行排序
        }
        //
    }
    //每根扫描线中选取2个曲率最大的点作为候选特征点
    vector<vector<double> > candidate;
    // cout << "D" << endl;
    // for(int i=0;i<16;i++)
    for(int i=0;i<LINE_NUM;i++)
    {
        if(scan_lines[i].size()>N)
        {   
            //把每根线上曲率最大的两个点的存储进来
            candidate.push_back(scan_lines[i][0]);
            candidate.push_back(scan_lines[i][1]);
        }
    }
    //排序并打印candidate
    sort(candidate.begin(),candidate.end(),sortFunCur);
    // cout << "打印排序后的candidate" << endl;
    // for(int i=0;i<candidate.size();i++)
    // {
    //     for(int j=0;j<6;j++)
    //     {
    //         cout << candidate[i][j] << "  ";
    //     }
    //     cout << endl;
    // } 
    //每个类选择4个作为特征点，存储x y z cur
    //默认选择4个，如果没有四个，则按照candidate中数目来选
    int N_fp=4;
    if(candidate.size()<N_fp)
    {
        N_fp=candidate.size();
    }
    // cout << "N_fp=" << N_fp << endl;
    // cout << "F" << endl;
    for(int i=0;i<N_fp;i++)
    {   
        // cout << "i=" << i << endl;
        //4个属性值，x y z cur
        for(int j=0;j<4;j++)
        {   
            // cout << "j=" << j << endl;
            // cout << "candidate" << candidate[i][j+2] << endl;
            // cout << "feature_array" << (*feature_array)[i][j] << endl;
            (*feature_array)[i][j]=candidate[i][j+2];
            // cout << "feature_array" << (*feature_array)[i][j] << endl;
        }
    }
    // cout << "G" << endl;
}

// 最简单的一种方式，计算AABB包围盒子
double* EuClusterCore::AABBbox_compute(vector<vector<double> > points)
{   
    int num=points.size();

    double min_x = 100;
    double max_x = -100;
    double min_y = 100;
    double max_y = -100;

    for(int i=0;i<num;i++)
    {
        if(points[i][0]<min_x)
        {
            min_x=points[i][0];
        }
        if(points[i][0]>max_x)
        {
            max_x=points[i][0];
        }
        if(points[i][1]<min_y)
        {
            min_y=points[i][1];
        }
        if(points[i][1]>max_y)
        {
            max_y=points[i][1];
        }
    }
    double center_x=(min_x+max_x)/2,
                    center_y=(min_y+max_y)/2,
                    length=max_x-min_x,
                    width=max_y-min_y,
                    angle=0;

    static double output[5]={center_x,center_y,length,width,angle};
    return output;
}

//最小二乘法计算包围盒
double* EuClusterCore::LSMbox_compute(vector<vector<double> > points)
{   
    // cout << "in the function of LSMbox_compute" << endl;
    // int num=points.size();
    // cout << "num=" << num << endl;
    //
    double t1=0, t2=0, t3=0, t4=0;  
    for(int i=0; i<points.size(); i++)  
    {  
        // cout << "i=" << i << endl;
        //x-points[i][0]  y-points[i][1]
        t1 += points[i][0]*points[i][0];  
        t2 += points[i][0];  
        t3 += points[i][0]*points[i][1];  
        t4 += points[i][1];  
    }  
    double k = (t3*points.size() - t2*t4) / (t1*points.size() - t2*t2);  
    //double b = (t1*t4 - t2*t3) / (t1*x.size() - t2*t2);  
    //
    //根据斜率计算包围盒子参数
    // cout << "ready to in the function of box_compute " << endl;

    double *res=box_compute(k,points);

    // cout << "at the end of function of LSMbox_compute" << endl << endl;

    return res;
}

//计算主方向，pub_edge_points发布边缘点
//输入的点为投影到xoy平面并进行了一次体素滤波的点
//极坐标算边缘点找边界点和角点
double EuClusterCore::compute_main_ori(vector<vector<double> > points,double degree,
                                       vector<vector<double> >* all_edge_points)
{   
    // cout << "compute_main_ori内部" << endl;
    double main_ori;
    int num=points.size();

    //如果只有一个点
    if(num==1)
    {
        return 0;
    }

    //第一步，点云坐标转化为极坐标形式
    std::vector<vector<double> > points_polar;
    
    for(int i=0;i<num;i++)
    {   
        std::vector<double> temp(2);   //初始化，默认为0
        double x = points[i][0];       
        double y = points[i][1];
        //范围-pi到pi，前面是y后面是x
        //对于为0的情况同样管用
        double theta=atan2(y,x);
        double rho=sqrt(x*x+y*y);
        temp[0]=theta;
        temp[1]=rho;
        points_polar.push_back(temp);
    }
    // cout << "A" << endl;

    //第二步，列表排序
    //先得到整体角度的最大值和最小值
    double theta_min,theta_max;
    //索引从0开始
    int theta_min_index = 0,theta_max_index = 0;
    for(int i=0;i<num;i++)
    {   
        //比较theta，是0
        if (points_polar[i][0]<points_polar[theta_min_index][0])
            theta_min_index=i;
        else if (points_polar[i][0]>points_polar[theta_max_index][0])
        {
            theta_max_index=i;
        }
    }
    theta_min=points_polar[theta_min_index][0];
    theta_max=points_polar[theta_max_index][0];
    double rad=degree/180*pi;
    int col_table=ceil((theta_max-theta_min)/rad);     //这个表格的列数
    //cout << "col_table:"<<col_table<<endl;
    // cout << "B1" << endl;
    
    std::vector<vector<double> > table;
    for(int i=0;i<col_table;i++)
    {
        vector<double> v_temp;
        table.push_back(v_temp);
    }
    // cout << "B2" << endl;
    

    //点数量序列
    std::vector<int> num_table;
    for(int i=0;i<col_table;i++)
    {
        num_table.push_back(0);
    }
    
    //索引序列
    std::vector<vector<int> > index_table;
    for(int i=0;i<col_table;i++)
    {
        vector<int> v_temp;
        index_table.push_back(v_temp);
    }
    // cout << "B4" << endl;


    //按列分组
    for(int i=0;i<num;i++)
    {   
        // cout << "i=" << i << endl;
        double theta=points_polar[i][0];
        double rho=points_polar[i][1];
        // cout << "B5" << endl;
        int index = ceil((theta-theta_min)/rad)-1;
        if (index==-1)
            index=0;
        num_table[index]++;        
        // cout << "B6" << endl;
        // cout << "index:" << index << endl;
        // cout << "col_table" << col_table << endl;
        table[index].push_back(rho);
        // cout << "B7" << endl;
        index_table[index].push_back(i);
        // cout << "B8" << endl;
    }
    // cout << "C" << endl;

    //获得所有边界点的索引
    //先排序，然后取第一行
    int col = col_table;
    int num_output = 0;
    //存储输出索引的标签，-1表示没有点
    std::vector<int> index_output;

    //找到每一列rho最小的索引
    //第i列
    for(int i=0;i<col;i++)
    {   
        int min=0;  //假设最小值的索引为0
        //如果这一列有点
        if (num_table[i]!=0)
        {    //第j行
            for(int j=1;j<num_table[i];j++)
            {
                if (table[i][j]<table[i][min])
                    min=j;
            }
            num_output=num_output+1;
            //输出所有边缘点的索引
            index_output.push_back(index_table[i][min]);
        }
    }
    // cout << "D" << endl;

    //边缘点
    // cout << "A" << endl;
    std::vector<vector<double> > points_edge;
    for(int i=0;i<num_output;i++)
    {
        double x = points[index_output[i]][0];
        double y = points[index_output[i]][1];

        std::vector<double> v_temp(2);
        v_temp[0]=x;
        v_temp[1]=y;
        //这个类的边缘点
        points_edge.push_back(v_temp);
        //所有类的边缘点
        all_edge_points->push_back(v_temp);
    }
    // cout << "B" << endl;

    //角度最大和最小点的坐标
    //x1 y1为角度最小点的坐标，x2 y2为角度最大点的坐标
    double x1=points[theta_min_index][0];
    double y1=points[theta_min_index][1];
    double x2=points[theta_max_index][0];
    double y2=points[theta_max_index][1];
    //到这条直线距离最远定为角点
    double k=(y2-y1)/(x2-x1);
    double b=y1-k*x1;
    double dis_max=0;           //最大距离
    int index_dis_max=0;        //最大距离所对应的索引，从零开始
    //对所有边缘点进行计算
    for(int i=0;i<num_output;i++)
    {   
        //待计算的点的xy坐标
        //这里应该代入边缘点坐标 points_edge
        double x=points_edge[i][0];
        double y=points_edge[i][1];
        double dis=abs(k*x-y+b)/sqrt(k*k+1);
        if (dis>dis_max)
        {
            dis_max=dis;
            index_dis_max=i;
        }
    }
    // cout << "F" << endl;

    //已经计算出了角点的索引
    //x0,y0为角点的坐标
    double x0=points_edge[index_dis_max][0];
    double y0=points_edge[index_dis_max][1];
    //选择线段较长的作为主方向
    //l1为角度较小的点与角点的连线，l2为角度较大的点
    double l1=sqrt((x0-x1)*(x0-x1)+(y0-y1)*(y0-y1));
    double l2=sqrt((x0-x2)*(x0-x2)+(y0-y2)*(y0-y2));
    // cout << "x0=" << x0 << "    y0=" << y0 << endl;
    // cout << "x1=" << x1 << "    y1=" << y1 << endl;
    // cout << "x2=" << x2 << "    y2=" << y2 << endl;
    // cout << "l1=" << l1 << "    l2=" << l2 << endl;
    if (l1>l2)
    {
        main_ori=(y0-y1)/(x0-x1);
    }
    else
    {
        main_ori=(y0-y2)/(x0-x2);
    }
    
    // cout << "代码块内部=："<< main_ori << endl;

    return main_ori;
}

//project and voxel filter
//将这个类别或者这个格子里面的点投影到xy平面并进行一次体素滤波
double EuClusterCore::project_to_xy(vector<vector<double> > points,
                                                                              vector<vector<double> >* points_xy)
{   
    // cout << "in the funciton of project_to_xy" << endl;
    // cout << "points.size()" << points.size() << endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_points_xy_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    for(int i=0;i<points.size();i++)
    {
        double x=points[i][0];
        double y=points[i][1];
        pcl::PointXYZ point;
        point.x=x;
        point.y=y;
        point.z=0;
        cloud_points_xy_ptr->points.push_back(point);  //add a point to a pointcloud ptr
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_ptxy_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud_points_xy_ptr);
    double voxel=0.1;
    vg.setLeafSize(voxel,voxel,voxel);
    vg.filter(*filtered_ptxy_ptr);
    //add a r-filter
    // pcl::RadiusOutlierRemoval<pcl::PointXYZ> sor;
	// sor.setInputCloud(filtered_ptxy_ptr);
	// sor.setRadiusSearch(0.5);
	// sor.setMinNeighborsInRadius(2);
	// sor.setNegative(false); 
	// sor.filter(*filtered_ptxy_ptr); 
    //

    for(size_t i=0;i<filtered_ptxy_ptr->points.size();i++)
    {
        double x=filtered_ptxy_ptr->points[i].x;
        double y=filtered_ptxy_ptr->points[i].y;
        vector<double> pxy_temp;
        pxy_temp.push_back(x);
        pxy_temp.push_back(y);
        points_xy->push_back(pxy_temp);
        //save to all points xy for pub
        all_points_xy.push_back(pxy_temp);
    }
    // cout << "num of points_xy is " << points_xy->size() << endl;
}

//计算包围盒参数  x  y  l  w  angle  存储所有的边缘点用作可视化用
double* EuClusterCore::box_compute(double main_ori,vector<vector<double> > points)
{   

    // cout << "在compute_box函数中"<<endl;
    int num=points.size();
    // cout << "num=" << num << endl;

    if(num==1)
    {
        static double output[5]={points[0][0],points[0][1],0.1,0.1,0};
        return output;
    }

    //主方向
    double k=main_ori;
    //cout << "k："<< k << endl;
    //y=kx
    double w_max=-100;
    double w_min=100;
    //C++下标从0开始
    int idx_w_max=0;
    int idx_w_min=0;
    for(int i=0;i<num;i++)
    {   
        double x=points[i][0],y=points[i][1];
        double w_now=-(k*points[i][0]-points[i][1])/sqrt(k*k+1);
        // if(x>8  &&  x<8.5 && y<-1)
        // {
        //     cout << "i=" << i << "  w_now=" << w_now << "   w_max=" << w_max << "   w_min=" << w_min << "   x=" << x << "  y="<< y <<endl;
        // }
        if(w_now>w_max)
        {
            w_max=w_now;
            idx_w_max=i;
        }
        if(w_now<w_min)
        {
            w_min=w_now;
            idx_w_min=i;
        }
    }
    double width=w_max-w_min;
    // cout << "w_max:" << w_max << " w_min:" << w_min << endl;
    // cout << "idx_w_max:" << idx_w_max << " idx_w_min:" << idx_w_min << endl;
    //防止width过小
    if(width<0.1)
    {
        width=0.1;
    }

    //x1 y1为距离最大的点（数值上）
    //x2 y2为距离最小的点（数值上）
    double x1=points[idx_w_max][0];
    double y1=points[idx_w_max][1];
    double x2=points[idx_w_min][0];
    double y2=points[idx_w_min][1];
    // cout << "打印宽度方向上的边缘点" << endl;
    // cout << "x1=" << x1 << "    y1=" << y1 << endl;
    // cout << "x2=" << x2 << "    y2=" << y2 << endl;

    //另一个方向
    //y=-1/k x
    double l_max=-100;
    double l_min=100;
    //下标从零开始
    int idx_l_max=0;
    int idx_l_min=0;
    for(int i=0;i<num;i++)
    {   
        //x+ky=0
        double l_now=-(points[i][0]+k*points[i][1])/sqrt(1+k*k);
        if(l_now>l_max)
        {
            l_max=l_now;
            idx_l_max=i;
        }
        if(l_now<l_min)
        {
            l_min=l_now;
            idx_l_min=i;
        }
    }
    double length=l_max-l_min;
    //cout << "l_max:" << l_max << " l_min:" << l_min<<endl;
    //距离大者为x3，距离小者为x4
    double x3=points[idx_l_max][0];
    double y3=points[idx_l_max][1];
    double x4=points[idx_l_min][0];
    double y4=points[idx_l_min][1];
    // cout << "打印长度方向上的边缘点" << endl;
    // cout << "x3=" << x3 << "    y3=" << y3 << endl;
    // cout << "x4=" << x4 << "    y4=" << y4 << endl;

    //计算交点，得中心点坐标
    double k1=k;
    double b1=((y1+y2)-k*(x1+x2))/2;
    double k2=-1/k;
    double b2=((x3+x4)/k+(y3+y4))/2;
    double center_x=(b2-b1)/(k1-k2);
    double center_y=k1*center_x+b1;

    //角度  弧度值
    double angle=atan(k);
    // cout << "angle=" << angle*180/pi << endl;

    static double output[5];
    output[0]=center_x;
    output[1]=center_y;
    output[2]=length;
    output[3]=width;
    output[4]=angle;

    // cout << "A" << endl;

    // cout << "x="<< center_x << "  y=" << center_y << "  l=" << length << "  w=" << width << "  angle=" << angle << endl;

    return output;
}

//return an array 
double* EuClusterCore::enum_compute(vector<vector<double> > points)
{   
    // cout << "in the function of enum_compute" << endl;
    double S_res[46];
    double Smin=1000;
    double kmin=0;
    double alphamin=0;
    static double params[5];
    int epoch=0;
    // cout << "A" << endl;
    for(double alpha=0;alpha<=90;alpha=alpha+2)
    {   
        // cout << "----------------------------" << endl;
        // cout << "epoch=" << epoch << endl;
        double angle;
        if((alpha==0)||(alpha==90))
        {
            angle=alpha+0.5;
        }
        else
        {
            angle=alpha;
        }
        // cout << "B" << endl;
        angle=angle*180/pi;
        double k=tan(angle);
        double *res=box_compute(k,points);
        double x=res[0],y=res[1],l=res[2],w=res[3],theta=res[4];
        double S=l*w;
        S_res[epoch]=S;
        epoch=epoch+1;
        // cout << "C" << endl;
        if(S<Smin)
        {
            Smin=S;
            kmin=k;
            alphamin=alpha;
            params[0]=x;params[1]=y;params[2]=l;params[3]=w;params[4]=theta;
        }
        // cout << "D" << endl;
        if(epoch>2)
        {   
            // cout << "E" << endl;
            if((S_res[epoch-1]<S_res[epoch]) && (S_res[epoch-1]<S_res[epoch-2]))
            {
                break;
            }
        }
    }
    // cout << "F" << endl;
    return params;
}

//k1特征矩阵，k特征矩阵，k1的类别数n，k的类别数m
void EuClusterCore::similarity_martix(vector<vector<double> >* M_S)
{   
    // cout << "in the function of similarity_matirx " << endl;
    //二维int容器，使用且改变 M_S
    // cout << M_S->size() << "  " (*M_S)[0].size() << endl;
    // int n=M_S->size();
    // int m=(*M_S)[0].size();
    // cout << "n=" << n << "  m=" << m << endl;
    for(int i=0;i<n;i++)
    {
        for(int j=0;j<m;j++)
        {
            similarity[i][j]=0;
        }
    }
    //
    // cout << "在函数中打印featurek1"<<endl;
    // for(int i=0;i<n;i++) 
    // {   
    //     cout << i << "  ";
    //     for(int j=0;j<8;j++)
    //     {
    //         cout << feature_k1[i][j] << "  ";
    //     }
    //     cout << endl;
    // }
    // cout << "在函数中打印featurek"<<endl;
    // for(int i=0;i<m;i++)
    // {   
    //     cout << i << "  ";
    //     for(int j=0;j<8;j++)
    //     {
    //         cout << feature_k[i][j] << "  ";
    //     }
    //     cout << endl;
    // }

    //权重参数
    double a=3,b=1,c=1,d=1,e=0.5,f=2,dev=0.7;       //dev分母调整，防止分母为零
    double distance_value=10;      //距离阈值
    double similar_value=0;       //相似度阈值，超过这个值认为有效，否则赋值为0
    // double similar_max=100;         //防止出现巨大的情况
    //n行，m列
    for(int i=0;i<n;i++)
    {   
        // cout << "A" << endl;
        //一共六个特征
        double xk1=feature_k1[i][0];
        double yk1=feature_k1[i][1];
        double lk1=feature_k1[i][2];
        double dk1=feature_k1[i][3];
        double hk1=feature_k1[i][4];
        double Vk1=lk1*dk1*hk1;
        double Sk1=feature_k1[i][5];
        double n1=feature_k1[i][8];     //这一帧物体具有的点数
        // cout << "在相似度矩阵函数中打印距离" << endl;
        // cout << endl << "i=" << i << " Sk1=" << Sk1 << endl;
        for(int j=0;j<m;j++)
        {   
            // cout << "B" << endl;
            double xk=feature_k[j][0];
            double yk=feature_k[j][1];
            double lk=feature_k[j][2];
            double dk=feature_k[j][3];
            double hk=feature_k[j][4];
            double Vk=lk*dk*hk;
            double Sk=feature_k[j][5];
            double n2=feature_k[i][8];     //这一帧物体具有的点数
            // cout << "j=" << j << " Sk=" << Sk << endl;
            
            // //如果两个聚类的距离过大则不需要计算相似度
            double dis=sqrt((xk1-xk)*(xk1-xk)+(yk1-yk)*(yk1-yk));
            // cout << "i=" << i << " j=" << j << endl;
            if (dis>distance_value)
            {   
                // cout << "由于距离太远而相似度为0" << endl;
                (*M_S)[i][j]=0;
                continue;
            }
            // //计算一下这两者的面积变化
            // //面积变化率,sk1这是强度啊大哥
            // double Squarek1=lk1*dk1,Squarek=lk*dk;
            // double Smax=(Squarek1>Squarek?Squarek1:Squarek);
            // double Smin=(Squarek1<Squarek?Squarek1:Squarek);
            // double dS=(Smax-Smin)/Smax;
            // // cout <<  "dS=" << dS << endl;
            // //如果面积的变化率大于一定的阈值，则相似度为0
            // double dS_value=0.25;
            // if(dS>dS_value)
            // {   
            //     // cout << "由于面积变化过大而相似度为0" << endl;
            //     (*M_S)[i][j]=0;
            //     similarity[i][j]=0;
            //     continue;
            // }
            //如果两个点的数量变化太大，认为不是一个物体
            int nmax=(n1>n2?n1:n2);
            int nmin=(n1<n2?n1:n2);
            double dn=(nmax-nmin)/nmax;
            if(dn>0.35)
            {   
                // cout << "由于点数相差过大而相似度为0" << endl;
                (*M_S)[i][j]=0;
                similarity[i][j]=0;
                continue;
            }

            //调整参数不应该放在括号里面，防止负数和正数抵消
            //C++没有平方运算符  //假设把这个i去掉，看主函数里面结果
            //纠结是否需要+i
            double value=a/(pow((xk1-xk),2)+pow((yk1-yk),2)+dev)  
                                        +b/(pow((lk1-lk),2)+dev)             
                                        +c/(pow((dk1-dk),2)+dev)             
                                        +d/(pow((hk1-hk),2)+dev)             
                                        +e/(pow((Vk1-Vk),2)+dev)             
                                        +f/(pow((Sk1-Sk),2)+dev); 
            //超过这个值认为有效，否则赋值为0
            if(value<similar_value)
            {
                value=0;
            }
            (*M_S)[i][j]=value;
            similarity[i][j]=value;
        }
    }
    // cout << "在函数内部打印相似度矩阵" << endl;
    // for(int i=0;i<n;i++)
    // {
    //     for(int j=0;j<m;j++)
    //     {
    //         cout << (*M_S)[i][j] << "  ";
    //     }
    //     cout << endl;
    // }   
    // cout << "similarity array " << endl;
    // for(int i=0;i<n;i++)
    // {
    //     for(int j=0;j<m;j++)
    //     {
    //         cout << similarity[i][j] << "  ";
    //     }
    //     cout << endl;
    // }
}
//计算相似度矩阵

//匹配策略，输入相似度矩阵，输出对照表
void EuClusterCore::correspond(vector<vector<double> > M_S,
                               vector<int>* k1_corr,vector<int>* k_corr)
{   
    // int n=k1_corr->size();
    // int m=k_corr->size();
    // cout << "n=" << n << "    m=" << m << endl;
    // 创建一个F，避免改变M_S
    vector<vector<double> > F;
    for(int i=0;i<n;i++)
    {   
        vector<double> F_raw;
        for(int j=0;j<m;j++)
        {
            F_raw.push_back(M_S[i][j]);
        }
        F.push_back(F_raw);
    }
    double F_ARRAY[CLUSTER_NUM][CLUSTER_NUM];
    for(int i=0;i<n;i++)
    {
        for(int j=0;j<m;j++)
        {
            F_ARRAY[i][j]=similarity[i][j];
        }
    }
    // cout << "在函数中打印F的初值" << endl;
    // for(int i=0;i<n;i++)
    // {
    //     for(int j=0;j<m;j++)
    //     {
    //         cout << F[i][j] << "  ";
    //     }
    //     cout << endl;
    // }
    // cout << "print F_array" << endl;
    // for(int i=0;i<n;i++)
    // {
    //     for(int j=0;j<m;j++)
    //     {
    //         cout << F_ARRAY[i][j] << "  ";
    //     }
    //     cout << endl;
    // }
    //匹配策略
    //第一步，对整个矩阵的元素进行排序
    // cout << "test A" << endl;
    vector<double> F_sort;
    //复制过来后再降序排列
    for(int i=0;i<n;i++)
    {
        for(int j=0;j<m;j++)
        {   
            //use F_array instead of F
            F_sort.push_back(F_ARRAY[i][j]);
        }
    }
    // cout << "test B" << endl;
    //rbegin 和 rend为降序
    sort(F_sort.rbegin(),F_sort.rend());
    // cout << "在函数中打印F_sort" << endl;
    // for(int i=0;i<F_sort.size();i++)
    //     cout << F_sort[i] << endl;
    // cout << "test C" << endl;
    int num_search=0;     
    //对排序后所有的数据进行循环
    for(int i=0;i<F_sort.size();i++)
    {   
        double F_max=F_sort[i];
        //如果已经到0了，停止
        if (F_max==0)
        {
            break;
        }
        //遍历这个矩阵中的所有元素
        //当前元素为第j行第k列
        for(int j=0;j<n;j++)
        {
            for(int k=0;k<m;k++)
            {
                //相等说明第j行与第k列相对应
                //说明第k+1帧的j与第k帧的k对应
                //这一步操作最多执行min(n,m次) 
                if (F_ARRAY[j][k]==F_max)
                {
                    num_search++;
                    k1_corr->at(j)=k;
                    k_corr->at(k)=j;
                    //将这一行，这一列的所有元素全部设置为零
                    for(int temp=0;temp<m;temp++)
                    {
                        F_ARRAY[j][temp]=-1;
                    }
                    for(int temp=0;temp<n;temp++)
                    {
                        F_ARRAY[temp][k]=-1;
                    }
                    //第j行元素全部设置为-1,0也可能代表相似度为0
                    //第k列元素全部设置为-1,0也可能代表相似度为0
                }
            }        
        }
        //如果达到最大搜索次数        
        if (num_search==((n<m)?n:m))
        {
            break;
        }   
    }
    // cout << "test D" << endl;
    //打印k1corr
    // cout << "在函数中打印k1corr"<< endl;
    // for(int i=0;i<n;i++)
    //     cout << k1_corr->at(i) << endl;
}
//匹配策略

//未处理的放入Q中
int EuClusterCore::Q_generate(vector<vector<double> >* Q,
                              vector<int> k1_corr,vector<int> k_corr)
{   
    int n=k1_corr.size();
    int m=k_corr.size();
    int num_Q=0;
    //上一帧存在但是当前帧不存在的，表示暂时消失的
    for(int i=0;i<m;i++)
    {   
        //没有匹配应该等于-1,C++
        if (k_corr[i]==-1)
        {
            //一共六个特征
            vector<double> v_temp;
            for(int temp=0;temp<6;temp++)
            {   
                v_temp.push_back(feature_k[i][temp]);
            }
            Q->push_back(v_temp);
            num_Q++;
            //cout << num_Q << "  " << *(Q+num_Q*6+0);
        }
    }
    // cout << "G1" << endl;
    // cout << "n=" << n << endl;
    //这一帧存在但上一帧不存在的，表示新出现的
    for(int i=0;i<n;i++)
    {   
        // cout << "i=" << i << endl;
        //C++从零开始
        if (k1_corr[i]==-1)
        {   
            vector<double> v_temp;
            // cout << "G11" << endl;
            for(int temp=0;temp<6;temp++)
            {   
                // cout << "G12" << endl;
                // cout << "num_Q=" << num_Q << "temp=" << temp << endl;
                v_temp.push_back(feature_k1[i][temp]);
                // cout << "G13" << endl;
            }
            Q->push_back(v_temp);
            // cout << "G14" << endl;
            num_Q++;
        }
    }
    // cout << "G2" << endl;
    return num_Q;
}
//动态列表Q的添加

//状态列表G的计算
//最初为根据两帧的包围盒中心计算
//先尝试采用最近点的坐标计算
void EuClusterCore::G_generate(vector<vector<double> >* G,
                vector<int> k1_corr,vector<int> k_corr,double T)
{   
    // cout << "in G_generate" << endl;
    //这里只需要获得当前帧的xk1 yk1和上一帧的xk yk
    //feature_k  feature_k1特征矩阵，k1_corr匹配矩阵
    //当前帧k+1，有n个类别
    int n=k1_corr.size();
    int m=k_corr.size();

    for(int i=0;i<n;i++)
    {
        int corr=k1_corr[i];      //是否有对应 第k+1帧的i和第k帧的corr对应
        //没有对应应该是-1,c++从0开始
        if(corr==-1)//没有对应
        {                                
            (*G)[i][0]=feature_k1[i][0];    //x
            (*G)[i][1]=feature_k1[i][1];    //y
            //没有对应就速度为0
            (*G)[i][2]=0;                   //speed这个不能完全为0
            // cout << "无对应情况下i=" << i << "  G[i][2]=" << (*G)[i][2] << endl;
            (*G)[i][3]=0;                   //theta
            // cout << "无对应情况下i=" << i << "  G[i][3]=" << (*G)[i][3] << endl;
        }                
        else//有对应
        {   
            // cout << "i=" << i << "  corr=" << corr << endl;
            //如果这个类别的点的数量小于30，不计算速度
            //第8个才是点数量
            if(feature_k1[i][8]<30)
            {   
                // cout << "点数太少，不具有参考意义" << endl;
                double xk1=feature_k1[i][0],yk1=feature_k1[i][1];
                (*G)[i][0]=xk1;
                (*G)[i][1]=yk1;
                (*G)[i][2] = 0;
                (*G)[i][3]=0;
                continue;
            }
            // cout << "test1" << endl;
            //x和y的变化量，前一时刻指向后一时刻
            double dx,dy;
            double xk1=feature_k1[i][0],yk1=feature_k1[i][1],
                            xk=feature_k[corr][0],yk=feature_k[corr][1];
            //保留小数点后一位，精确到分米
            xk1=double((int)(xk1*10))/10;
            yk1=double((int)(yk1*10))/10;
            xk=double((int)(xk*10))/10;
            yk=double((int)(yk*10))/10;
            // cout << "xk1=" << xk1 << "  yk1=" << yk1 << "  xk=" << xk << "  yk=" << yk << endl;
            (*G)[i][0]=xk1;
            (*G)[i][1]=yk1;
            //假定激光雷达误差+-2cm
            //速度限定

            //之前是用包围盒中心计算的
            // double xclosek1=feature_k1[i][10],yclosek1=feature_k1[i][11],
            //                 xthetamaxk1=feature_k1[i][12],ythetamaxk1=feature_k1[i][13],
            //                 xthetamink1=feature_k1[i][14],ythetamink1=feature_k1[i][15],
            //                 xclosek=feature_k[corr][10],yclosek=feature_k[corr][11],
            //                 xthetamaxk=feature_k[i][12],ythetamaxk=feature_k[i][13],
            //                 xthetamink=feature_k[i][14],ythetamink=feature_k[i][15];
            // xclosek1=double((int)(xclosek1*10))/10;
            // yclosek1=double((int)(yclosek1*10))/10;
            // xclosek=double((int)(xclosek*10))/10;
            // yclosek=double((int)(yclosek*10))/10;
            // cout << "xclosek1=" << xclosek1 << "  yclosek1=" << yclosek1 
            //           << "  xclosek=" << xclosek << "  yclosek=" << yclosek << endl;
            // //最大扫描角和最小扫描角中间的那个点
            // double xmidk1=(xthetamaxk1+xthetamink1)/2,
            //                 ymidk1=(ythetamaxk1+ythetamink1)/2,
            //                 xmidk=(xthetamaxk+xthetamink)/2,
            //                 ymidk=(ythetamaxk+ythetamink)/2;
            // cout << "xmidk1=" << xmidk1 << "  ymidk1=" << ymidk1 << "   xmidk=" << xmidk << "   ymidk=" << ymidk << endl;
            // dx=xk1-xk;dy=yk1-yk;
            
            //用点云中距离原点最近点计算 

            // dx=xclosek1-xclosek;dy=yclosek1-yclosek;

            //用包围矩形顶点中距离原点最近的点计算
            //返回值为数组           // double *res=box_compute(main_ori,points_xy);
            // cout << "test2" << endl;
            // double Vertex1[2],Vertex2[2];
            double *Vertex1=closest_vertex_box(xk1,yk1,feature_k1[i][2],feature_k1[i][3],feature_k1[i][6]);
            double x1=Vertex1[0],y1=Vertex1[1];
            // cout << "test21" << endl;
            double *Vertex2=closest_vertex_box(xk,yk,feature_k[corr][2],feature_k[corr][3],feature_k[corr][6]);
            double x2=Vertex2[0],y2=Vertex2[1];
            // cout << "x1=" << x1 << "    x2=" << x2 << endl;
            dx=x1-x2;
            dy=y1-y2;
            // cout << "test22" << endl;

            // cout << "dx=" << dx << "    dy=" << dy << endl;
            //dis精确到小数点后二位
            double dis = sqrt(pow(dx,2)+pow(dy,2));
            dis=double((int)(dis*100))/100;
            // cout << "dis=" << dis << endl;
            if(dis<=0.05)
            {
                dis=0;
            }
            // cout << "dis=" << dis << endl;
            double v = dis/T;
            (*G)[i][2] = v;
            (*G)[i][3]=atan2(dy,dx);             //theta
        }
    }
    // cout << "在函数中打印G" << endl;
    // for(int i=0;i<n;i++)
    // {   
    //     cout << i << "  ";
    //     for(int j=0;j<4;j++)
    //     {
    //         cout << (*G)[i][j] << "  ";
    //     }
    //     cout << endl;
    // }
}
//状态列表G的计算

//按照距离分组并聚类
void EuClusterCore::cluster_segment(pcl::PointCloud<pcl::PointXYZI>::Ptr in_pc)
{   
    //添加中文
    setlocale(LC_CTYPE, "zh_CN.utf8");
    //添加中文输出

    // cout << "in_pc->points.size()=" << in_pc->points.size() << endl;

    // pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);

    //create 2d pc
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::copyPointCloud(*in_pc, *cloud_2d);
    //临时的cloud_2d变量，来测试一下
    // cout << "测试不同类型的点云可不可以复制" << endl;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_XYZ(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::copyPointCloud(*in_pc, *cloud_XYZ);
    // cout << "cloud_XYZ->points.size()" << cloud_XYZ->points.size() << endl;

    // cout << "cloud_2d->points.size()=" << cloud_2d->points.size() << endl;

    //去除Nan点,输入点云，输出点云和对应保留的索引
    // std::vector<int> mapping;
    // pcl::removeNaNFromPointCloud(*cloud_2d, *cloud_2dt, mapping);
    // cout << "cloud_2d.size" << cloud_2d->points.size() << endl;
    // cout << "cloud_2dt.size" << cloud_2dt->points.size() << endl;

    // cout << "mapping.size()" << mapping.size() << endl;
    
    //make it flat,先投影到xoy平面
    for (size_t i = 0; i < cloud_2d->points.size(); i++)
    {   
        // cout << "x=" << cloud_2d->points[i].x << 
        //         "  y=" << cloud_2d->points[i].y << "  z=" << cloud_2d->points[i].z << endl;
        cloud_2d->points[i].z = 0;
    }

    // cout << "具有的点的数量：" << cloud_2d->points.size() << endl;

    //now we need a voxel filter
    // cout << "before vf size is " << cloud_2d->points.size() << endl;
    // pcl::VoxelGrid<pcl::PointXYZI> filter;
    // filter.setInputCloud(cloud_2d);
    // double voxel_size=0.01;
    // filter.setLeafSize(voxel_size, voxel_size, voxel_size);
    // filter.filter(*cloud_2d);
    // cout << "after vf size is " << cloud_2d->points.size() << endl;

    // if (cloud_2d->points.size() > 0)
    //     tree->setInputCloud(cloud_2d);

    //这两个函数一次只能调用一个
    if(CLUSTER_MODE==0)
    {
        Euclidean_cluster_function(cloud_2d);
    }
    else
    {
        Euclidean_cluster_DIY(cloud_2d);
    }

    // test_fcn(local_indices,1);

    //当前帧这个距离范围内总的类别数，这个在h文件中定义
    n=local_indices.size();
    int num_box = 0;   //这个类别需要划分的盒子数量-不可通行区域

    //存储所有类的边缘点
    vector<vector<double> > all_edge_points;

    //按照类别保存这一帧的点的坐标存入txt
    // string filename;
    // filename=to_string(frame_id);
    // string filepath="/home/sugar/catkin_ws/src/euclidean_cluster/data/coordinate/";
    // filepath.append(filename);
    // std::ofstream out(filepath,std::ios::app);
    // out<<fixed<<setprecision(2);

    //对这一帧按找类别进行循环
    // cout << "A" << endl;
    for (size_t i = 0; i < local_indices.size(); i++)
    {   
        // cout << "2  0   0   0" << endl;
        // out << "-1  -1  -1  -1" << endl;
        int num = int(local_indices[i].indices.size());    //这个类别具有的点的数量
        //存储这个类别具有的点的数量
        feature_k1[i][8]=num;
        
        // cout << "i=" << i << "  the points num is " << num << endl;

        double min_x = 100;
        double max_x = -100;
        double min_y = 100;
        double max_y = -100;
        double min_z = 100;
        double max_z = -100;
        
        //创建一个数组存储这个聚类的所有点的xyz坐标
        std::vector<vector<double> > points;
        //这个类别中所有的点投影到XOY平面并体素滤波
        std::vector<vector<double>> points_xy;

        //激光雷达平均强度
        double intensity_ave=0;
        
        //距离原点最近的点
        double rho_min=100;
        auto rho_min_idx=local_indices[i].indices.begin();
        //水平扫描角最大和最小的两个点
        double theta_max=-200,theta_min=200;
        auto theta_max_idx=local_indices[i].indices.begin(),
        theta_min_idx=local_indices[i].indices.begin();
        int theta_max_num,theta_min_num;
        int point_num=0;
        
        // cout << "A1" << endl;
        //对这个类的所有点进行循环,算坐标最值以及平均强度
        for (auto pit = local_indices[i].indices.begin(); pit != local_indices[i].indices.end(); ++pit)
        {   
            pcl::PointXYZI p;
            p.x = in_pc->points[*pit].x;
            p.y = in_pc->points[*pit].y;
            p.z = in_pc->points[*pit].z;
            // cout << "p.z=" << p.z << endl;
            p.intensity = in_pc->points[*pit].intensity;
            //水平扫描角度
            // double h_angle=atan2(p.y,p.x);
            // h_angle=h_angle*180/pi;
            // //水平扫描角极值求解
            // if(h_angle>theta_max)
            // {   
            //     theta_max=h_angle;
            //     theta_max_idx=pit;
            //     theta_max_num=point_num;
            // }
            // if(h_angle<theta_min)
            // {   
            //     theta_min=h_angle;
            //     theta_min_idx=pit;
            //     theta_min_num=point_num;
            // }
            // double rho=p.x*p.x+p.y*p.y;
            // if(rho<rho_min)
            // {   
            //     rho_min=rho;
            //     rho_min_idx=pit;
            // }
            // //垂直扫描角度
            // double v_angle=atan2(p.z,sqrt(p.x*p.x+p.y*p.y));
            // v_angle=v_angle*180/pi;
            // cout << "i=" << i << " h_angle=" << h_angle << " v_angle=" << v_angle << endl;

            //输出坐标
            // if(i==0)
            // {
                // cout << p.x<<"     "<<p.y<<"    "<<  p.z<<"  "<<p.intensity<<endl;
            // }
            
            //点的坐标存入txt
            // out << p.x << "  " << p.y << "  " << p.z << "  " << p.intensity << endl;

            //复制到vector中
            std::vector<double> v_temp(3);
            v_temp[0]=p.x;
            v_temp[1]=p.y;
            v_temp[2]=p.z;
            points.push_back(v_temp);

            if (p.x < min_x)
                min_x = p.x;
            if(p.y < min_y)
                min_y = p.y;
            if (p.z < min_z)
                min_z = p.z;
            if (p.x > max_x)
                max_x = p.x;
            if (p.y > max_y)
                max_y = p.y;
            if (p.z > max_z)
                max_z = p.z;
            // cout << "执行了求边界范围的操作"<<"min_x:"<<min_x<<endl;

            //计算强度的平均值
            intensity_ave+=p.intensity;

            //
            point_num++;
        }
        //
        // cout  << "i=" << i << endl;
        // cout << "打印角度最大和最小的索引" << endl;
        // cout << "theta_max_idx=" << theta_max_idx << endl;
        // cout << "theta_min_idx=" << theta_min_idx << endl;
        feature_k1[i][5]=intensity_ave;
        //将求到的相关点的坐标赋值
        feature_k1[i][9]=in_pc->points[*rho_min_idx].x;
        feature_k1[i][10]=in_pc->points[*rho_min_idx].y;
        feature_k1[i][11]=in_pc->points[*theta_max_idx].x;
        feature_k1[i][12]=in_pc->points[*theta_max_idx].y;
        feature_k1[i][13]=in_pc->points[*theta_min_idx].x;
        feature_k1[i][14]=in_pc->points[*theta_min_idx].y;
        //保留一位小数
        feature_k1[i][9]=double((int)(feature_k1[i][9]*10))/10;
        feature_k1[i][10]=double((int)(feature_k1[i][10]*10))/10;
        feature_k1[i][11]=double((int)(feature_k1[i][11]*10))/10;
        feature_k1[i][12]=double((int)(feature_k1[i][12]*10))/10;
        feature_k1[i][13]=double((int)(feature_k1[i][13]*10))/10;
        feature_k1[i][14]=double((int)(feature_k1[i][14]*10))/10;
        
        // if (points.size()>1000)
        // {
        //     cout << "points的维度" << points.size() << endl;
        // }
        double h=max_z-min_z;
        feature_k1[i][4]=h;
        //激光雷达强度的平均值
        intensity_ave/=num;
        //计算主方向
        double degree=0.2;   //the old is 2
        
        //
        project_to_xy(points,&points_xy);

        // cout << "print points_xy" << endl;
        // for(int i=0;i<points_xy.size();i++)
        // {
        //     double x=points_xy[i][0],y=points_xy[i][1];
        //     // if(x>8 && x<8.5 && y<-1)
        //     // {
        //         // cout << "i=" << i << "  x=" << x << "   y=" << y << endl;
        //         // cout << "  " << x << "   " << y << endl;
        //     // }
        // }

        //----------------------选择一个使用------------------------//
        //提取边缘计算主方向,然后再计算包围盒
        // double main_ori = compute_main_ori(points_xy,degree,&all_edge_points);
        // double *res=box_compute(main_ori,points_xy);
        //迭代的方法计算包围盒
        // double *res=enum_compute(points_xy);
        //最小二乘法计算包围盒子
        double *res=LSMbox_compute(points_xy);
        //AABB包围盒子计算
        // double *res= AABBbox_compute(points_xy);

        // cout << "from all_edge_points to test_vector " << endl;
        //test
        // for(int i=0;i<all_edge_points.size();i++)
        // {
        //     vector<double> v_temp;
        //     double x=all_edge_points[i][0];
        //     double y=all_edge_points[i][1];
        //     // cout << "i=" << "   x=" << x << "   y=" << y << endl;
        //     v_temp.push_back(x);
        //     v_temp.push_back(y);
        //     test_vector.push_back(v_temp);
        // }

        //每个类存储四个点作为特征点,每个点存储x y z cur  4个点，每个点4个参数
        //创建一个容器，改变容器的值
        // cout << "打印所有的特征点" << endl;
        // vector<vector<double> > feature_array;
        // for(int i=0;i<4;i++)
        // {
        //     vector<double> v_temp(4);
        //     feature_array.push_back(v_temp);
        // }
        // cout << "feature_array.size()=" << feature_array.size() << endl;
        // cout << "feature_array[0].size()=" << feature_array[0].size() << endl;

        // feature_points(points,&feature_array);   //问题在这里,不运行这个没有影响

        //需要把feature_array的点赋值到FP_data_k1中
        //整个大循环外i是类别数
        // for(int j=0;j<4;j++)
        // {
        //     for(int k=0;k<4;k++)
        //     {   
        //         //i代表第i个类别
        //         FP_data_k1[i][j][k]=feature_array[j][k];
        //     }
        // }
        // cout << "在cluster_segment函数中" << endl;
        // for(int j=0;j<4;j++)
        // {
        //     for(int k=0;k<4;k++)
        //     {
        //         cout << feature_array[j][k] << "  ";
        //     }
        //     cout << endl;
        // }
        
        double x_cluster=res[0];
        double y_cluster=res[1];
        double l_cluster=res[2];
        double w_cluster=res[3];
        double angle_cluster=res[4];
        // cout << "test B1" << endl;
        // cout << "box:" << x_cluster << "  " << y_cluster << "  " << l_cluster << "  " << w_cluster 
                            //    << "  " << angle_cluster << endl; 
        feature_k1[i][0]=x_cluster;
        feature_k1[i][1]=y_cluster;
        feature_k1[i][2]=l_cluster;
        feature_k1[i][3]=w_cluster;
        //加入angle
        feature_k1[i][6]=angle_cluster;
        feature_k1[i][7]=(max_z + min_z)/2;      //最大z值和最小z值的中间值
        // cout << "test B2" << endl;
        //keep 2 dot
        for(int temp=0;temp<8;temp++)
        {   
            // cout << "before change feature_k1[i][temp]=" << feature_k1[i][temp] << endl;
             feature_k1[i][temp]=(double)((int)(feature_k1[i][temp]*100.0)/100.0);
            //  cout << "after change feature_k1[i][temp]=" << feature_k1[i][temp] << endl;
        }
        //for l and w
        feature_k1[i][2]=feature_k1[i][2]+0.2;feature_k1[i][3]=feature_k1[i][3]+0.2;
        // cout << "test B3" << endl;
        //
        double x_range=max_x-min_x;
        double y_range=max_y-min_y; 
        double limit=5.0;                  //尺寸判断阈值，超过则判断为大物体

        // cout << "test C" << endl; 

        //划分可通行区域
        if(true)
        {
            //根据这个区域的大小划分不可通行区域
            if(x_range>limit||y_range>limit)
            {   
                // cout << "D3" << endl;
                double grid_size=2.0;
                int num_x=ceil(x_range/grid_size);
                int num_y=ceil(y_range/grid_size);
                for(int i1=0;i1<num_x;i1++)
                {
                    for(int j1=0;j1<num_y;j1++)
                    {   
                        //这个小栅格的x和y的上下界限
                        double x_low = min_x+i1*grid_size,
                        x_high = min_x+(i1+1)*grid_size,                    
                        y_low = min_y+j1*grid_size,
                        y_high = min_y+(j1+1)*grid_size;
                        //这个小栅格中的点坐标
                        std::vector<vector<double> > points_this_grid;
                        //这个栅格投影到XOY并体素滤波
                        std::vector<vector<double>> points_grid_xy;
                        //统计在这个栅格中的点
                        for (auto pit = local_indices[i].indices.begin(); pit != local_indices[i].indices.end(); ++pit)
                        {
                            //需要分割的情况
                            double p_grid_x = in_pc->points[*pit].x,
                                p_grid_y = in_pc->points[*pit].y;
                            if((p_grid_x >= x_low && p_grid_x <= x_high)&&
                            (p_grid_y >= y_low && p_grid_y <= y_high))
                            {   
                                std::vector<double> v_temp(2);
                                v_temp[0]=p_grid_x;
                                v_temp[1]=p_grid_y;
                                points_this_grid.push_back(v_temp);
                            }
                            else continue;
                        }
                        // cout << "D4" << endl;
                        // cout << "这个栅格的点的数量："<< points_this_grid.size() << endl;

                        //这个栅格里面有点才需要创建一个包围盒子
                        if(points_this_grid.size()!=0)
                        {   
                            // cout << "C0分割后栅格里面有点" << endl;

                            //投影到XOY平面并体素滤波
                            // cout << "points_this_grid.size()=" << points_this_grid.size() << endl;
                            project_to_xy(points_this_grid,&points_grid_xy);
                            // cout << "points_grid_xy.size()=" << points_grid_xy.size() << endl;
                            
                            //----------------------------选择一种方法进行计算----------------------------
                            //边缘点计算包围盒子
                            // double degree_grid=2;
                            // double main_ori_grid=compute_main_ori(points_this_grid,degree_grid,&all_edge_points);
                            // double *res_box_grid=box_compute(main_ori_grid,points_this_grid);
                            //最小二乘法计算包围盒子
                            double *res_box_grid=LSMbox_compute(points_grid_xy);
                            //AABB包围盒子计算
                            // double *res_box_grid=AABBbox_compute(points_grid_xy);

                            // cout << "C1" << endl;

                            //这里不需要存到feature_k1中，因为不是整个类别的参数
                            //feature的数据结构  x  y  l  d   h   S
                            double center_x_grid=res_box_grid[0],   
                                center_y_grid=res_box_grid[1],
                                length_grid=res_box_grid[2],
                                width_grid=res_box_grid[3],
                                angle_grid=res_box_grid[4];
                            // cout << "center_x_grid" << center_x_grid 
                                //  << "center_y_grid" << center_y_grid << endl;

                            // cout << "C1" << endl;
                            
                            double pos_x=center_x_grid,pos_y=center_y_grid,pos_z=(max_z + min_z)/2; 
                            // cout << "C2" << endl;           
                            double length_ = length_grid;
                            double width_ = width_grid;
                            //不太需要关注高度
                            double height_ = 1;
                            // if (height_<0.1)
                            // {
                            //     height_=0.1;
                            // }
                            //把通行区域包围盒子的信息传递到数组中
                            PAbox_data[PAbox_num][0]=pos_x;
                            PAbox_data[PAbox_num][1]=pos_y;
                            PAbox_data[PAbox_num][2]=pos_z;
                            PAbox_data[PAbox_num][3]=length_;
                            PAbox_data[PAbox_num][4]=width_;
                            PAbox_data[PAbox_num][5]=height_;
                            PAbox_data[PAbox_num][6]=angle_grid;
                            PAbox_num++;
                        }
                    // cout << "C6" << endl;
                    }
                // cout << "C7" << endl;        
                }
            // cout << "C8" << endl;
            }
            //如果这个聚类的范围不是那么大，则不需要进行分割操作
            else
            {   
                // cout << "D" << endl;
                //不分割，则包围盒结果直接作为参数
                double center_x_grid=feature_k1[i][0],   //x 
                    center_y_grid=feature_k1[i][1],   //y
                    center_z_grid=(max_z + min_z)/2,  //z
                    length_grid=feature_k1[i][2],     //length
                    width_grid=feature_k1[i][3],      //width
                    height_grid=max_z-min_z,          //height
                    angle_grid=angle_cluster;         //这个栅格的angle即为整个类的angle
                // cout << "center_x_grid=" << center_x_grid << "  center_y_grid=" << center_y_grid;
                double length_ = length_grid;
                double width_ = width_grid;
                double height_ = height_grid;
                if (height_<0.1)
                {   
                    height_=0.1;
                }
                //角度转换为四元数
                
                //数据存入PAbox_data
                PAbox_data[PAbox_num][0]=center_x_grid;
                PAbox_data[PAbox_num][1]=center_y_grid;
                PAbox_data[PAbox_num][2]=center_z_grid;
                PAbox_data[PAbox_num][3]=length_grid;
                PAbox_data[PAbox_num][4]=width_grid;
                PAbox_data[PAbox_num][5]=height_grid;
                PAbox_data[PAbox_num][6]=angle_grid;
                PAbox_num++;
            }
        }
    }
    //关闭文件
    // out.close();
    // cout << "B" << endl;

    //发布边缘点
    if(false)
    {
        //所有类的相关参数已经计算完毕,准备发布边缘点
        unsigned int num_points=all_edge_points.size();
        pcl::PointCloud<pcl::PointXYZ> cloud;
        sensor_msgs::PointCloud2 cloud_output;
        // sensor_msgs::PointCloud cloud;
        // cloud.header.stamp = ros::Time::now();
        cloud.header.frame_id = str_frame;
        cloud.points.resize(num_points);
        // cloud.channels[0].name="intensity";
        // cloud.channels[0].values.resize(num_points);
        // cout << "CS6" << endl;
        for(int i=0;i<num_points;i++)
        {
            cloud.points[i].x = all_edge_points[i][0];
            cloud.points[i].y = all_edge_points[i][1];
            cloud.points[i].z = 0;
            // cloud.channels[0].values[i] = 10;
        }
        pcl::toROSMsg(cloud,cloud_output);
        // cout << "E3" << endl;
        pub_edge_points.publish(cloud_output);
        // cout << "CS7" << endl;
    }
}

void EuClusterCore::to_grid_map(double output_data[CLUSTER_NUM][7],int n,vector<vector<int> >* grid_map)
{
    // cout << "----------------to_grid_map函数定义和调用成功----------------" << endl;
    //改变其中的变量的值的方式(*grid_map)[i][j]=1;
    //二维int容器，使用且改变值
    //x方向的栅格个数和y方向的栅格个数
    int xsize=grid_map->size();
    int ysize=(*grid_map)[0].size();
    //依次遍历所有的类别，如果这个物体扫过了这个栅格，则赋值为1
    for(int i=0;i<n;i++)
    {
        //先整个简单的，物体没有运动
        //step1计算四个顶点的坐标
        double x=output_data[i][0],
               y=output_data[i][1],
               l=output_data[i][2],
               w=output_data[i][3],
               angle=output_data[i][4],
               v=output_data[i][5],
               theta=output_data[i][6];
        // cout << "i=" << i << "-------------------" << endl;
        // cout << "x=" << x << "  y=" << y << "  l=" << l << "  w=" << w
            //  << "  angle=" << angle << "  v=" << v << "  theta=" << theta << endl;
        double k0=tan(angle);
        // cout << "k0=" << k0 << endl;
        //方向1和方向2的的单位向量-主方向和副方向
        double dir1[2]={1/sqrt(1*1+k0*k0),k0/sqrt(1*1+k0*k0)},
               dir2[2]={-k0/sqrt(1*1+k0*k0),1/sqrt(1*1+k0*k0)};
        //都把方向化为y向正的
        if(dir1[1]<0)
        {
            dir1[0]=-dir1[0];
            dir1[1]=-dir1[1];
        }
        // cout << "dir1:" << dir1[0] << "  " << dir1[1] << endl;
        // cout << "dir2:" << dir2[0] << "  " << dir2[1] << endl;
        //p1为左上角的点，顺时针排列
        double p[4][2];
        //1号点
        p[0][0]=x-dir1[0]+dir2[0];
        p[0][1]=y-dir1[1]+dir2[1];
        //2号点
        p[1][0]=x+dir1[0]+dir2[0];
        p[1][1]=y+dir1[1]+dir2[1];
        //3号点
        p[2][0]=x+dir1[0]-dir2[0];
        p[2][1]=y+dir1[1]-dir2[1];
        //4号点
        p[3][0]=x-dir1[0]-dir2[0];
        p[3][1]=x-dir1[1]-dir2[1];
        //
        // cout << "打印顶点的连续坐标" << endl;
        // for(int i=0;i<4;i++)
        // {
        //     cout << "i=" << i << "  " << p[i][0] << "  " << p[i][1] << endl;
        // }
        //step2计算四条边的方程，在栅格坐标系中表示
        //栅格坐标系即x y 坐标均为整数
        //边的编号，点1和点2为边1,以此类推
        //把所有的点的坐标化为栅格坐标
        for(int i=0;i<4;i++)
        {
            p[i][0] = ceil((p[i][0]+ROI_REAR)/GRID_SIZE)-1;
            p[i][1] = ceil((p[i][1]+ROI_LATERAL)/GRID_SIZE)-1;
        }
        // cout << "打印顶点的栅格化坐标" << endl;
        // for(int i=0;i<4;i++)
        // {
        //     cout << "i=" << i << "  " << p[i][0] << "  " << p[i][1] << endl;
        // }
        //计算栅格表示下的斜率和截距
        double k[4],b[4];
        for(int i=0;i<4;i++)
        {   
            if(i<3)
            {
                k[i]=(p[i][1]-p[i+1][1])/(p[i][0]-p[i+1][0]);
                //k=(y2-y1)/(x2-x1)
                b[i]=p[i][1]-k[i]*p[i][0];
                //b=y-k*x
            }
            else
            {
                k[i]=(p[i][1]-p[0][1])/(p[i][0]-p[0][0]);
                b[i]=p[i][1]-k[i]*p[i][0];
            }
        }
        // cout << "打印栅格化的斜率和截距" << endl;
        // for(int i=0;i<4;i++)
        // {
        //     cout << "i=" << i << "  k=" << k[i] << "  b=" << b[i] << endl;
        // }
        //step3根据四条边框定的范围对在内部的栅格进行赋值
        //不需要所有的都遍历，只需要遍历这个区域范围内的
        //代入四条直线的直线方程中即可判断
        //y=kx+b  kx-y+b=0
        //找到x和y 的 最大最小值
        //xmin_index和ymin的索引
        int xmin_index=0,xmax_index=0,ymin_index=0,ymax_index=0;
        for(int i=0;i<4;i++)
        {
            if(p[i][0]<p[xmin_index][0])
            {
                xmin_index=i;
            }
            if(p[i][0]>p[xmax_index][0])
            {
                xmax_index=i;
            }
            if(p[i][1]<p[ymin_index][1])
            {
                ymin_index=i;
            }
            if(p[i][1]>p[ymax_index][1])
            {
                ymax_index=i;
            }
        }
        double xmin=p[xmin_index][0],xmax=p[xmax_index][0],
               ymin=p[ymin_index][1],ymax=p[ymax_index][1];
        //将坐标转化为栅格坐标，先栅格化再进行的斜率和截距的计算，不需要再次栅格化
        // int x = ceil((in->points[i].x+ROI_REAR)/grid_size)-1;
        // int y = ceil((in->points[i].y+ROI_LATERAL)/grid_size)-1;
        // int xmingrid=ceil((xmin+ROI_REAR)/GRID_SIZE)-1,
        //     xmaxgrid=ceil((xmax+ROI_REAR)/GRID_SIZE)-1,
        //     ymingrid=ceil((ymin+ROI_LATERAL)/GRID_SIZE)-1,
        //     ymaxgrid=ceil((ymax+ROI_LATERAL)/GRID_SIZE)-1;
        // //对这个矩形区域内的栅格进行判断
        int xmingrid=xmin,
            xmaxgrid=xmax,
            ymingrid=ymin,
            ymaxgrid=ymax;
        // cout << "xmingrid=" << xmingrid << "  xmaxgrid=" << xmaxgrid
        //      << "  ymingrid=" << ymingrid << "  ymaxgrid=" << ymaxgrid
        //      << endl;
        // cout << "打印障碍物的栅格" << endl;
        for(int i=xmingrid;i<xmaxgrid;i++)
        {
            for(int j=ymingrid;j<xmaxgrid;j++)
            {   
                // cout << "i=" << i << "  j=" << j << endl;
                // int dot;
                // cin >> dot;
                //满足在四条线之内，在L1和L2下面，在L3和L4上面
                //kx-y+b>0说明在这条线下面
                double d[4];
                for(int i=0;i<4;i++)
                {
                    d[i]=k[i]*i-j+b[i];
                    // cout << "d[" << i << "]=" << d[i] << "  "; 
                }
                cout << endl;
                if(d[0]>0 && d[1]>0 && d[2]<0 && d[3]<0)
                {
                    //这个栅格为障碍物
                    (*grid_map)[i][j]=1;
                    // cout << "i=" << i << "  j=" << j << "赋值为1" << endl;
                }
            }
        }
    }
}

void EuClusterCore::data_to_txt()
{   
    cout << "存储为文件" << endl;
    //data struct1 n*8*4  x-y-z-1
    //data struct2 h w l x y z
    //the data struct of feature_k1 x y l w h s angle z
    //keep in mind that l is parallel with x-axis and w with y
    string filename1,filename2,id;
    if(frame_id<10)
    {   
        // cout << "1" << endl;
        //txt1
        filename1="vertex_000000000";
        id=to_string(frame_id);
        //txt2
        filename2="box_000000000";

    }
    else if(frame_id<100)
    {   
        // cout << "2" << endl;
        //txt1
        filename1="vertex_00000000";
        id=to_string(frame_id);     
        //txt2
        filename2="box_00000000";   
    }
    else if(frame_id<1000)
    {   
        // cout << "3" << endl;
        cout << "frame_id = " << frame_id << endl;
        //txt1
        filename1="vertex_0000000";
        id=to_string(frame_id);     
        //txt2
        filename2="box_0000000";   
    }
    else
    {
        // cout << "4" << endl;
        cout << "frame_id = " << frame_id << endl;
        //txt1
        filename1="vertex_000000";
        id=to_string(frame_id);     
        //txt2
        filename2="box_000000";   
    }
    //txt1
    filename1.append(id);
    filename1.append(".txt");
    string filepath1="/home/sugar/catkin_ws/src/euclidean_cluster/data/vertex/";
    filepath1.append(filename1);
    // cout << "filepath1:" << filepath1 << endl;
    std::ofstream out1(filepath1,std::ios::app);
    out1<<fixed<<setprecision(2);
    //txt2
    filename2.append(id);
    filename2.append(".txt");
    string filepath2="/home/sugar/catkin_ws/src/euclidean_cluster/data/box/";
    filepath2.append(filename2);
    // cout << "filepath2:" << filepath2 << endl;
    std::ofstream out2(filepath2,std::ios::app);
    out2<<fixed<<setprecision(2);

    //the old version
    // std::ofstream out1("/home/sugar/catkin_ws/src/euclidean_cluster/data/data1.txt",std::ios::app);
    // out1<<fixed<<setprecision(2);
    // out1 << "\n" << "a new frame" << "frame_id = " << frame_id << "\n";
    // std::ofstream out2("/home/sugar/catkin_ws/src/euclidean_cluster/data/data2.txt",std::ios::app);
    // out2<<fixed<<setprecision(2);
    // out2 << "\n" << "a new frame" << "frame_id = " << frame_id << "\n";

    //PAbox的数量  PAbox_num//可通行区域的包围盒的参数 x y z l w h angle
    for(int i=0;i<n;i++)
    {
        double x=feature_k1[i][0],y=feature_k1[i][1],z=feature_k1[i][7],
                        l=feature_k1[i][2],w=feature_k1[i][3],h=feature_k1[i][4],
                        angle=feature_k1[i][6]; 
        //人工优化
        // if(x<10&&abs(y)<1)
        // {
        //     x=7;
        //     y=-0.1;
        //     z=-0.65;
        //     l=3.3;w=1.9;h=1.6;
        //     angle=0;
        // }
        //clip out the bad box
        // if((l>8)||(w>8)||(l/w>5)||(l*w>12))
        // {
        //     continue;
        // }
        //先列出假设没有旋转时各个顶点的坐标
        //Pl-points' coordinate in the frame of box
        Eigen::Matrix<double, 4, 8> Pl;
        Pl << 0.5*l,-0.5*l,-0.5*l, 0.5*l, 0.5*l,-0.5*l,-0.5*l, 0.5*l,
            0.5*w, 0.5*w,-0.5*w,-0.5*w, 0.5*w, 0.5*w,-0.5*w,-0.5*w,
            0.5*h    , 0.5*h    , 0.5*h    , 0.5*h    ,     -0.5*h,     -0.5*h,     -0.5*h,     -0.5*h,
            1,1,1,1,1,1,1,1;
        // cout << "Pl=" << endl << Pl << endl;     
        //坐标变换矩阵T
        Eigen::Matrix<double, 4, 4> T;
        T << cos(angle),-sin(angle), 0,x,
            sin(angle), cos(angle), 0,y,
                0     ,     0     , 1,z,
                0     ,     0     , 0,1;
        // cout << "T=" << endl << T << endl;
        //矩阵相乘
        //Pw-points world激光雷达的坐标系下的坐标
        Eigen::Matrix<double, 4, 8> Pw;
        Pw=T*Pl;
        // cout << "Pw=" << endl << Pw << endl;
        //every column is a point x y z 1
        for(int j=0;j<8;j++)
        {
            for(int k=0;k<4;k++)
            {
                out1 << Pw(k,j);
                if(k==3)
                {
                    out1 << "\n";
                }
                else
                {
                    out1 << ",";
                }
            }
        }
        out2 << h << "," << w << "," << l << "," << x << "," << y << "," << z << "," << angle << "\n";
    }
    out1.close();
    out2.close();

    // cout << "continue?" << endl;
    // int flag;
    // cin >> flag; 

    //for test 
//     double way_points[3]={1,2,3};
//     std::ofstream out("/home/sugar/catkin_ws/src/euclidean_cluster/test.txt",std::ios::app);
//     out<<fixed<<setprecision(2)<<way_points[0]<<"\t\t"<<way_points[1]<<"\t\t"<<way_points[2]<<std::endl; 
//     out.close();
}

void EuClusterCore::read_txt()
{
    vector<double> vec;
    string value;
    string dot=",";
    std::ifstream myFile;
    myFile.open("/home/sugar/catkin_ws/src/euclidean_cluster/test.txt", std::ios::app);
    if (myFile.is_open())
    {
        std::cout << "File is open." << std::endl;
        while (myFile >> value)
        {   
            //读了一行
            cout << "string=" << value << endl;
            //相等则为0，不等为-1
            if(value.compare(dot)==0)
            {   
                continue;
            }
            else
            {
                //字符串转double
                double num=atof(value.c_str());
                std::cout << "num is " << num << std::endl;
                vec.push_back(num);
            }
        }
    }
}

//update the corr_all
void EuClusterCore::update_corr_all(vector<int> k1_corr)
{   
    // cout << "in the function of update_corr_all" << endl;
    //for test
    // cout << "corr_all[0][0]=" << corr_all[0][0] << endl;
    for(int i=0;i<10;i++)
    {   
        // cout << "i=" << i << endl;
        if(i<9)
        {
            //get the index of two lines
            int index1=18-2*i,index2=16-2*i;
            //get the size of these two frames
            int num1=corr_all[0][index1],num2=corr_all[0][index2];
            // cout << "index1= " << index1 << "   index2=" << index2 << endl;
            // cout << "num1=" << num1 << "    num2=" << num2 << endl;
            //move
            for(int j=0;j<num1+1;j++)
            {   
                corr_all[j][index1]=-1;
                corr_all[j][index1+1]=-1;
            }
            for(int j=0;j<num2+1;j++)
            {
                corr_all[j][index1]=corr_all[j][index2];
                corr_all[j][index1+1]=corr_all[j][index2+1];
            }
        }
        else
        {
            for(int j=0;j<n+1;j++)
            {
                if(j==0)
                {
                    corr_all[j][0]=n;
                    corr_all[j][1]=n;
                }
                else
                {
                    corr_all[j][0]=j-1;
                    corr_all[j][1]=k1_corr[j-1];
                }
            }
        }
    }
    //print the corr_all
    // cout << endl << "print the corr_all " << endl;
    // for(int i=0;i<10;i++)
    // {   
    //     int num=corr_all[0][i];
    //     // cout << "i=" << i << "  num=" << num << endl;
    //     // cout << "a new corr relation" << endl;
    //     for(int j=0;j<num+1;j++)
    //     {
    //         // cout << corr_all[j][i*2] << "  " << corr_all[j][i*2+1] << endl;
    //         cout << corr_all[j][i] << "    ";
    //     }
    //     cout << endl;
    // }
    // cout << "in the end of update_corr_all " << endl;
}

void EuClusterCore::update_corr_chain()
{
    //input corr_all
    //1 step, initilization,the first column -1 means has no corr, 1 means has corr
    for(int i=0;i<n;i++)
    {
        for(int j=0;j<7;j++)
        {
            corr_chain[i][j]=-1;
        }
    }
    //2 step, update
    for(int i=0;i<n;i++)
    {
        int index1=corr_all[i+1][0];
        // cout << "i=" << i << "  index1=" << index1 << endl;
        corr_chain[i][1]=index1;
        for(int j=1;j<=5;j++)
        {   
            int index2=corr_all[index1+1][2*j-1];
            // cout << "i=" << i << "  j=" << j << "   index2=" << index2 << endl;
            if(index2==-1)
            {   
                //we dont need to compute this cluster
                break;
            }
            corr_chain[i][j+1]=index2;
            index1=index2;
            if(j==5)
            {   
                //1 means during all 5 frames have correlation
                corr_chain[i][0]=1;
            }
        }
    }

}

//print corr_chain 
void EuClusterCore::print_corr_chain()
{
    cout << endl << "print corr_chain" << endl;
    for(int i=0;i<n;i++)
    {
        for(int j=0;j<7;j++)
        {
            cout << corr_chain[i][j] << "  ";
        }
        cout << endl;
    }
}

//frames means we use this frame and the last [frames] frames such 5
//计算state向量（速度）
//获得前5帧的平均值作为前第3帧的真实值
void EuClusterCore::corr_chain_speed(int frames)
{
    //the date struct fo feature_last_five x y 
    //x1 means the last one, x5 means the last 5
    // 1 means has corr
    for(int i=0;i<n;i++)
    {   
        if(corr_chain[i][0]==1)
        {
            // cout << "i=" << i << endl;
            int index1=corr_chain[i][2],
                index2=corr_chain[i][3],
                index3=corr_chain[i][4],
                index4=corr_chain[i][5],
                index5=corr_chain[i][6];
            // cout << "index1=" << index1 << "    index2=" << index2<< "  index3=" 
            //         << index3<< "   index4=" << index4<< "  index5=" << index5<<endl;
            double x1=feature_last_five[0][index1][0],y1=feature_last_five[0][index1][1],
            x2=feature_last_five[1][index2][0], y2=feature_last_five[1][index2][1],
            x3=feature_last_five[2][index3][0],y3=feature_last_five[2][index3][1],
            x4=feature_last_five[3][index4][0],y4=feature_last_five[3][index4][1],
            x5=feature_last_five[4][index5][0],y5=feature_last_five[4][index5][1];
            // cout << "x1=" << x1 << "    y1=" << y1
            //         << "    x2=" << x2 << "    y2=" << y2 
            //         << "    x3=" << x3 << "    y3=" << y3 
            //         << "    x4=" << x4 << "    y4=" << y4 
            //         << "    x5=" << x5 << "    y5=" << y5  << endl; 
            //data of this frame 
            double x=feature_k1[i][0],y=feature_k1[i][1];
            // cout << "x=" << x << "  y=" << y << endl;
            //use the average of last 5 to represent the last 3rd frame
            double xave=(x1+x2+x3+x4+x5)/5,yave=(y1+y2+y3+y4+y5)/5;
            // cout << "xave=" << xave << "    yave=" << yave << endl;
            //use this xave yave and current frame's x y to compute the velocity
            double T=0.1;
            double vx=(x-xave)/(3*T),vy=(y-yave)/(3*T);
            double v=sqrt(vx*vx+vy*vy),theta=atan2(vy,vx);
            // cout << "v=" << v <<"   theta=" << theta << endl;
            //update state matrix
            state[i][0]=x;state[i][1]=y;state[i][2]=v;state[i][3]=theta;
        }
        else
        {
            double x=feature_k1[i][0],y=feature_k1[i][1];
            state[i][0]=x;state[i][1]=y;state[i][2]=0;state[i][3]=0;
        }
    }
    //pritn state
    // cout << endl << "print state " << endl;
    // for(int i=0;i<n;i++)
    // {
    //     for(int j=0;j<4;j++)
    //     {
    //         cout << state[i][j] << "    ";
    //     }
    //     cout << endl;
    // }
}

//update the feature_last_five at the end of each frame
void EuClusterCore::update_feature_last_five()
{   
    // cout << "in the function of update_feature_last_five " << endl;
    //whate we need is feature_k1, n,cate_num;
    //the first step,update the cate_num[][]
    for(int i=4;i>0;i--)
    {
        cate_num[i]=cate_num[i-1];
    }
    // cout << "1" << endl;
    cate_num[0]=n;
    //the second step,update the feature_last_five
    for(int i=4;i>0;i--)
    {   
        //at this time the cate_num has been updated
        int num=cate_num[i];
        for(int j=0;j<num;j++)
        {   
            //现在一共15个特征
            for(int k=0;k<15;k++)
            {
                feature_last_five[i][j][k]=feature_last_five[i-1][j][k];
            }
        }
    }
    // cout << "2" << endl;
    //
    for(int j=0;j<n;j++)
    {
        for(int k=0;k<15;k++)
        {
            feature_last_five[0][j][k]=feature_k1[j][k];
        }
    }
    // cout << "3" << endl;
}

//print feature_last_five
void EuClusterCore::print_feature_last_five()
{
    cout << endl << "print feature_last_five " << endl;
    for(int i=0;i<5;i++)
    {   
        cout << "i=" << i << endl;
        int num=cate_num[i];
        for(int j=0;j<num;j++)
        {
            for(int k=0;k<8;k++)
            {
                cout << feature_last_five[i][j][k] << "     ";
            }
            cout << endl;
        }
    }
}


void EuClusterCore::print_information(vector<int> k1_corr)
{   
    //print k1_corr
    // cout << "print k1_corr" << endl;
    // cout << "n=" << n << endl;
    // for(int i=0;i<n;i++)
    // {
    //     cout << i << "==" << k1_corr[i] << endl;
    // }
    
    //print feature_k1
    // cout << "print feature_k1 " << endl;
    // for(int i=0;i<n;i++)
    // {
    //     for(int j=0;j<8;j++)
    //     {
    //         cout << feature_k1[i][j] << "     ";
    //     }
    //     cout << endl;
    // }
}

//use a function to use pub the speed arrows
void EuClusterCore::publish_speed_arrow(vector<vector<double> > G)
{   
    // cout << "in the publish_speed_arrow  " << endl;
    // cout << "print G" << endl;
    // for(int i=0;i<n;i++)
    // {   
    //     cout << i << "  ";
    //     for(int j=0;j<4;j++)
    //     {
    //         cout << G[i][j] << "    ";
    //     }
    //     cout << endl;
    // }
    //画箭头的一种新的写法
    //这种way可以销毁之前的箭头，其实就是让它透明
    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.clear();
    visualization_msgs::Marker arrow_marker;
    arrow_marker.header.frame_id = str_frame;
    arrow_marker.header.stamp = ros::Time::now();
    arrow_marker.ns = "speed_arrow";
    arrow_marker.color.r = 1.0f;
    arrow_marker.color.g = 0.0f;
    arrow_marker.color.b = 0.0f;
    arrow_marker.color.a = 1.0;
    arrow_marker.lifetime = ros::Duration();
    arrow_marker.frame_locked = true;
    arrow_marker.type = visualization_msgs::Marker::ARROW;
    arrow_marker.action = visualization_msgs::Marker::ADD;
    //makrer_id用于计数
    int marker_id = 0;
    //n表示需要画的箭头数量
    for (size_t i = 0; i < n; ++i)
    {
    arrow_marker.id = marker_id;
    arrow_marker.color.a = 1;
    arrow_marker.pose.position.x = G[i][0];
    arrow_marker.pose.position.y = G[i][1];
    //
    double length = output_data[i][2];
    double width = output_data[i][3];
    // if((length>3)||(width>3))
    // {
    //     arrow_marker.color.a = 0;
    // }
    arrow_marker.pose.position.z = feature_k1[i][7];
    // cout << "i=" << i << "  arrow_marker.pose.position.z=" << arrow_marker.pose.position.z << endl;
    //如果速度为零，不显示
    if(G[i][2]==0)
    {
        arrow_marker.color.a = 0;
        arrow_marker.scale.x = 0.1;
    }
    else
    {
        double speed_scale=1;
        double length=G[i][2]/speed_scale;
        // if(length>3)
        // {
        //     length=3;
        // }
        arrow_marker.scale.x = length;
    }
    arrow_marker.scale.y = 0.3;
    arrow_marker.scale.z = 0.3;
    //角度问题
    double theta=G[i][3];
    Quaternion q=ToQuaternion(theta,0,0);;
    double w=q.w,x=q.x,y=q.y,z=q.z;
    arrow_marker.pose.orientation.w = w;
    arrow_marker.pose.orientation.x = x;
    arrow_marker.pose.orientation.y = y;
    arrow_marker.pose.orientation.z = z;
    //压入
    marker_array.markers.push_back(arrow_marker);
    ++marker_id;
    }
    //arrow_max表示至今为止最多的箭头数量
    if (marker_array.markers.size() > arrow_max)
    {
    arrow_max = marker_array.markers.size();
    }
    //对于多余的不需要显示的，进行透明化
    for (size_t i = marker_id; i < arrow_max; ++i)
    {
    arrow_marker.id = i;
    arrow_marker.color.a = 0;
    arrow_marker.pose.position.x = 0;
    arrow_marker.pose.position.y = 0;
    arrow_marker.pose.position.z = 0;
    arrow_marker.scale.x = 0.1;
    arrow_marker.scale.y = 0.1;
    arrow_marker.scale.z = 0.1;
    marker_array.markers.push_back(arrow_marker);
    ++marker_id;
    }
    markerArrayPub.publish(marker_array);
    //箭头输出结束
}

//output_data的数据格式
////x  y z  l  w  h  angle  v  theta  angle-box  theta-move
void EuClusterCore::output_data_make()
{   
    //PA_box is the passerable box  x y z l w h angle 
    //box_V2 is the new cluster box
    //state is the new state vector
    // cout << "in the function of output data make " << endl;
    // cout << "PAbox_num=" << PAbox_num << endl;
    //the num of PAbox is not n
    for(int i=0;i<PAbox_num;i++)
    {
        // cout << "i=" << i << endl;
        output_data[i][0]=PAbox_data[i][0];   //x
        output_data[i][1]=PAbox_data[i][1];   //y
        output_data[i][2]=PAbox_data[i][2];   //z
        output_data[i][3]=PAbox_data[i][3];   //l
        output_data[i][4]=PAbox_data[i][4];   //w
        output_data[i][5]=PAbox_data[i][5];   //h
        output_data[i][6]=PAbox_data[i][6];   //angle - dirction of box
        output_data[i][7]=state[i][2];   //v
        output_data[i][8]=state[i][3];   //theta
    }
    //如果是存cbox用这个
    // for(int i=0;i<n;i++)
    // {
    //     // cout << "i=" << i << endl;
    //     //feature_k1  
    //     //x y l w h s angle z num xlcose yclose xthetamax ythetamax xthetamin ythetamin
    //     output_data[i][0]=feature_k1[i][0];   //x
    //     output_data[i][1]=feature_k1[i][1];   //y
    //     output_data[i][2]=feature_k1[i][7];   //z
    //     output_data[i][3]=feature_k1[i][2];   //l
    //     output_data[i][4]=feature_k1[i][3];   //w
    //     output_data[i][5]=feature_k1[i][4];   //h
    //     output_data[i][6]=feature_k1[i][6];   //angle - dirction of box
    //     output_data[i][7]=state[i][2];   //v
    //     output_data[i][8]=state[i][3];   //theta
    // }
    // cout << "out the function of output data make " << endl;
}

//data source: output_data
void EuClusterCore::publish_rs_msgs()
{
    //根据output_data发布障碍物信息
    //先定义一个list
    // cout << "根据output_data发布障碍物信息" << endl;
    rs_perception::PerceptionListMsg rs_msg_array;
    //再定义每个单独的msg
    rs_perception::PerceptionMsg rs_msg;
    //循环赋值
    for(int i=0;i<n;i++)
    {  
        // cout << "i=" << i << endl;
        //output data x y z l w h angle v theta
        double l=output_data[i][3],w=output_data[i][4];
        //if l is too short or w is too short, don't publish
        if((l<0.3)||(w<0.3))
        {
            continue;
        }
        double time =ros::Time::now().toSec(); //把时间戳转化成浮点型格式
        rs_msg.timestamp = time;
        //位置
        // cout << "output_data[i][0]=" << output_data[i][0] 
        //      << " output_data[i][1]=" << output_data[i][1] << endl;
        rs_msg.location.x=output_data[i][0];
        rs_msg.location.y=output_data[i][1];
        rs_msg.location.z=output_data[i][2];
        //方向
        rs_msg.direction.x=1;
        rs_msg.direction.y=1;
        rs_msg.direction.z=1;
        //
        rs_msg.yaw=output_data[i][6];
        rs_msg.length=output_data[i][3];
        rs_msg.width=output_data[i][4];
        rs_msg.height=output_data[i][5];
        //添加到array中
        rs_msg_array.perceptions.push_back(rs_msg);
    }
    //发布
    rs_msg_array.header=point_cloud_header_;
    pub_rs_msgs.publish(rs_msg_array);
}
//data source:PAbox_data
void EuClusterCore::publish_PAbox_array()
{
    visualization_msgs::MarkerArray PAbox_array;
    PAbox_array.markers.clear();
    visualization_msgs::Marker PAbox_marker;
    // PAbox_marker.header.frame_id = "rslidar";
    PAbox_marker.header.frame_id = str_frame;
    //century cup
    // PAbox_marker.header.frame_id = "lidar_link";
    //kitti测试数据
    // PAbox_marker.header.frame_id = "velo_link";
    PAbox_marker.header.stamp = ros::Time::now();
    PAbox_marker.ns = "PAbox_array";
    PAbox_marker.color.r = 0.0f;
    PAbox_marker.color.g = 0.0f;
    PAbox_marker.color.b = 1.0f;
    PAbox_marker.color.a = 0.5;
    PAbox_marker.lifetime = ros::Duration();
    PAbox_marker.frame_locked = true;
    PAbox_marker.type = visualization_msgs::Marker::CUBE;
    PAbox_marker.action = visualization_msgs::Marker::ADD;
    //makrer_id用于计数
    int marker_PAbox_id = 0;
    //n表示所有的类别数
    for (size_t i = 0; i < PAbox_num; ++i)
    {
        PAbox_marker.id = marker_PAbox_id;
        PAbox_marker.color.a = 0.5;
        PAbox_marker.pose.position.x = PAbox_data[i][0];
        PAbox_marker.pose.position.y = PAbox_data[i][1];
        PAbox_marker.pose.position.z = PAbox_data[i][2];
        PAbox_marker.scale.x = PAbox_data[i][3];
        PAbox_marker.scale.y = PAbox_data[i][4];
        PAbox_marker.scale.z = PAbox_data[i][5];
        //角度问题
        double angle=PAbox_data[i][6];
        Quaternion q=ToQuaternion(angle,0,0);;
        double w=q.w,x=q.x,y=q.y,z=q.z;
        PAbox_marker.pose.orientation.w = w;
        PAbox_marker.pose.orientation.x = x;
        PAbox_marker.pose.orientation.y = y;
        PAbox_marker.pose.orientation.z = z;
        //压入
        PAbox_array.markers.push_back(PAbox_marker);
        ++marker_PAbox_id;
    }
    //PAbox_max表示至今为止最多的盒子数量
    if (PAbox_array.markers.size() > PAbox_max)
    {
        PAbox_max = PAbox_array.markers.size();
    }
    //对于多余的不需要显示的，进行透明化
    for (size_t i = marker_PAbox_id; i < PAbox_max; ++i)
    {
        PAbox_marker.id = i;
        PAbox_marker.color.a = 0;
        PAbox_marker.pose.position.x = 0;
        PAbox_marker.pose.position.y = 0;
        PAbox_marker.pose.position.z = 0;
        PAbox_marker.scale.x = 0.1;
        PAbox_marker.scale.y = 0.1;
        PAbox_marker.scale.z = 0.1;
        PAbox_array.markers.push_back(PAbox_marker);
        ++marker_PAbox_id;
    }
    markerPAboxPub.publish(PAbox_array);
    //把PAbox_num赋值为0
    PAbox_num=0;
}
//data source: feature_k1
void EuClusterCore::publish_cbox_array()
{   
    cout << "in the function of publish_cbox_array" << endl;
    //创建一个包围盒子用于绘制类别包围盒，不同于之前的通行区域包围盒子
    visualization_msgs::MarkerArray marker_cbox_array;
    marker_cbox_array.markers.clear();
    visualization_msgs::Marker cbox_marker;
    // cbox_marker.header.frame_id = "rslidar";
    cbox_marker.header.frame_id = str_frame;
    //century cup
    // cbox_marker.header.frame_id = "lidar_link";
    cbox_marker.header.stamp = ros::Time::now();
    cbox_marker.ns = "cbox_array";
    cbox_marker.color.r = 0.0f;
    cbox_marker.color.g = 1.0f;
    cbox_marker.color.b = 0.0f;
    cbox_marker.color.a = 1.0;
    cbox_marker.lifetime = ros::Duration();
    cbox_marker.frame_locked = true;
    cbox_marker.type = visualization_msgs::Marker::CUBE;
    cbox_marker.action = visualization_msgs::Marker::ADD;
    //makrer_id用于计数
    int marker_cbox_id = 0;
    //n表示所有的类别数
    for (size_t i = 0; i < n; ++i)
    {   
        cbox_marker.id = marker_cbox_id;
        cbox_marker.color.a = 0.5;
        //output - PAbox  feature_k1 x y  l w h s angle z
        cbox_marker.pose.position.x = feature_k1[i][0];
        cbox_marker.pose.position.y = feature_k1[i][1];
        cbox_marker.pose.position.z = feature_k1[i][7];       //the data struct of feature_k1 
        double length = feature_k1[i][2];
        double width = feature_k1[i][3];
        double height = feature_k1[i][4];
        //角度问题
        double angle=feature_k1[i][6];
        //人工优化
        // if(feature_k1[i][0]<10&&abs(feature_k1[i][1])<1)
        // {
        //     cbox_marker.pose.position.x=7;
        //     cbox_marker.pose.position.y=-0.1;
        //     cbox_marker.pose.position.z=-0.65;
        //     length=3.3;width=1.9;height=1.6;
        //     angle=0;
        // }
        //
        cbox_marker.scale.x = length;
        cbox_marker.scale.y = width;
        cbox_marker.scale.z = height;
        // cout << "i=" << i << "  x=" << feature_k1[i][0] << "    y=" << feature_k1[i][1]  
        //           << "  z=" << feature_k1[i][7]  
        //           << "  l=" << length << "  w=" << width << " h=" << height 
        //           << " angle=" << feature_k1[i][6] << endl;
        Quaternion q=ToQuaternion(angle,0,0);;
        double w=q.w,x=q.x,y=q.y,z=q.z;
        cbox_marker.pose.orientation.w = w;
        cbox_marker.pose.orientation.x = x;
        cbox_marker.pose.orientation.y = y;
        cbox_marker.pose.orientation.z = z;
        //
        // if((length>8)||(width>8)||(length/width>5)||(length*width>12))
        // {
        //     cbox_marker.color.a = 0;
        // }
        //压入
        marker_cbox_array.markers.push_back(cbox_marker);
        ++marker_cbox_id;
    }
    //cbox_max表示至今为止最多的盒子数量
    if (marker_cbox_array.markers.size() > cbox_max)
    {
        cbox_max = marker_cbox_array.markers.size();
    }
    //对于多余的不需要显示的，进行透明化
    for (size_t i = marker_cbox_id; i < cbox_max; ++i)
    {
        cbox_marker.id = i;
        cbox_marker.color.a = 0;
        cbox_marker.pose.position.x = 0;
        cbox_marker.pose.position.y = 0;
        cbox_marker.pose.position.z = 0;
        cbox_marker.scale.x = 0.1;
        cbox_marker.scale.y = 0.1;
        cbox_marker.scale.z = 0.1;
        marker_cbox_array.markers.push_back(cbox_marker);
        ++marker_cbox_id;
    }
    markerCboxPub.publish(marker_cbox_array);
    //cbox输出结束
}

//data source: feature_k1,打印当前帧与上一帧的对应
// void EuClusterCore::publish_index_array(vector<int> k1_corr)
// {
//     //用来标记编号用
//     visualization_msgs::MarkerArray marker_index_array;
//     marker_index_array.markers.clear();
//     visualization_msgs::Marker index_marker;
//     // index_marker.header.frame_id = "rslidar";
//     index_marker.header.frame_id = str_frame;
//     //century cup
//     // index_marker.header.frame_id = "lidar_link";
//     index_marker.header.stamp = ros::Time::now();
//     index_marker.ns = "index_array";
//     //字用白色
//     index_marker.color.r = 1.0f;
//     index_marker.color.g = 1.0f;
//     index_marker.color.b = 1.0f;
//     index_marker.color.a = 1;
//     index_marker.lifetime = ros::Duration();
//     index_marker.frame_locked = true;
//     index_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
//     index_marker.action = visualization_msgs::Marker::ADD;
//     //makrer_id用于计数
//     int marker_index_id = 0;
//     //n表示所有的类别数
//     for (size_t i = 0; i < n; ++i)
//     {
//         index_marker.id = marker_index_id;
//         double x=feature_k1[i][0],y=feature_k1[i][1],z=feature_k1[i][7],
//                         l=feature_k1[i][2],w=feature_k1[i][3],h=feature_k1[i][4];
//         index_marker.pose.position.x = x;
//         index_marker.pose.position.y = y;
//         index_marker.pose.position.z = z+0.5*h+0.1;
//         //修改字体的大小
//         index_marker.scale.x = 0.75;
//         index_marker.scale.y = 0.75;
//         index_marker.scale.z = 2;
//         //文字信息
//         ostringstream str;
//         // str<<i;
//         str<<i<<"=="<<k1_corr[i];
//         index_marker.text=str.str();
//         //角度问题
//         index_marker.pose.orientation.w = 0;
//         index_marker.pose.orientation.x = 0;
//         index_marker.pose.orientation.y = 0;
//         index_marker.pose.orientation.z = 0;
//         //压入
//         marker_index_array.markers.push_back(index_marker);
//         ++marker_index_id;
//     }
//     //index_max表示至今为止最多的盒子数量
//     if (marker_index_array.markers.size() > index_max)
//     {
//     index_max = marker_index_array.markers.size();
//     }
//     //对于多余的不需要显示的，进行透明化
//     for (size_t i = marker_index_id; i < index_max; ++i)
//     {
//     index_marker.id = i;
//     index_marker.color.a = 0;
//     index_marker.pose.position.x = 0;
//     index_marker.pose.position.y = 0;
//     index_marker.pose.position.z = 0;
//     index_marker.scale.x = 0.1;
//     index_marker.scale.y = 0.1;
//     index_marker.scale.z = 0.1;
//     marker_index_array.markers.push_back(index_marker);
//     ++marker_index_id;
//     }
//     //
//     markerIndexPub.publish(marker_index_array);
// }

//data source: feature_k1,打印当前帧
void EuClusterCore::publish_index_array()
{
    //用来标记编号用
    visualization_msgs::MarkerArray marker_index_array;
    marker_index_array.markers.clear();
    visualization_msgs::Marker index_marker;
    // index_marker.header.frame_id = "rslidar";
    index_marker.header.frame_id = str_frame;
    //century cup
    // index_marker.header.frame_id = "lidar_link";
    index_marker.header.stamp = ros::Time::now();
    index_marker.ns = "index_array";
    //字用白色
    index_marker.color.r = 1.0f;
    index_marker.color.g = 1.0f;
    index_marker.color.b = 1.0f;
    index_marker.color.a = 1;
    index_marker.lifetime = ros::Duration();
    index_marker.frame_locked = true;
    index_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    index_marker.action = visualization_msgs::Marker::ADD;
    //makrer_id用于计数
    int marker_index_id = 0;
    //n表示所有的类别数
    for (size_t i = 0; i < n; ++i)
    {
        index_marker.id = marker_index_id;
        double x=feature_k1[i][0],y=feature_k1[i][1],z=feature_k1[i][7],
                        l=feature_k1[i][2],w=feature_k1[i][3],h=feature_k1[i][4];
        index_marker.pose.position.x = x;
        index_marker.pose.position.y = y;
        index_marker.pose.position.z = z+0.5*h+0.1;
        //修改字体的大小
        index_marker.scale.x = 0.75;
        index_marker.scale.y = 0.75;
        index_marker.scale.z = 2;
        //文字信息
        ostringstream str;
        // str<<i;
        str<<i;
        index_marker.text=str.str();
        //角度问题
        index_marker.pose.orientation.w = 0;
        index_marker.pose.orientation.x = 0;
        index_marker.pose.orientation.y = 0;
        index_marker.pose.orientation.z = 0;
        //压入
        marker_index_array.markers.push_back(index_marker);
        ++marker_index_id;
    }
    //index_max表示至今为止最多的盒子数量
    if (marker_index_array.markers.size() > index_max)
    {
    index_max = marker_index_array.markers.size();
    }
    //对于多余的不需要显示的，进行透明化
    for (size_t i = marker_index_id; i < index_max; ++i)
    {
    index_marker.id = i;
    index_marker.color.a = 0;
    index_marker.pose.position.x = 0;
    index_marker.pose.position.y = 0;
    index_marker.pose.position.z = 0;
    index_marker.scale.x = 0.1;
    index_marker.scale.y = 0.1;
    index_marker.scale.z = 0.1;
    marker_index_array.markers.push_back(index_marker);
    ++marker_index_id;
    }
    //
    
    markerIndexPub.publish(marker_index_array);
}


//用state发布速度箭头
void EuClusterCore::publish_speed_arrow_v2()
{   
    // cout << "in the publish_speed_arrow_v2  " << endl;
    //画箭头的一种新的写法
    //这种way可以销毁之前的箭头，其实就是让它透明
    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.clear();
    visualization_msgs::Marker arrow_marker;
    // arrow_marker.header.frame_id = "rslidar";
    arrow_marker.header.frame_id = str_frame;
    //century cup
    // arrow_marker.header.frame_id = "lidar_link";
    arrow_marker.header.stamp = ros::Time::now();
    arrow_marker.ns = "speed_arrow_v2";
    arrow_marker.color.r = 0.0f;
    arrow_marker.color.g = 0.0f;
    arrow_marker.color.b = 1.0f;
    arrow_marker.color.a = 0.2;
    arrow_marker.lifetime = ros::Duration();
    arrow_marker.frame_locked = true;
    arrow_marker.type = visualization_msgs::Marker::ARROW;
    arrow_marker.action = visualization_msgs::Marker::ADD;
    //makrer_id用于计数
    int marker_id = 0;
    //n表示需要画的箭头数量
    for (size_t i = 0; i < n; ++i)
    {
    arrow_marker.id = marker_id;
    arrow_marker.color.a = 1;
    arrow_marker.pose.position.x = state[i][0];
    arrow_marker.pose.position.y = state[i][1];
    //
    double length = output_data[i][2];
    double width = output_data[i][3];
    arrow_marker.pose.position.z = feature_k1[i][7];
    // cout << "i=" << i << "  arrow_marker.pose.position.z=" << arrow_marker.pose.position.z << endl;
    //如果速度为零，不显示;点太少认为速度估计没有意义
    if(state[i][2]==0||feature_k1[i][8]<30)
    {
        arrow_marker.color.a = 0;
        arrow_marker.scale.x = 0.1;
    }
    else
    {
        double speed_scale=1;
        double length=state[i][2]/speed_scale;
        arrow_marker.scale.x = length;
    }
    arrow_marker.scale.y = 0.3;
    arrow_marker.scale.z = 0.3;
    //角度问题
    double theta=state[i][3];
    Quaternion q=ToQuaternion(theta,0,0);;
    double w=q.w,x=q.x,y=q.y,z=q.z;
    arrow_marker.pose.orientation.w = w;
    arrow_marker.pose.orientation.x = x;
    arrow_marker.pose.orientation.y = y;
    arrow_marker.pose.orientation.z = z;
    //压入
    marker_array.markers.push_back(arrow_marker);
    ++marker_id;
    }
    //arrow_max表示至今为止最多的箭头数量
    if (marker_array.markers.size() > arrow_max)
    {
    arrow_max = marker_array.markers.size();
    }
    //对于多余的不需要显示的，进行透明化
    for (size_t i = marker_id; i < arrow_max; ++i)
    {
    arrow_marker.id = i;
    arrow_marker.color.a = 0;
    arrow_marker.pose.position.x = 0;
    arrow_marker.pose.position.y = 0;
    arrow_marker.pose.position.z = 0;
    arrow_marker.scale.x = 0.1;
    arrow_marker.scale.y = 0.1;
    arrow_marker.scale.z = 0.1;
    marker_array.markers.push_back(arrow_marker);
    ++marker_id;
    }
    markerArrayPub_v2.publish(marker_array);
    //箭头输出结束
}

void EuClusterCore::compute_vertex()
{
    // //the corner of box
    // cout << "计算每个聚类的包围盒子的顶点的坐标" << endl;
    // vector<vector<double> > vertexs;
    // //初始化，开辟区域
    // for(int i=0;i<n;i++)
    // {
    //     vector<double> temp;
    //     //8个顶点，每个顶点3个坐标值，一共24
    //     for(int j=0;i<24;i++)
    //     {
    //         temp.push_back(0);
    //     }
    //     vertexs.push_back(temp);
    // }
    // //计算
    // for(int i=0;i<n;i++)
    // {   
    //     cout << "类别号索引i=" << i << endl;
    //     vector<double> temp;
    //     //获取矩形的参数
    //     //特征向量的数据结构 x y l w h s angle z
    //     double x=feature_k1[i][0],y=feature_k1[i][1],z=feature_k1[i][7],
    //            l=feature_k1[i][2],w=feature_k1[i][3],h=feature_k1[i][4],
    //            angle=feature_k1[i][6];
    //     //如果这个类在相机的前方才需要进行坐标变换
    //     if(x>0)
    //     {
    //         //先列出假设没有旋转时各个顶点的坐标
    //         //Pl-points local局部坐标系下的坐标
    //         Eigen::Matrix<double, 4, 8> Pl;
    //         Pl << 0.5*l,-0.5*l,-0.5*l, 0.5*l, 0.5*l,-0.5*l,-0.5*l, 0.5*l,
    //             0.5*w, 0.5*w,-0.5*w,-0.5*w, 0.5*w, 0.5*w,-0.5*w,-0.5*w,
    //             0    , 0    , 0    , 0    ,     h,     h,     h,     h,
    //             1,1,1,1,1,1,1,1;
    //         // cout << "Pl=" << endl << Pl << endl;     
    //         //坐标变换矩阵T
    //         Eigen::Matrix<double, 4, 4> T;
    //         T << cos(angle),-sin(angle), 0,x,
    //             sin(angle), cos(angle), 0,y,
    //                 0     ,     0     , 1,z,
    //                 0     ,     0     , 0,1;
    //         // cout << "T=" << endl << T << endl;
    //         //矩阵相乘
    //         //Pw-points world激光雷达的坐标系下的坐标
    //         Eigen::Matrix<double, 4, 8> Pw;
    //         Pw=T*Pl;
    //         cout << "Pw=" << endl << Pw << endl;
    //         //矩阵中单个元素 A(0,0)
    //         //前三列存储到顶点中
    //         for(int j=0;j<8;j++)
    //         {
    //             for(int k=0;k<3;k++)
    //             {   
    //                 temp.push_back(Pw(k,j));
    //             }
    //         }
    //         vertexs.push_back(temp);
    //         //求出了八个顶点的坐标之后，需要转移到相机坐标
    //         //相机约定 0-左侧灰度（参考相机）1-右侧灰度 2-左侧彩色 3-右侧彩色
    //         //转移到左侧彩色
    //         cout << "激光雷达包围盒顶点转化为相机像素点" << endl;
    //         //激光雷达中包围盒的八个顶点转换到左侧彩色相机
    //         Eigen::Matrix<double, 3, 4> P_rect_02; //相机0到相机2
    //         Eigen::Matrix<double, 4, 4> R_rect_00; //矫正相机，使图像平面共面，拓展成4*4
    //         Eigen::Matrix<double, 4, 4> Tr_velo_to_cam; //雷达到相机，4*4矩阵
    //         // P_rect_02: 
    //         // 7.188560e+02 0.000000e+00 6.071928e+02 4.538225e+01 
    //         // 0.000000e+00 7.188560e+02 1.852157e+02 -1.130887e-01 
    //         // 0.000000e+00 0.000000e+00 1.000000e+00 3.779761e-03
    //         P_rect_02 << 7.188560e+02, 0.000000e+00, 6.071928e+02, 4.538225e+01, 
    //                     0.000000e+00, 7.188560e+02, 1.852157e+02, -1.130887e-01, 
    //                     0.000000e+00, 0.000000e+00, 1.000000e+00, 3.779761e-03;
    //         // R_rect_00: 3*3拓展成4*4需要补0
    //         // 9.999454e-01 7.259129e-03 -7.519551e-03 
    //         // -7.292213e-03 9.999638e-01 -4.381729e-03 
    //         // 7.487471e-03 4.436324e-03 9.999621e-01
    //         R_rect_00 << 9.999454e-01, 7.259129e-03, -7.519551e-03, 0,
    //                     -7.292213e-03, 9.999638e-01, -4.381729e-03, 0,
    //                     7.487471e-03, 4.436324e-03,  9.999621e-01, 0,
    //                     0           , 0           ,  0           , 1;
    //         //Tr_velo_to_cam
    //         // R: 
    //         // 7.967514e-03 -9.999679e-01 -8.462264e-04 
    //         // -2.771053e-03 8.241710e-04 -9.999958e-01 
    //         // 9.999644e-01 7.969825e-03 -2.764397e-03
    //         // T: 
    //         // -1.377769e-02 -5.542117e-02 -2.918589e-01
    //         Tr_velo_to_cam << 7.967514e-03, -9.999679e-01, -8.462264e-04, -1.377769e-02,
    //                         -2.771053e-03,  8.241710e-04, -9.999958e-01, -5.542117e-02,
    //                         9.999644e-01,  7.969825e-03, -2.764397e-03, -2.918589e-01,
    //                         0           ,  0           ,  0           ,  1;
    //         //Pw为输出的激光雷达坐标系中的8个盒子顶点的坐标
    //         // Eigen::Matrix<double, 4, 8> Pw;
    //         // Pw <<  19.3748,19.3688,18.633,18.639,19.3748,19.3688,18.633,18.639,
    //         //     -1.85381,-1.65481,-1.677,-1.876,-1.85381,-1.65481,-1.677,-1.876,
    //         //     -1.466,-1.466,-1.466,-1.466,-1.352,-1.352,-1.352,-1.352,
    //         //     1,    1,    1,   1,   1,   1, 1,1;
    //         //定义一个数组来存储所有的u和v
    //         double Y_array[2][8];
    //         for(int i=0;i<8;i++)
    //         {   
    //             cout << endl << "点" << i << endl;
    //             Eigen::Matrix<double, 4, 1> X;    //输入的激光雷达的坐标点 x-y-z-1
    //             Eigen::Matrix<double, 3, 1> Y;    //输出的像素点 u-v-1
    //             for(int j=0;j<3;j++)
    //             {
    //                 X(j)=Pw(j,i);
    //             }
    //             cout << "打印输入向量" << endl;
    //             cout << "X=" << endl << X << endl;
    //             //Y=P_rect_02*R_rect_00*(R|T)*X
    //             Y=P_rect_02*R_rect_00*Tr_velo_to_cam*X;  //u-v-w u-v需要除以w取整
    //             double u=Y(0);
    //             double v=Y(1);
    //             double w=Y(2);
    //             Y(0)=(int)(u/w);
    //             Y(1)=(int)(v/w);
    //             Y(2)=1;
    //             cout << "打印输出向量" << endl;
    //             cout << "Y=" << endl << Y << endl;
    //             //将所有结果存储到一个数组当中
    //             Y_array[0][i]=Y(0);
    //             Y_array[1][i]=Y(1);
    //         }
    //         //获取u和v的最大最小值
    //         double umin=Y_array[0][0],umax=Y_array[0][0],vmin=Y_array[1][0],vmax=Y_array[1][0];
    //         for(int i=0;i<8;i++)
    //         {
    //             double u=Y_array[0][i],v=Y_array[1][i];
    //             if(umin>u)
    //             {
    //                 umin=u;
    //             }
    //             if(umax<u)
    //             {
    //                 umax=u;
    //             }
    //             if(vmin>v)
    //             {
    //                 vmin=v;
    //             }
    //             if(vmax<v)
    //             {
    //                 vmax=v;
    //             }
    //         }
    //         cout << "打印最终图像的边界" << endl;
    //         cout << "umin=" << umin << "  umax=" << umax 
    //              << "  vmin=" << vmin << "  vmax=" << vmax << endl;
    //         //整段复制
    //     }
    // }
}

//根据矩形参数计算哪个顶点距离原点最近
//输入参数：中心x，中心y，长l，宽w，角度angle
double* EuClusterCore::closest_vertex_box(double x,double y,double l,double w,double angle)
{   
    // cout << "in closest_vertex_box" << endl;
    // cout << "x=" << x << "  y=" << y << "   l=" << l << "   w=" << w << "   angle=" << angle << endl;
    double k=tan(angle);
    //p is the center of box
    double p[2],l_vector[2],w_vector[2];
    p[0]=x;
    p[1]=y;
    l_vector[0]=0.5*l/sqrt(k*k+1);
    l_vector[1]=0.5*l*k/sqrt(k*k+1);
    w_vector[0]=-0.5*w*k/sqrt(k*k+1);
    w_vector[1]=0.5*w/sqrt(k*k+1);
    //compute the four vectexs of the rectangle A B C D
    //计算矩形的四个顶点  4行2列，x-y
    double P[4][2];
    P[0][0]=p[0]+l_vector[0]+w_vector[0];
    P[0][1]=p[1]+l_vector[1]+w_vector[1];
    P[1][0]=p[0]-l_vector[0]+w_vector[0];
    P[1][1]=p[1]-l_vector[1]+w_vector[1];
    P[2][0]=p[0]-l_vector[0]-w_vector[0];
    P[2][1]=p[1]-l_vector[1]-w_vector[1];
    P[3][0]=p[0]+l_vector[0]-w_vector[0];
    P[3][1]=p[1]+l_vector[1]-w_vector[1];
    // cout << "test1" << endl;
    // cout << "P" << endl;
    // for(int num=0;num<4;num++)
    // {
    //     cout << P[num][0] << "    " << P[num][1] << endl;
    // }
    //计算谁距离原点最近,计算结果为P_close[2]
    double DIS_O[4];
    DIS_O[0]=sqrt(P[0][0]*P[0][0]+P[0][1]*P[0][1]);
    DIS_O[1]=sqrt(P[1][0]*P[1][0]+P[1][1]*P[1][1]);
    DIS_O[2]=sqrt(P[2][0]*P[2][0]+P[2][1]*P[2][1]);
    DIS_O[3]=sqrt(P[3][0]*P[3][0]+P[3][1]*P[3][1]);
    // cout << "test2" << endl;
    //1-A,2-B,3-C,4-D
    int flag=0;
    for(int j=0;j<n;j++)
    {
        if(DIS_O[j]<DIS_O[flag])
        {
            flag=j;
        }
    }
    //不然程序没法运行
    static double Vertex_close[2];
    Vertex_close[0]=P[flag][0];
    Vertex_close[1]=P[flag][1];
    // cout << "结果:" << Vertex_close[0] << "  " << Vertex_close[1] << endl;
    
    // cout << "function over" << endl;

    return Vertex_close;
}

//input state(speed and theta), and period time T , info of bb feature_k1
//state 5 frames 
//we should use box_V2 instead of feature_k
void EuClusterCore::kalman_predict(double T)
{   
    //step one-compute the info of prectic box
    //x y z theta l w h angle 
    // double box_predict[CLUSTER_NUM][8];
    for(int i=0;i<n;i++)
    {   
        //pos at this frame
        // double x0=state[i][0],y0=state[i][1],z0=feature_k1[i][7];
        // double v=state[i][2],theta=state[i][3];    //theta is the direction of move 
        // //info of box
        // double l=feature_k1[i][2],w=feature_k1[i][3],h=feature_k1[i][4];
        // double angle=feature_k1[i][6];     //angle is the orientation of box
        
        //method2 use box_V2
        double x0=box_V2[i][0],y0=box_V2[i][1],z0=box_V2[i][2];
        double v=state[i][2],theta=state[i][3];    //theta is the direction of move 
        double l=box_V2[i][3],w=box_V2[i][4],h=box_V2[i][5];
        double angle=box_V2[i][6];

        //pos at the next frame
        double x1,y1,z1;
        //unit vector [cos(theta),sin(theta)]
        x1=x0+v*T*cos(theta);y1=y0+v*T*sin(theta);z1=z0;
        box_predict[i][0]=x1;box_predict[i][1]=y1;box_predict[i][2]=z1;
        box_predict[i][3]=theta;
        box_predict[i][4]=l;box_predict[i][5]=w;box_predict[i][6]=h;
        box_predict[i][7]=angle;
    }
    //--------------------step2------------------------------
    visualization_msgs::MarkerArray marker_kbox_array;
    marker_kbox_array.markers.clear();
    visualization_msgs::Marker kbox_marker;
    kbox_marker.header.frame_id = str_frame;
    kbox_marker.header.stamp = ros::Time::now();
    kbox_marker.ns = "kbox_array";
    kbox_marker.color.r = 0.0f;
    kbox_marker.color.g = 1.0f;
    kbox_marker.color.b = 0.0f;
    kbox_marker.color.a = 0.5;
    kbox_marker.lifetime = ros::Duration();
    kbox_marker.frame_locked = true;
    kbox_marker.type = visualization_msgs::Marker::CUBE;
    kbox_marker.action = visualization_msgs::Marker::ADD;
    //makrer_id用于计数
    int marker_kbox_id = 0;
    //n表示所有的类别数
    for (size_t i = 0; i < n; ++i)
    {
        kbox_marker.id = marker_kbox_id;
        kbox_marker.color.a = 0.5;
        kbox_marker.pose.position.x = box_predict[i][0];
        kbox_marker.pose.position.y = box_predict[i][1];
        kbox_marker.pose.position.z = box_predict[i][2];       //the data struct of feature_k1 
        double theta=box_predict[i][3];
        double length = box_predict[i][4];
        double width = box_predict[i][5];
        double height = box_predict[i][6];
        double angle = box_predict[i][7];
        kbox_marker.scale.x = length;
        kbox_marker.scale.y = width;
        kbox_marker.scale.z = height;
        //角度问题
        Quaternion q=ToQuaternion(angle,0,0);   //it is a question to use angle or theta
        double w=q.w,x=q.x,y=q.y,z=q.z;
        kbox_marker.pose.orientation.w = w;
        kbox_marker.pose.orientation.x = x;
        kbox_marker.pose.orientation.y = y;
        kbox_marker.pose.orientation.z = z;
        //压入
        marker_kbox_array.markers.push_back(kbox_marker);
        ++marker_kbox_id;
    }
    //kbox_max表示至今为止最多的kbox盒子数量
    if (marker_kbox_array.markers.size() > kbox_max)
    {
        kbox_max = marker_kbox_array.markers.size();
    }
    //对于多余的不需要显示的，进行透明化
    for (size_t i = marker_kbox_id; i < kbox_max; ++i)
    {
        kbox_marker.id = i;
        kbox_marker.color.a = 0;
        kbox_marker.pose.position.x = 0;
        kbox_marker.pose.position.y = 0;
        kbox_marker.pose.position.z = 0;
        kbox_marker.scale.x = 0.1;
        kbox_marker.scale.y = 0.1;
        kbox_marker.scale.z = 0.1;
        marker_kbox_array.markers.push_back(kbox_marker);
        ++marker_kbox_id;
    }
    kalmanBoxPub.publish(marker_kbox_array);
    //kalman box输出结束
}

//input state vector , compute the area of union/intersection
//input : x y l w  theta-the direction of boudingbox,  alpha-the direction of move
void EuClusterCore::Ratio_IU_compute()
{   
    double RatioIU[CLUSTER_NUM];
    for(int i=0;i<n;i++)
    {
        double l=feature_k1[i][2],w=feature_k1[i][3],theta=feature_k1[i][7];
        double v=state[i][2],alpha=state[i][3];
        double t=0.1;
        double LI=l-abs(v*t*cos(alpha-theta)),WI=w-abs(v*t*sin(alpha-theta));
        double SI=LI*WI;
        double S=l*w;
        double R=SI/(2*S-SI);
        RatioIU[i]=R;
    }
}

//use Ratio of IU to design match strategy
//input: box_predict     x y z theta l w h angle   angle is the direction of box, the num is m
//input:box_V2   x y z l w h angle , the num is n
void EuClusterCore::Match_by_RIU()
{   
    double RIU_matrix[CLUSTER_NUM][CLUSTER_NUM];
    //a matrix of n*m
    for(int i=0;i<n;i++)
    {   
        double x1=box_V2[i][0],y1=box_V2[i][1],l1=box_V2[i][3],w1=box_V2[i][4],angle1=box_V2[i][6];
        cout << "x1=" << x1 << "    y1=" << y1 << "  l1=" << l1 << "    w1=" << w1 << "  angle1=" << angle1 << endl;
        for(int j=0;j<m;j++)
        {    
            double res;
            cout << "i=" << i << "  j=" << j << endl;
            double x2=box_predict[j][0],y2=box_predict[j][1],l2=box_predict[j][4],w2=box_predict[j][5],
                            angle2=box_predict[j][7];
            cout << "x2=" << x2 << "    y2=" << y2 << "  l2=" << l2 << "    w2=" << w2 << "  angle2=" << angle2 << endl;
            x1=0;y1=0;angle1=0;
            x2=x2-x1;y2=y2-y1;angle2=angle2-angle1;
            double S1=l1*w1,S2=l2*w2;
            // double Smax=(S1>S2?S1:S2),Smin=(S1<S2?S1:S2);
            // double dA=(S2-S1)/S2;
            // if(dA>0.5)
            // {
            //     res=0;
            //     RIU_matrix[i][j]=res;
            //     continue;
            // }
            //judge if there are any points inside the box
            //for simplicity we think they are parallel
            double xl1=-0.5*l1,yb1=-0.5*w1,xr1=0.5*l1,yt1=0.5*w1;
            double xl2=x2-0.5*l2,yb2=y2-0.5*w2,xr2=x2+0.5*l2,yt2=y2+0.5*w2;
            double xmin,ymin,xmax,ymax;
            xmin=max(xl1,xl2);
            ymin=max(yb1,yb2);
            xmax=min(xr1,xr2);
            ymax=min(yt1,yt2);
            double length=xmax-xmin,width=ymax-ymin;
            double cross_square;
            if((length<=0)||(width<=0))
            {   
                cout << "point A" << endl;
                res=0;
                RIU_matrix[i][j]=res;
                continue;
            }
            else
            {
                cross_square=length*width;
            }
            cout << "S1=" << S1 << "    S2=" << S2 << "    cross_square=" << cross_square << endl;
            res=cross_square/(S1+S2-cross_square);
            RIU_matrix[i][j]=res;
        }
    }
    cout << "print RIU_matrix" << endl;
    for(int i=0;i<n;i++)
    {
        for(int j=0;j<m;j++)
        {
            cout << RIU_matrix[i][j] << "  ";
        }
        cout << endl;
    }
    //use correspond to compute corr_relation
    std::vector<vector<double> > M_S;
    for(int i=0;i<n;i++)
    {   
        std::vector<double> S_raw;   //某一行的相似度值
        for(int j=0;j<m;j++)
        {
            S_raw.push_back(RIU_matrix[i][j]);
        }
        M_S.push_back(S_raw);
    }
    std::vector<int> k1_corr;
    std::vector<int> k_corr;
    for(int i=0;i<n;i++)
    {
        k1_corr.push_back(-1);
    }
    for(int j=0;j<m;j++)
    {
        k_corr.push_back(-1);
    }
    correspond(M_S,&k1_corr,&k_corr);
    //计算匹配率
    cout << "print_k1_corr" << endl;
    int success=0;
    for(int i=0;i<n;i++)
    {   
        if(k1_corr[i]!=-1)
            success++;
        // cout <<"i-" <<  k1_corr[i] << endl;
    }
    cout << "匹配率:" << int(success/n*100) << "%" << endl;
}

//用最大的矩形来调整矩形框
//input: x y l w alpha-the driection of box, lmax and wmax (l_w_max)
//output: xnew ynew lmax wmax
void EuClusterCore::box_adjustment()
{   
    // cout << "in the function of box_adjustment" << endl;
    //x y l w
    // double rectangle_update[CLUSTER_NUM][4];
    for(int i=0;i<n;i++)
    {   
        // cout << "i=" << i << endl;
        double x=feature_k1[i][0],y=feature_k1[i][1],l=feature_k1[i][2],w=feature_k1[i][3],alpha=feature_k1[i][6];
        // cout << "alpha=" << alpha*180.0/pi << endl;
        double k=tan(alpha);
        double lmax,wmax;
        lmax=l_w_max[i][0];wmax=l_w_max[i][1];
        // cout << "lmax=" << lmax << "  wmax=" << wmax << endl;
        //p is the center of box
        double p[2],l_vector[2],w_vector[2];
        p[0]=x;
        p[1]=y;
        l_vector[0]=0.5*l/sqrt(k*k+1);
        l_vector[1]=0.5*l*k/sqrt(k*k+1);
        w_vector[0]=-0.5*w*k/sqrt(k*k+1);
        w_vector[1]=0.5*w/sqrt(k*k+1);
        //compute the four vectexs of the rectangle A B C D
        //计算矩形的四个顶点
        double P[4][2];
        P[0][0]=p[0]+l_vector[0]+w_vector[0];
        P[0][1]=p[1]+l_vector[1]+w_vector[1];
        P[1][0]=p[0]-l_vector[0]+w_vector[0];
        P[1][1]=p[1]-l_vector[1]+w_vector[1];
        P[2][0]=p[0]-l_vector[0]-w_vector[0];
        P[2][1]=p[1]-l_vector[1]-w_vector[1];
        P[3][0]=p[0]+l_vector[0]-w_vector[0];
        P[3][1]=p[1]+l_vector[1]-w_vector[1];
        // cout << "P" << endl;
        // for(int num=0;num<4;num++)
        // {
        //     cout << P[num][0] << "    " << P[num][1] << endl;
        // }
        //计算谁距离原点最近,计算结果为P_close[2]
        double DIS_O[4];
        DIS_O[0]=sqrt(P[0][0]*P[0][0]+P[0][1]*P[0][1]);
        DIS_O[1]=sqrt(P[1][0]*P[1][0]+P[1][1]*P[1][1]);
        DIS_O[2]=sqrt(P[2][0]*P[2][0]+P[2][1]*P[2][1]);
        DIS_O[3]=sqrt(P[3][0]*P[3][0]+P[3][1]*P[3][1]);
        //1-A,2-B,3-C,4-D
        int flag=0;
        for(int j=0;j<n;j++)
        {
            if(DIS_O[j]<DIS_O[flag])
            {
                flag=j;
            }
        }
        double P_close[2];
        P_close[0]=P[flag][0];
        P_close[1]=P[flag][1];
        //compute the center postion after adjst
        int sl,sw;      //symbol for compute, it is +1 or -1
        double P_O[2];
        // cout << "flag=" << flag << endl;
        // cout << "Pclose: " << P_close[0] << "    " << P_close[1] << endl;
        switch(flag)
        {
            case 0:
            {   
                sl=1;
                sw=1;
                // cout << "flag=0" << endl;
                break;
            }
            case 1:
            {
                sl=-1;
                sw=1;
                // cout << "flag=1" << endl;
                break;
            }
            case 2:
            {
                sl=-1;
                sw=-1;
                // cout << "flag=2" << endl;
                break;
            }
            case 3:
            {
                sl=1;
                sw=-1;
                // cout << "flag=3" << endl;
                break;
            }
        }
        // cout << "sl=" << sl << "  sw=" << sw << endl;
        P_O[0]=P_close[0]-sl*l_vector[0]*lmax/l-sw*w_vector[0]*wmax/w;
        P_O[1]=P_close[1]-sl*l_vector[1]*lmax/l-sw*w_vector[1]*wmax/w;
        // cout << "Po  " <<  P_O[0] << "  " << P_O[1] << endl;
        rectangle_update[i][0]=P_O[0];
        rectangle_update[i][1]=P_O[1];
        rectangle_update[i][2]=lmax;
        rectangle_update[i][3]=wmax;
    }
}

//function for test
void EuClusterCore::print_feature_k1()
{   
    cout << "print feature_k1 " << endl;
    cout << "n=" << n << endl;
    for(int i=0;i<n;i++)
    {      
        cout << i << "    ";
        //特征点从9到14,8为这个类别具有的点的数量
        for(int j=8;j<15;j++)
        {
            cout << feature_k1[i][j] << "    ";
        }
        cout << endl;
    }
}

void EuClusterCore::print_feature_k()
{   
    cout << "print feature_k " << endl;
    for(int i=0;i<n;i++)
    {
        for(int j=0;j<8;j++)
        {
            cout << feature_k[i][j] << "    ";
        }
        cout << endl;
    }
}

//input: corr_chain   feature_last_five before updateing,  feature_k1
void EuClusterCore::update_l_w_max(vector<int> k1_corr)
{   
    cout << "in the begin of function update_l_w_max " << endl;
    // for(int i=0;i<n;i++)
    // {   
    //     double lmax=feature_k1[i][2],wmax=feature_k1[i][3];
    //     //num of comparision is 5
    //     for(int j=0;j<5;j++)
    //     {
    //         int index=corr_chain[i][j+2];
    //         //the last j frame , the NO.index culster 's length and width
    //         double l=feature_last_five[j][index][2],w=feature_last_five[j][index][3];
    //         if(l>lmax)
    //         {
    //             lmax=l;
    //         }
    //         if(w>wmax)
    //         {
    //             wmax=w;
    //         }
    //     }
    //     l_w_max[i][0]=lmax;
    //     l_w_max[i][1]=wmax;
    // }

    //the first step, use feature_k1 to initilize box_V2
    //box_v2  x y z l w h angle
    for(int i=0;i<n;i++)
    {   
        double x=feature_k1[i][0],y=feature_k1[i][1];
        double l=feature_k1[i][2],w=feature_k1[i][3];
        box_V2[i][0]=x;box_V2[i][1]=y;
        box_V2[i][3]=l;box_V2[i][4]=w;
    }
    // cout << "before compute box_V2" << endl;
    // cout << "print box_V2" << endl;
    // for(int i=0;i<n;i++)
    // {
    //     for(int j=0;j<7;j++)
    //     {
    //         cout << box_V2[i][j] << "  ";
    //     }
    //     cout << endl;
    // }
    // cout << "print box_V2_last" << endl;
    // for(int i=0;i<n;i++)
    // {
    //     for(int j=0;j<7;j++)
    //     {
    //         cout << box_V2_last[i][j] << "  ";
    //     }
    //     cout << endl;
    // }
    // cout << "print k1_corr" << endl;
    // for(int i=0;i<n;i++)
    // {
    //     cout << k1_corr[i] << endl;
    // }
    //another method
    for(int i=0;i<n;i++)
    {
        double lmax=box_V2[i][3],wmax=box_V2[i][4];
        int index=k1_corr[i];
        // cout << "i=" << i << "  index="  << index << endl;
        if(index==-1)
        {
            continue;
        }
        // cout << "test 1" << endl;
        double llast=box_V2_last[index][3],wlast=box_V2_last[index][4];
        // cout << "llast=" << llast << "  wlast=" << wlast << endl;
        if(lmax<llast)
        {
            lmax=llast;
        }
        if(wmax<wlast)
        {
            wmax=wlast;
        }
        l_w_max[i][0]=lmax;
        l_w_max[i][1]=wmax;
        box_V2[i][3]=lmax;
        box_V2[i][4]=wmax;
    }
    // cout << "after compute print box v2" << endl;
    // for(int i=0;i<n;i++)
    // {
    //     for(int j=0;j<7;j++)
    //     {
    //         cout << box_V2[i][j] << "  ";
    //     }
    //     cout << endl;
    // }    
    // cout << "print l_w_max" << endl;
    // for(int i=0;i<n;i++)
    // {
    //     cout << l_w_max[i][0] << "      " << l_w_max[i][1] << endl;
    // }
    cout << "in the end of function update_l_w_max " << endl;
}

//input feature_k1, l_w_max
void EuClusterCore::publish_cboxV2_array()
{   
    // cout << "in the function of publish_cboxV2_array " << endl;
    //first step, compute rectangle parameters
    box_adjustment();
    //the output is rectangle[][]  xnew ynew lmax wmax
    //feature_k1 x y l w h s angle z
    
    //an extra step, box_parameters x y z l w h angle
    // double box_V2[CLUSTER_NUM][7];
    for(int i=0;i<n;i++)
    {
        double x=feature_k1[i][0],y=feature_k1[i][1],z=feature_k1[i][7],
                        l=feature_k1[i][2],w=feature_k1[i][3],h=feature_k1[i][4],
                        angle=feature_k1[i][6],
                        xnew=rectangle_update[i][0],ynew=rectangle_update[i][1],
                        lmax=rectangle_update[i][2],wmax=rectangle_update[i][3];
        box_V2[i][0]=xnew;box_V2[i][1]=ynew;box_V2[i][2]=z;
        box_V2[i][3]=lmax;box_V2[i][4]=wmax;box_V2[i][5]=h;
        box_V2[i][6]=angle;
    }
    // cout << "before pub the box_v2 is " << endl;
    // // cout << "print box_V2" << endl;
    // // for(int i=0;i<n;i++)
    // // {
    // //     for(int j=0;j<7;j++)
    // //     {
    // //         cout << box_V2[i][j] << "    ";
    // //     }
    // //     cout << endl;
    // // }
    //step2 pub
    visualization_msgs::MarkerArray marker_cbox_array;
    marker_cbox_array.markers.clear();
    visualization_msgs::Marker cbox_marker;
    cbox_marker.header.frame_id = str_frame;
    cbox_marker.header.stamp = ros::Time::now();
    cbox_marker.ns = "cbox_array";
    cbox_marker.color.r = 1.0f;
    cbox_marker.color.g = 0.0f;
    cbox_marker.color.b = 0.0f;
    cbox_marker.color.a = 0.5;
    cbox_marker.lifetime = ros::Duration();
    cbox_marker.frame_locked = true;
    cbox_marker.type = visualization_msgs::Marker::CUBE;
    cbox_marker.action = visualization_msgs::Marker::ADD;
    //makrer_id用于计数
    int marker_cbox_id = 0;
    //n表示所有的类别数
    for (size_t i = 0; i < n; ++i)
    {
        cbox_marker.id = marker_cbox_id;
        cbox_marker.color.a = 0.5;
        cbox_marker.pose.position.x = box_V2[i][0];
        cbox_marker.pose.position.y = box_V2[i][1];
        cbox_marker.pose.position.z = box_V2[i][2];
        double length = box_V2[i][3];
        double width = box_V2[i][4];
        double height = box_V2[i][5];
        cbox_marker.scale.x = length;
        cbox_marker.scale.y = width;
        cbox_marker.scale.z = height;
        //角度问题
        // double angle=output_data[i][4];
        double angle=box_V2[i][6];
        Quaternion q=ToQuaternion(angle,0,0);;
        double w=q.w,x=q.x,y=q.y,z=q.z;
        cbox_marker.pose.orientation.w = w;
        cbox_marker.pose.orientation.x = x;
        cbox_marker.pose.orientation.y = y;
        cbox_marker.pose.orientation.z = z;
        //压入
        marker_cbox_array.markers.push_back(cbox_marker);
        ++marker_cbox_id;
    }
    //cbox_max表示至今为止最多的盒子数量 
    //the num of cbox_V2  is the same as cbox
    if (marker_cbox_array.markers.size() > cboxV2_max)
    {
        cboxV2_max = marker_cbox_array.markers.size();
    }
    //对于多余的不需要显示的，进行透明化
    for (size_t i = marker_cbox_id; i < cboxV2_max; ++i)
    {
        cbox_marker.id = i;
        cbox_marker.color.a = 0;
        cbox_marker.pose.position.x = 0;
        cbox_marker.pose.position.y = 0;
        cbox_marker.pose.position.z = 0;
        cbox_marker.scale.x = 0.1;
        cbox_marker.scale.y = 0.1;
        cbox_marker.scale.z = 0.1;
        marker_cbox_array.markers.push_back(cbox_marker);
        ++marker_cbox_id;
    }
    markerCboxV2Pub.publish(marker_cbox_array);
    cout << "in the end of publish_cboxV2_array " << endl;
}

//ues box_V2 to update box_V2_last
void EuClusterCore::update_boxV2()
{
    // cout << "in the funciton of update_boxV2" << endl;
    for(int i=0;i<n;i++)
    {
        for(int j=0;j<7;j++)
        {
            box_V2_last[i][j]=box_V2[i][j];
        }
    }
    // cout << "after update box_V2_last is" << endl;
    // for(int i=0;i<n;i++)
    // {
    //     for(int j=0;j<7;j++)
    //     {
    //         cout << box_V2_last[i][j] << "  ";
    //     }
    //     cout << endl;
    // }
}

//相机转到雷达相关函数

//点云转为vector
vector<vector<double> > EuClusterCore::pointcloud_to_vector(pcl::PointCloud<pcl::PointXYZI>::Ptr in_pc)
{
    vector<vector<double>   > points;
    for(int i=0;i<in_pc->points.size();i++)
    {
        vector<double> point;
        point.push_back(in_pc->points[i].x);
        point.push_back(in_pc->points[i].y);
        point.push_back(in_pc->points[i].z);
        points.push_back(point);
    }
    return points;
}

//读取矩形框的txt数据并转化为数组
int EuClusterCore::read_from_txt(vector<vector<double> >* rect_data)
{   
    //矩形框中心xy以及长宽
    string Filepath="/home/sugar/catkin_ws/src/pc_process/data/rect/test.txt";
    std::fstream in;
    in.open(Filepath, std::ios::in);
    if (!in.is_open()) 
    {
		cout << "Can not find " << Filepath << endl;
		system("pause");
	}
    string buff;
    int i=0;
    while(getline(in,buff))
    {
        vector<double> nums;
        char *s_input = (char *)buff.c_str();
        const char * split = ",";
        char *p = strtok(s_input,split);
        double a;
        while(p!=NULL)
        {
            a = atof(p);
            cout << "a=" << a << endl;
            nums.push_back(a);
            p=strtok(NULL,split);
        }
        for(int b=0;b<nums.size();b++)
        {
            (*rect_data)[i][b]=nums[b];
        } 
        i++;
    }
    in.close();
    cout << " get data " << endl;
    int num_rect = i;
    cout << "行数=" << num_rect << endl;
    return num_rect;
}

//对雷达所有点进行判断，看其在哪个矩形框中
//参数：雷达点云，雷达到相机T1，相机到像素T2，矩形框参数，矩形框数量
void EuClusterCore::cluster_by_cam(vector<vector<double> > points,
                                                                            Eigen::Matrix<double, 4, 4> T1,Eigen::Matrix<double, 3, 3> T2,
                                                                            double rect[][4],int num_rect)
{   
    //存储结果索引,多少个矩形框就有多少个类别
    vector<vector<int> > res(num_rect);
    //点云在相机上的范围 xmin xmax ymin ymax
    vector< vector<int> > area(num_rect, vector<int>(4, 0));
    int num_p=points.size();
    for(int i=0;i<num_p;i++)
    {
        double x=points[i][0],y=points[i][1],z=points[i][2];
        Eigen::Matrix<double, 4, 1> p_lidar;
        p_lidar << x, y, z, 1;
        Eigen::Matrix<double, 4, 1> p_cam;
        p_cam=T1*p_lidar;
        Eigen::Matrix<double, 3, 1> p_cam_xy;
        p_cam_xy<<p_cam[0],p_cam[1],1;
        Eigen::Matrix<double, 3, 1> p_pixel;
        p_pixel=T2*p_cam_xy;
        double p[2]={p_pixel[0],p_pixel[1]};
        for(int j=0;j<num_rect;j++)
        {   
            double rect_t[4]={rect[j][0],rect[j][1],rect[j][2],rect[j][3]};
            if(point_in_rect(p,rect_t))
            {   
                res[j].push_back(i);
                cout << "第" << i << "个点属于第" << j << "个类" << endl;
                break;
            }
        }
    }
    //根据结果索引进行计算 x y l w angle
    double res_boxes[CLUSTER_NUM][5];
    for(int i=0;i<num_rect;i++)
    {   
        vector<vector<double> > points_cluster;
        //索引到坐标
        for(int j=0;j<res[i].size();j++)
        {
            vector<double> point;
            point.push_back(points[res[i][j]][0]);
            point.push_back(points[res[i][j]][1]);
            points_cluster.push_back(point);
        }
        double *box=LSMbox_compute(points_cluster);
        for(int j=0;j<5;j++)
        {
            res_boxes[i][j]=box[j];
        }
    }
    //结果出来以后，根据结果绘制包围盒
    publish_box_array(res_boxes,n);
}

//一个通用的绘制包围盒的函数
void EuClusterCore::publish_box_array(double boxes[][5],int n)
{   
    cout << "in the function of publish_cbox_array" << endl;
    //创建一个包围盒子用于绘制类别包围盒，不同于之前的通行区域包围盒子
    visualization_msgs::MarkerArray marker_cbox_array;
    marker_cbox_array.markers.clear();
    visualization_msgs::Marker cbox_marker;
    // cbox_marker.header.frame_id = "rslidar";
    cbox_marker.header.frame_id = str_frame;
    cbox_marker.header.stamp = ros::Time::now();
    cbox_marker.ns = "cbox_array";
    cbox_marker.color.r = 0.0f;
    cbox_marker.color.g = 1.0f;
    cbox_marker.color.b = 0.0f;
    cbox_marker.color.a = 1.0;
    cbox_marker.lifetime = ros::Duration();
    cbox_marker.frame_locked = true;
    cbox_marker.type = visualization_msgs::Marker::CUBE;
    cbox_marker.action = visualization_msgs::Marker::ADD;
    //makrer_id用于计数
    int marker_cbox_id = 0;
    //n表示所有的类别数
    for (size_t i = 0; i < n; ++i)
    {   
        cbox_marker.id = marker_cbox_id;
        cbox_marker.color.a = 0.5;
        //output - PAbox  feature_k1 x y  l w h s angle z
        cbox_marker.pose.position.x = boxes[i][0];
        cbox_marker.pose.position.y = boxes[i][1];
        cbox_marker.pose.position.z = 0;       //the data struct of feature_k1 
        double length = boxes[i][2];
        double width = boxes[i][3];
        double height = 1;
        //角度问题
        double angle=boxes[i][4];
        cbox_marker.scale.x = length;
        cbox_marker.scale.y = width;
        cbox_marker.scale.z = height;
        Quaternion q=ToQuaternion(angle,0,0);;
        double w=q.w,x=q.x,y=q.y,z=q.z;
        cbox_marker.pose.orientation.w = w;
        cbox_marker.pose.orientation.x = x;
        cbox_marker.pose.orientation.y = y;
        cbox_marker.pose.orientation.z = z;
        marker_cbox_array.markers.push_back(cbox_marker);
        ++marker_cbox_id;
    }
    //rect_max表示至今为止最多的盒子数量
    if (marker_cbox_array.markers.size() > rect_max)
    {
        rect_max = marker_cbox_array.markers.size();
    }
    //对于多余的不需要显示的，进行透明化
    for (size_t i = marker_cbox_id; i < rect_max; ++i)
    {
        cbox_marker.id = i;
        cbox_marker.color.a = 0;
        cbox_marker.pose.position.x = 0;
        cbox_marker.pose.position.y = 0;
        cbox_marker.pose.position.z = 0;
        cbox_marker.scale.x = 0.1;
        cbox_marker.scale.y = 0.1;
        cbox_marker.scale.z = 0.1;
        marker_cbox_array.markers.push_back(cbox_marker);
        ++marker_cbox_id;
    }
    markerBoxPub.publish(marker_cbox_array);
}


//主函数
void EuClusterCore::point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr)
{   
    //
    // ros::Rate r(10);

    cout << "第" << frame_id << "帧开始" << endl;
    // cout << "1  0   0   0" << endl;     //用于输出坐标用
    //计算时间
    time_t begin,end;
    double ret;
    begin=clock();
    //计算时间

    //current_pc_ptr和filtered_pc_ptr的类型全部改变
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);

    //in_cloud_ptr为ROSmsg格式的
    point_cloud_header_ = in_cloud_ptr->header;
    str_frame = point_cloud_header_.frame_id;
    

    //从ROSMsg导入，把第一个参数转化为第二个参数
    //current_pc_ptr为计算时使用的点
    pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);
    // cout << "位置1  the num of current_pc_ptr is " << current_pc_ptr->points.size() << endl;

    // for(size_t i=0;i<current_pc_ptr->points.size();i++)
    // {
    //     cout << current_pc_ptr->points[i].x << "   " << current_pc_ptr->points[i].x << "   " 
    //               << current_pc_ptr->points[i].z << endl; 
    // }

    // down sampling the point cloud before cluster 在聚类之前再进行一次降采样
    voxel_grid_filter(current_pc_ptr, filtered_pc_ptr, LEAF_SIZE);
    // cout << "位置3  the num of filtered_pc_ptr is " << filtered_pc_ptr->points.size() << endl;

    // cluster_segment(filtered_pc_ptr);
    //欧式聚类相关内容
    cluster_segment(current_pc_ptr);

    // cout << "test2" << endl;

    //普通模式才需要匹配和跟踪相关的内容
    if(work_mode_==0)
    {
        // cout << "before normilization" << endl;
        // print_feature_k1();
        //normilize the parameters of boundingbox,make l is bigger than w
        for(int i=0;i<n;i++)
        {
            double l=feature_k1[i][2];
            double w=feature_k1[i][3];
            double angle=feature_k1[i][6];
            if(l<w)
            {   
                // cout << "need to normilize " << endl;
                // cout << "angle=" << angle << endl;
                double temp=l;
                l=w;
                w=temp;
                angle=angle+90.0/180.0*pi;
                // cout << "angle=" << angle << endl;
                feature_k1[i][2]=l;
                feature_k1[i][3]=w;
                feature_k1[i][6]=angle;
            }
        }
        // cout << "after normilization" << endl;
        
        // print_feature_k1();

        vector<vector<double> > G;
        std::vector<int> k1_corr;
        std::vector<int> k_corr;
        // if(work_mode_==0)
        // {
            //初始化G  n行4列 x y v theta
            // cout << "进行相似度计算和匹配之前" << endl;
            // cout << "m=" << m << "    n=" << n << endl; 
            for(int i=0;i<n;i++)
            {
                vector<double> G_raw(4,-1);
                G.push_back(G_raw);
            }
            //进行匹配  初始化为-1表示没有匹配上
            for(int i=0;i<n;i++)
            {
                k1_corr.push_back(-1);
            }
            for(int j=0;j<m;j++)
            {
                k_corr.push_back(-1);
            }
            //如果为第一帧，直接赋值
            // cout << "frame_id的值" << frame_id << endl;
            if(frame_id==0)
            {
                //x y直接用特征矩阵，v赋值成1,theta=angle
                for(int i=0;i<n;i++)
                {
                    G[i][0]=feature_k1[i][0];    //x
                    G[i][1]=feature_k1[i][1];    //y
                    G[i][2]=0;                   //v
                    G[i][3]=feature_k1[i][6];    //theta
                }
            }
            else
            {
                //相似度矩阵初始化
                std::vector<vector<double> > M_S;
                for(int i=0;i<n;i++)
                {   
                    std::vector<double> S_raw;   //某一行的相似度值
                    for(int j=0;j<m;j++)
                    {
                        S_raw.push_back(0);
                    }
                    M_S.push_back(S_raw);
                }
                // cout << "test4" << endl;

                similarity_martix(&M_S);
                // cout << "test5" << endl;
                // cout << "相似度矩阵的维度" << M_S.size() << "  " << M_S[0].size() << endl;

                //this function will change the value of k1_corr
                correspond(M_S,&k1_corr,&k_corr);
                // cout << "test6" << endl;
                //没有处理的添加到动态列表Q中
                vector<vector<double> > Q;
                int num_Q=Q_generate(&Q,k1_corr,k_corr);

                //动态目标运动状态估计
                //针对当前第k+1帧进行处理，总共有n个类别暂时用匀速直线运动来处理
                //数据格式  x  y  speed  theta        x坐标-y坐标-速度-运动方向
                double T=0.1;          //雷达的采集周期
                G_generate(&G,k1_corr,k_corr,T);
                // cout << "k1corr的维度=" << k1_corr.size() << "kcorr的维度=" << k_corr.size() << endl;
            }
            // cout << "test7" << endl;
        // }
        
        // cout << "print_k1_corr" << endl;
        int success=0;
        for(int i=0;i<n;i++)
        {   
            if(k1_corr[i]!=-1)
                success++;
            // cout << i << "==" << k1_corr[i] << endl;
        }
        // cout << "匹配率:" << int(success*100.0/n) << "%" << endl;
    }

    //马老师模式发布相关数据
    if(work_mode_==2)
    {
        //制作用于规划的输出数据
        output_data_make();
        publish_rs_msgs();
    }
    
    //相机到雷达的方式
    if(work_mode_==3)
    {

    }

    //计算当前各个包围盒子之间的中心距离
    // cout << "计算当前帧各个盒子中心之间的距离" << endl;
    // for(int i=0;i<n;i++)
    // {
    //     for(int j=0;j<n;j++)
    //     {   
    //         double x1=output_data[i][0],y1=output_data[i][1],
    //                x2=output_data[j][0],y2=output_data[j][1];
    //         double dis=sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
    //         // cout << "i=" << i << " j=" << j << " dis=" << dis << endl;
    //     }
    // }
    // cout << "test9" << endl;

    // cout << " k1_corr " << endl;
    // for(int i=0;i<n;i++)
    // {
    //     cout << i << "--" << k1_corr[i] << endl;
    // }
    // print_feature_k1();

    //发布相关内容
    publish_cbox_array();
    // publish_index_array(k1_corr);
    // publish_speed_arrow(G);
    publish_PAbox_array();

    // cout << "test10" << endl;

    if(work_mode_==1)
    {
        //save data to txt 
        data_to_txt();
    }
    
    // print_information(k1_corr);

    //五帧平均滤波相关
    if(work_mode_==0)
    {
        //corr_all
        // cout << "in point_cb1" << endl;
        // update_corr_all(k1_corr);
        // print_corr_all();
        // cout << "in point_cb2" << endl;
        update_corr_chain();
        // print_corr_chain();
        // cout << "in point_cb3" << endl;
        corr_chain_speed(5);     //计算5帧平均滤波后的速度
        publish_speed_arrow_v2();    //use 5 average filter
    }
    
    // cout << "test11" << endl;

    //kalman predict 
    // cout << "in point_cb6" << endl;

    //卡尔曼预测
    // kalman_predict(0.1);

    //update the maximal length and width
    // print_feature_k1();
    // print_feature_k();

    //每次用大的长宽去更新矩形参数
    // update_l_w_max(k1_corr);
    // publish_cboxV2_array();

    //交并比匹配
    // cout << "Match_by_RIU" << endl;
    // Match_by_RIU();

    // cout << "test11" << endl;

    //feature_point
    //这一帧结束，当前帧的FP_data赋值给上一帧
    // for(int i=0;i<n;i++)
    // {
    //     for(int j=0;j<4;j++)
    //     {   
    //         for(int k=0;k<4;k++)
    //         {
    //             FP_data_k[i][j][k]=FP_data_k1[i][j][k];
    //         }  
    //     }
    // }

    // //publish point cloud
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_box_edge_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    // //save the point to a array
    // // cout << "test vector " << endl;
    // for(int i=0;i<test_vector.size();i++)
    // {
    //     double x=test_vector[i][0];
    //     double y=test_vector[i][1];
    //     // cout << "i=" << i << "  x=" << x << "   y=" << y << endl;
    //     double z=0;
    //     double intensity=50;
    //     pcl::PointXYZI point;
    //     point.x=x;
    //     point.y=y;
    //     point.z=z;
    //     point.intensity=intensity;
    //     cloud_box_edge_ptr->points.push_back(point);  //add a point to a pointcloud ptr
    // }
    // publish_cloud(pub_box_edge_pts, cloud_box_edge_ptr, in_cloud_ptr->header);
    // cout << "end for test vector " << endl;

    // cout << "pub all points xy " << endl;
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_all_ptxy_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    // for(int i=0;i<all_points_xy.size();i++)
    // {
    //     double x=all_points_xy[i][0];
    //     double y=all_points_xy[i][1];
    //     // cout << "i=" << i << "  x=" << x << "   y=" << y << endl;
    //     double z=0;
    //     double intensity=50;
    //     pcl::PointXYZI point;
    //     point.x=x;
    //     point.y=y;
    //     point.z=z;
    //     point.intensity=intensity;
    //     cloud_all_ptxy_ptr->points.push_back(point);  //add a point to a pointcloud ptr
    // }
    // publish_cloud(pub_all_pts_xy, cloud_all_ptxy_ptr, in_cloud_ptr->header);

    // cout << "test12" << endl;

    if(work_mode_==0)
    {
        //the update of feature_k should be processed at the last
        //这一帧结束，当前帧的类别数赋值给m，当前帧的feature赋值给上一帧
        // cout << "test12" << endl;
        update_feature_last_five();
        // cout << "test12-1" << endl;
        update_boxV2();
        // cout << "test13" << endl;
        m=n;
        for(int i=0;i<n;i++)
        {   
            //现在特征数量为15
            for(int j=0;j<15;j++)
            {
                feature_k[i][j]=feature_k1[i][j];
            }
        }
    }
    
    // read_txt();
    
    //计算时间
    end=clock();
    ret=double(end-begin)/CLOCKS_PER_SEC;
    ret=ret*1000;   //转化为毫秒
     cout << "run time:" << ret << "ms" << endl;
    // cout << "the frame " << frame_id << "over" << endl;

    if(ret>tmax)
    {
        tmax=ret;
    }
    cout << "tmax=" << tmax  << "ms " << endl << endl;
    //帧id增加
    // if(frame_id==152)
    // {
    //     int i;
    //     cin >> i;
    // }
    frame_id++;

    // cout << "是否进行下一帧" << endl;
    // int flag;
    // cin >> flag;

}