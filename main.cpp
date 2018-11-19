#include <iostream>
#include <vector>
#include <fstream>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>


using namespace cv;
using namespace std;


int main()
{
    ifstream inImgPath("../calibdata.txt"); /* 标定所用图像文件的路径 */
    ofstream fout("caliberation_result.txt");  /* 保存标定结果的文件 */
    //读取每一幅图像，从中提取出角点，然后对角点进行亚像素精确化
    cout << "Start to extract corners......"  << endl;
    int image_count = 0;  /* 图像数量 */
    vector<string> imgList;
    Size image_size;  /* 图像的尺寸 */
    Size board_size = Size(7,11);    /* 标定板上每行、列的角点数 */
    vector<Point2f> image_points_buf;  /* 缓存每幅图像上检测到的角点 */
    vector<vector<Point2f>> image_points_seq; /* 保存检测到的所有角点 */
    string filename;
    Mat imageInput;
    if (!inImgPath)
    {
        cout << "Can't load files" << endl;
    }
    while (getline(inImgPath,filename))
    {
        imgList.push_back(filename);
    }

    while (image_count < imgList.size())
    {
        filename = imgList[image_count++];
        // 用于观察检验输出
        cout << "image_count = " << image_count << endl;
        cout << filename.c_str() << endl;
        /* 输出检验*/
        //        cout << "-->count = " << count << endl;
        imageInput = imread(filename);

        //        imageSource = imread(filename);
        //        newimage = imageSource.clone();

        if (image_count == 1)  //读入第一张图片时获取图像宽高信息
        {
            image_size.width = imageInput.cols;
            image_size.height = imageInput.rows;
            cout << "image_size.width = " << image_size.width << endl;
            cout << "image_size.height = " << image_size.height << endl;
        }
        /* 提取角点 */
        if (0 == findChessboardCorners(imageInput,board_size,image_points_buf))
        {
            cout << "can not find chessboard corners!\n"; //找不到角点
            exit(1);
        }
        else
        {
            Mat view_gray;
            cvtColor(imageInput,view_gray,CV_RGB2GRAY);
            /* 亚像素精确化 */
            find4QuadCornerSubpix(view_gray,image_points_buf,Size(11,11)); //对粗提取的角点进行精确化
            image_points_seq.push_back(image_points_buf);  //保存亚像素角点
            /* 在图像上显示角点位置 */
            drawChessboardCorners(view_gray,board_size,image_points_buf,true); //用于在图片中标记角点
            imshow("Camera Calibration",view_gray);//显示图片
            waitKey(100);//暂停0.1S
        }




    int total = image_points_seq.size();
    cout << "total = " << total << endl;
    int CornerNum = board_size.width * board_size.height;  //每张图片上总角点数
    for (int ii=0 ; ii < total ;ii++)
    {
        cout << "--> No." << ii + 1 << " image's data -> "<<endl;
        if (0 == ii%CornerNum)// 77 是每幅图片的角点个数。此判断语句是为了输出 图片号，便于控制台观看
        {
            int i = -1;
            i = ii/CornerNum;
            int j=i+1;
        }

        else
        {
            cout.width(10);
        }
        //输出所有的角点
        cout << "CornerNum :" << CornerNum << endl;
        cout << " -->" << image_points_seq[ii][0].x << endl;
        cout << " -->" << image_points_seq[ii][0].y << endl << endl;

    }
    cout << endl <<"Extract corners conplete. \n";

    cout << "Star Calibrate......" << endl;
    /*棋盘三维信息*/
    Size square_size = Size(10,10);  /* 实际测量得到的标定板上每个棋盘格的大小 */
    vector<vector<Point3f>> object_points; /* 保存标定板上角点的三维坐标 */
    /*内外参数*/
    Mat cameraMatrix = Mat(3,3,CV_32FC1,Scalar::all(0)); /* 摄像机内参数矩阵 */
    vector<int> point_counts;  // 每幅图像中角点的数量
    Mat distCoeffs = Mat(1,5,CV_32FC1,Scalar::all(0)); /* 摄像机的5个畸变系数：k1,k2,p1,p2,k3 */
    vector<Mat> tvecsMat;  /* 每幅图像的旋转向量 */
    vector<Mat> rvecsMat; /* 每幅图像的平移向量 */
    /* 初始化标定板上角点的三维坐标 */
    int i, j, t;
    for (t = 0; t < image_count; t++)
    {
        vector<Point3f> tempPointSet;
        for (i = 0;i < board_size.height;i++)
        {
            for (j = 0;j<board_size.width;j++)
            {
                Point3f realPoint;
                /* 假设标定板放在世界坐标系中z=0的平面上 */
                realPoint.x = i * square_size.width;
                realPoint.y = j * square_size.height;
                realPoint.z = 0;
                tempPointSet.push_back(realPoint);
            }
        }
        object_points.push_back(tempPointSet);
    }
    /* 初始化每幅图像中的角点数量，假定每幅图像中都可以看到完整的标定板 */
    for (i = 0; i < image_count; i++)
    {
        point_counts.push_back(board_size.width * board_size.height);
    }

    /* 开始标定 */
    calibrateCamera(object_points,image_points_seq,image_size,cameraMatrix,distCoeffs,rvecsMat,tvecsMat,0);
    cout << "Evaluation Complate\n" << endl;
    //对标定结果进行评价
    cout << "Start evaluate calibration reault...\n" << endl;
    double total_err = 0.0; /* 所有图像的平均误差的总和 */
    double err = 0.0; /* 每幅图像的平均误差 */
    vector<Point2f> image_points2; /* 保存重新计算得到的投影点 */
    cout << "\t Each image's reprojection error:\n";
    fout << "Each image's reprojection error:\n";

    for (i = 0; i < image_count; i++)
    {
        vector<Point3f> tempPointSet=object_points[i];
        /* 通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点 */
        projectPoints(tempPointSet,rvecsMat[i],tvecsMat[i],cameraMatrix,distCoeffs,image_points2);
        /* 计算新的投影点和旧的投影点之间的误差*/
        vector<Point2f> tempImagePoint = image_points_seq[i];
        Mat tempImagePointMat = Mat(1,tempImagePoint.size(),CV_32FC2);
        Mat image_points2Mat = Mat(1,image_points2.size(), CV_32FC2);
        for (int j = 0 ; j < tempImagePoint.size(); j++)
        {
            image_points2Mat.at<Vec2f>(0,j) = Vec2f(image_points2[j].x, image_points2[j].y);
            tempImagePointMat.at<Vec2f>(0,j) = Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
        }
        err = norm(image_points2Mat, tempImagePointMat, NORM_L2);
        total_err += err/=  point_counts[i];
        cout << "NO." << i+1 << " image's average reprojection error:" << err << " pixels" << endl;
        fout << "NO." << i+1 << " image's average reprojection error:" << err << " pixels" << endl;
    }
    cout << "Overall average reprojection error:" << total_err/image_count << " pixels" << endl;
    fout << endl;
    fout << "Overall average reprojection error:" << total_err/image_count << " pixels" << endl << endl;
    cout << "Evaluation Complete." << endl;
    //保存定标结果
    cout << "Start saving calibration results......." << endl;
    Mat rotation_matrix = Mat(3,3,CV_32FC1, Scalar::all(0)); /* 保存每幅图像的旋转矩阵 */
    fout << "Camera Intrinsics：" << endl;
    fout << cameraMatrix << endl << endl;
    fout << "Distortion coefficient：\n";
    fout << distCoeffs << endl << endl << endl;
    for (int i = 0; i < image_count; i++)
    {
        fout << "NO." << i+1 << " Image's Rotation vector:" << endl;
        fout << tvecsMat[i] << endl;
        /* 将旋转向量转换为相对应的旋转矩阵 */
        Rodrigues(tvecsMat[i],rotation_matrix);
        fout << "NO." << i+1 << " Image's Rotation Matrix:" << endl;
        fout << rotation_matrix << endl;
        fout << "NO." << i+1 << " Image's Translation vector Matrix:" << endl;
        fout << rvecsMat[i] << endl << endl;
    }
    cout << "Saving Complete." << endl;
    fout << endl;
    /*****************************************************************************
                                   显示标定结果
******************************************************************************/
    Mat mapx = Mat(image_size,CV_32FC1);
    Mat mapy = Mat(image_size,CV_32FC1);
    Mat R = Mat::eye(3,3,CV_32F);
    std::cout << "Saving calibrated image..." << endl;
    string imageFileName;
    std::stringstream StrStm;


    cout<< "Frame #" << i+1 << " ... " << endl;
    initUndistortRectifyMap(cameraMatrix,distCoeffs,R,cameraMatrix,image_size,CV_32FC1,mapx,mapy);
    string filePath = (".");

    Mat imageSource = imread(filename);
    Mat newimage = imageSource.clone();

    remap(imageSource,newimage,mapx, mapy, INTER_LINEAR);
    //        undistort(imageSource,newimage,cameraMatrix,distCoeffs);
    StrStm.clear();
    filePath.clear();
    StrStm << i;
    StrStm >> imageFileName;
    imageFileName += "_d.jpg";
    imwrite(imageFileName,newimage);
}
    return 0;
}
