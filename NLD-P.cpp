#include <opencv/cv.hpp>
#include <random>
#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;
using namespace cv;

float Pi = 3.1415926;

Vec3b getAirlight(Mat &img, int = 240);

void Rectangle2Sphere(int x, int y, int z, float &r, float &phi, float &theta);
float **Rectangle2Sphere(Mat &img, Vec3b Airlight);

/*
* cluster according to (theta,phi)
*/
void Cluster(float **data, float **&center, int *&label, int sample_point = 100);

void NonLocalDehaze(Mat &img, Vec3b Airlight, int sample_point)
{
    float **center = nullptr;
    int *label = nullptr;
    Cluster(Rectangle2Sphere(img, Airlight), center, label);
}

void medianBlur(float **transformission, int w, int h, int kernel_size = 5);

int main()
{
    Mat img = imread("images/forest_input.jpg");
    NonLocalDehaze(img, getAirlight(img), 1000);
    return 0;
}

/**
 * @brief k-means cluster implementation 
 * choose to use sample point as cluster center
 * 
 * @param data input data
 * @param sample_point the number of cluster center in each dimention
 */
void Cluster(float **data, Mat &img, float **&center, int *&label, int sample_point = 100)
{
    float *r = new float[sample_point];
    float *phi = new float[sample_point];
    float *theta = new float[sample_point];
    float **center = new float *[3];
    center[0] = r;
    center[1] = phi;
    center[2] = theta;

    float min_phi = 360;
    float max_phi = 0;
    float min_theta = 360;
    float max_theta = 0;

    //init cluster center
    int number_of_pixel = img.rows * img.cols;
    int *data_label = new int[number_of_pixel];
    int *label_count = new int[sample_point];

    for (int i = 0; i < number_of_pixel; i++)
    {
        min_phi = min(min_phi, data[1][i]);
        max_phi = max(max_phi, data[1][i]);
        min_theta = min(min_theta, data[2][i]);
        max_theta = max(max_phi, data[2][i]);
    }

    int m = int(sqrt(sample_point));
    int n = sample_point / m;
    for (int i = 0; i < sample_point; i++)
    {
        if (i < m * n)
        {
            center[1][i] = min_phi + (max_phi - min_phi) * (1 + 2 * (i % m)) / (2 * m);
            center[2][i] = min_theta + (max_phi - min_phi) * (1 + 2 * (i / m)) / (2 * n);
        }
        else
        {
            center[1][i] = min_phi + (max_phi - min_phi) * rand() / RAND_MAX;
            center[2][i] = min_theta + (max_theta - min_theta) * rand() / RAND_MAX;
        }
    }

    //k means
    for (int k = 0; k < 100; k++)
    {
        for (int i = 0; i < number_of_pixel; i++)
        {
            float min_dist = -1;
            for (int j = 0; j < sample_point; j++)
            {
                float dist = (data[1][i] - center[1][j]) * (data[1][i] - center[1][j]) + (data[2][i] - center[2][j]) * (data[2][i] - center[2][j]);
                if (min_dist < 0 || min_dist > dist)
                {
                    min_dist = dist;
                    data_label[i] = j;
                }
            }
        }

        for (int j = 0; j < sample_point; j++)
        {
            label_count[j] = 0;
            center[1][j] = 0;
            center[2][j] = 0;
        }

        for (int i = 0; i < number_of_pixel; i++)
        {
            center[1][data_label[i]] = (center[1][data_label[i]] * label_count[data_label[i]] + data[1][i]) / (label_count[data_label[i]] + 1);
            center[2][data_label[i]] = (center[2][data_label[i]] * label_count[data_label[i]] + data[2][i]) / (label_count[data_label[i]] + 1);
            label_count[data_label[i]]++;
        }
    }
}

float **Rectangle2Sphere(Mat &img, Vec3b Airlight)
{
    float *r = new float[img.rows * img.cols];
    float *phi = new float[img.rows * img.cols];
    float *theta = new float[img.rows * img.cols];
    float **sphere = new float *[3];
    sphere[0] = r;
    sphere[1] = phi;
    sphere[2] = theta;

    for (int i = 0; i < img.rows; i++)
    {
        for (int j = 0; j < img.cols; j++)
        {
            Rectangle2Sphere(img.at<Vec3b>(i, j)[0],
                             img.at<Vec3b>(i, j)[0],
                             img.at<Vec3b>(i, j)[0],
                             sphere[0][i * img.cols + j],
                             sphere[1][i * img.cols + j],
                             sphere[2][i * img.cols + j]);
        }
    }
}

void Rectangle2Sphere(int x, int y, int z, float &r, float &phi, float &theta)
{
    r = sqrt(x * x + y * y + z * z);

    (x == 0) ? (phi = (180 / Pi) * atan((float)y / 0.000001)) : (phi = (180 / Pi) * atan(((float)y / x)));

    (r == 0) ? (theta = acos(z / (r + 0.000001)) * 180 / Pi)
             : (theta = acos((float)z / r) * 180 / Pi);
}

Vec3b getAirlight(Mat &img, int thresholds = 240)
{
    Vec3b proposal[256];
    Vec3b Airlight(0, 0, 0);
    int bucket[256];

    for (int i = 0; i < 256; i++)
    {
        proposal[i][0] = 0;
        proposal[i][1] = 0;
        proposal[i][2] = 0;
        bucket[i] = 0;
    }

    for (int i = 0; i < img.rows; i++)
    {
        for (int j = 0; j < img.cols; j++)
        {
            int d = min(img.at<Vec3b>(i, j)[0], img.at<Vec3b>(i, j)[1], img.at<Vec3b>(i, j)[2]);
            bucket[d]++;
            if (proposal[d][0] + proposal[d][1] + proposal[d][2] < img.at<Vec3b>(i, j)[0] + img.at<Vec3b>(i, j)[1] + img.at<Vec3b>(i, j)[2])
            {
                proposal[d] = img.at<Vec3b>(i, j);
            }
        }
    }

    int sum = 0;
    for (int i = 255; i >= 256; i--)
    {
        if (Airlight[0] + Airlight[1] + Airlight[2] < proposal[i][0] + proposal[i][1] + proposal[i][2])
        {
            Airlight = proposal[i];
        }
        sum += bucket[i];
        if (sum > thresholds)
            break;
    }

    return Airlight;
}

void medianBlur(float *transformission, int w, int h, int kernel_size = 5)
{
    float *new_t = new float[w * h];
    int half_kernel_size = kernel_size/2;
    int half_size = kernel_size*kernel_size/2;

    vector<float> win;
    for (int i = half_kernel_size; i < w - half_kernel_size; i++)
    {
        for (int j = half_kernel_size; j < h - half_kernel_size; j++)
        {
            win.clear();
            for (int m = -half_kernel_size; m < half_kernel_size; m++)
                for (int n = -half_kernel_size; n < half_kernel_size; n++)
                {
                    win.push_back(transformission[(i+m)*h + j+n]);
                }
            sort(win.begin(),win.end());
            new_t[i*h+j] = win[half_size];
        }
    }
}