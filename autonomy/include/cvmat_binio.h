/* 
Saves OpenCV mat to binary file.

From Nadav B stackoverflow answer:
https://stackoverflow.com/questions/16312904/how-to-write-a-float-mat-to-a-file-in-opencv/16314041

 */
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

/*
Will save in the file:
cols\n
rows\n
elemSize\n
type\n
DATA
*/
void serializeMatbin(cv::Mat& mat, std::string filename){
    if (!mat.isContinuous()) {
        std::cout << "Not implemented yet" << std::endl;
        exit(1);
    }

    int elemSizeInBytes = (int)mat.elemSize();
    int elemType        = (int)mat.type();
    int dataSize        = (int)(mat.cols * mat.rows * mat.elemSize());

    FILE* FP = fopen(filename.c_str(), "wb");
    int sizeImg[4] = {mat.cols, mat.rows, elemSizeInBytes, elemType };
    fwrite(/* buffer */ sizeImg, /* how many elements */ 4, /* size of each element */ sizeof(int), /* file */ FP);
    fwrite(mat.data, mat.cols * mat.rows, elemSizeInBytes, FP);
    fclose(FP);
}

cv::Mat deserializeMatbin(std::string filename){
    FILE* fp = fopen(filename.c_str(), "rb");
    if (!fp) {
      throw std::runtime_error("File open error on "+filename);
    }
    int header[4];
    if (4!=fread(header, sizeof(int), 4, fp)) {
      throw std::runtime_error("Can't read file header on "+filename);
    }
    int cols            = header[0]; 
    int rows            = header[1];
    int elemSizeInBytes = header[2];
    int elemType        = header[3];

    //std::cout << "rows="<<rows<<" cols="<<cols<<" elemSizeInBytes=" << elemSizeInBytes << std::endl;

    cv::Mat outputMat = cv::Mat::ones(rows, cols, elemType);

    size_t result = fread(outputMat.data, elemSizeInBytes, (size_t)(cols * rows), fp);

    if (result != (size_t)(cols * rows)) {
        throw std::runtime_error("Data read error on "+filename);
    }

    std::cout << ((float*)outputMat.data)[200] << std::endl;
    fclose(fp);
    return outputMat;
}

void testSerializeMatbin(){
    cv::Mat a = cv::Mat::ones(/*cols*/ 10, /* rows */ 5, CV_32F) * -2;
    std::string filename = "test.matbin";
    serializeMatbin(a, filename);
    cv::Mat b = deserializeMatbin(filename);
    std::cout << "Rows: " << b.rows << " Cols: " << b.cols << " type: " << b.type()<< std::endl;
}

