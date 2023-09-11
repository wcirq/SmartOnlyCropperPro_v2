//
// Created by qiulinmin on 8/1/17.
//

#ifndef CROPPER_DOC_SCANNER_H
#define CROPPER_DOC_SCANNER_H

#include <opencv2/opencv.hpp>
#include "opencv_utils.h"
#include <map>

namespace scanner{

    struct RectangularInfo
    {
        std::vector<Point> vertex;
        std::vector<Point> edges;

        RectangularInfo() {}
        RectangularInfo(std::vector<Point> vertex, std::vector<Point> edges): vertex(vertex), edges(edges) {}

    };

    class Scanner {
    public:
        int resizeThreshold = 500;

        Scanner(cv::Mat& bitmap, bool canny);
        virtual ~Scanner();
        std::vector<cv::Point> scanPoint(cv::Mat previewBitmap);
        RectangularInfo scanPoint2(cv::Mat previewBitmap);  // python版本
        cv::Mat ImageSharp(cv::Mat src,int nAmount);
        cv::Mat ReduceBackGroundAlgorithm(cv::Mat src,int flag);
    private:
        cv::Mat srcBitmap;
        float widthScale = 1.0f;
        float heightScale = 1.0f;

        bool canny = false;

        bool isHisEqual = false;

        std::vector<cv::Point> scanPointDegrade();

        cv::Mat resizeImage();

        cv::Mat preprocessedImage(cv::Mat &image, int cannyValue, int blurValue);

        cv::Point choosePoint(const cv::Point& center, std::vector<cv::Point> &points, int type);

        std::vector<cv::Point> selectPoints(std::vector<cv::Point> points);

        std::vector<cv::Point> sortPointClockwise(std::vector<cv::Point> vector);

        long long pointSideLine(cv::Point& lineP1, cv::Point& lineP2, cv::Point& point);

        Vec2d extensionLine(Vec4i& vec);

        void mergeLines(vector<Vec4i>& inArray, vector<Vec4i>& outArray);

        void linesLengthFilter(vector<Vec4i> &input, int minLength);

        vector<vector<Point2f>> findQuadrilateral(vector<Vec4i> &input, vector<vector<Vec4i>> &output, int allowExtension);

        vector<Point2f> getLinesNode(Vec4i &line1, Vec4i &line2, Vec4i &line3, Vec4i &line4, int extension);

        Point2d getNodePoint(Vec4i &line1, Vec4i &line2);

        double getPointsDistance(double x1, double y1, double x2, double y2);

        double getPointsDistanceV2(Point2d& point1, Point2d& point2);

        bool checkPoint(Vec4i &line1, Vec4i &line2, const Point2d& point, int extension);

        bool isConvexRect(vector<Point2f> &points);

        void restoreLineSize(vector<Vec4i> &lines, vector<Vec4i> &output);

        bool isPointInLine(Point2d &point, Point2d &point1, Point2d &point2);
    };

}

#endif //CROPPER_DOC_SCANNER_H
