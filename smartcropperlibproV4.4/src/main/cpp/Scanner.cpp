//
// Created by qiulinmin on 8/1/17.
//
#include <Scanner.h>
#include <algorithm>
#include <android/log.h>

#define TAG "ssss"
// 定义info信息
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,TAG,__VA_ARGS__)
// 定义debug信息
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, TAG, __VA_ARGS__)
// 定义error信息
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR,TAG,__VA_ARGS__)

using namespace scanner;
using namespace cv;
using namespace std;

const double EPS = 1e-7;

static bool sortByArea(const vector<Point2f> &v1, const vector<Point2f> &v2) {
    double v1Area = fabs(contourArea(Mat(v1)));
    double v2Area = fabs(contourArea(Mat(v2)));
    return v1Area > v2Area;
}

static bool sortByAreaInt(const vector<Point> &v1, const vector<Point> &v2) {
    double v1Area = fabs(contourArea(Mat(v1)));
    double v2Area = fabs(contourArea(Mat(v2)));
    return v1Area > v2Area;
}

Scanner::Scanner(cv::Mat& bitmap, bool canny) {
    srcBitmap = bitmap;
    Scanner::canny = canny;
}

Scanner::~Scanner() {
    srcBitmap.release();
};

vector<Point> Scanner::scanPoint(cv::Mat previewBitmap) {
    Mat image = resizeImage();
    Mat grayMat;
    cvtColor(image, grayMat, COLOR_BGR2GRAY);
//    Mat element = getStructuringElement(MORPH_RECT,Size(3,3));
//    dilate(grayMat, grayMat, element);
//    element = getStructuringElement(MORPH_RECT, Size(5, 5));
//    erode(grayMat, grayMat, element);
    // 检测线段
    vector<Vec4i> lines;
    HoughLinesP(grayMat, lines, 1, CV_PI / 180, 50, 50, 10);
    // 合并线段
    vector<Vec4i> mergedLines;
    mergeLines(lines, mergedLines);
    // 过滤较短的线段
    linesLengthFilter(mergedLines, 50);
    // 寻找四边形
    vector<vector<Vec4i>> rectLines;
    vector<vector<Point2f>> rects = findQuadrilateral(mergedLines, rectLines, 120);
    vector<Point> result;
    if (!rects.empty()) {
        vector<vector<Point>> rectsIntSort;
        for (const vector<Point2f>& ps : rects) {
            vector<Point> psi;
            for (const Point2f& p : ps) {
                Point ip(round(p.x), round(p.y));
                psi.emplace_back(ip);
            }
            psi = sortPointClockwise(psi);
            double area = fabs(contourArea(Mat(psi), true));
            double arcL = fabs(arcLength(Mat(psi), true));
            LOGD("area: %f", area);
            LOGD("arcLength: %f", arcL);
            if (arcL > 1000 && area > 50000) {
                rectsIntSort.emplace_back(psi);
            }
        }
        if (!rectsIntSort.empty()) {
            std::sort(rectsIntSort.begin(), rectsIntSort.end(), sortByAreaInt);
            result = rectsIntSort[0];
        }
    }
    if (result.empty()) {
        result = sortPointClockwise(scanPointDegrade());
        LOGD("scanPointDegrade!!!!");
    }

//    double v1Area = fabs(contourArea(rects[0]));
//    double v2Area = fabs(contourArea(rects[2]));
//
//    double v1Le = fabs(arcLength(Mat(rects[0]), true));
//    double v2L2 = fabs(contourArea(Mat(rects[2]), true));
//
//    LOGD("area: %f, %f", v1Area, v2Area);
//    LOGD("arcLength: %f, %f", v1Le, v2L2);

    // restore
    for (Point& p : result) {
        p.x = round(p.x * widthScale);
        p.y = round(p.y * heightScale);
    }
    if (!previewBitmap.empty()) {
        vector<Vec4i> restoreLines;
        restoreLineSize(mergedLines, restoreLines);
        drawLines(previewBitmap, restoreLines, 0, 0);
        drawPoint(previewBitmap, result, Scalar(0, 255,255));
    }
    return result;
}

RectangularInfo Scanner::scanPoint2(cv::Mat previewBitmap) {
    Mat image = resizeImage();
	std::vector<cv::Point> result;
	cv::Mat image_gray, image_binary;
    double rectArea = image.rows*image.cols;
	// 将彩图转为灰度图
	if (image.channels() == 3) {
		cv::cvtColor(image, image_gray, cv::COLOR_RGB2GRAY);
	}
	else if (image.channels() == 4) {
		cv::cvtColor(image, image_gray, cv::COLOR_RGBA2GRAY);
	}
	else {
		image_gray = image.clone();
	}

	// 二值化
	cv::threshold(image_gray, image_binary, 127, 255, cv::THRESH_BINARY);

	std::vector< std::vector< cv::Point> > contours;
	cv::findContours(
		image_binary,
		contours,
		cv::RETR_EXTERNAL,
		cv::CHAIN_APPROX_NONE
	);
	std::vector< cv::Point> max_contour;
	double max_area = 0;
	for (int i = 0; i < contours.size(); ++i) {
		double area = cv::contourArea(contours[i]);
		if (area > max_area) {
			max_area = area;
			max_contour = contours[i];
		}
	}
	if (max_area / rectArea < 0.03) {
        return RectangularInfo();  // 返回空
	}
    if (max_contour.size()==0)
    {
        return RectangularInfo();  // 返回空
    }
	std::vector<cv::Point> approxCurve; //存放逼近曲线的数组
	cv::approxPolyDP(max_contour, approxCurve, 25, true);
    if (approxCurve.size()!=4)
    {
        Point2f p[4];
        p[0] = Point2f(0, 0);
        p[1] = Point2f(image.cols-1, 0);
        p[2] = Point2f(image.cols-1, image.rows-1);
        p[3] = Point2f(0, image.rows-1);
        result.push_back(p[0]);
        result.push_back(p[1]);
        result.push_back(p[2]);
        result.push_back(p[3]);

        max_contour.clear(); // 清除掉边即图像边长组成的轮廓
    }
    else
    {
        result.assign(approxCurve.begin(), approxCurve.end());
    }
    result = sortPointClockwise(result);
    // restore
    for (Point& p : result) {
        p.x = round(p.x * widthScale);
        p.y = round(p.y * heightScale);
    }
    for (cv::Point& p: max_contour) {
        p.x = round(p.x * widthScale);
        p.y = round(p.y * heightScale);
    }
    return RectangularInfo(result, max_contour);
}

vector<Point> Scanner::scanPointDegrade() {
    vector<Point> result;
    int cannyValue[] = {100, 150, 300};
    int blurValue[] = {3, 7, 11, 15};
    //缩小图片尺寸
    Mat image = resizeImage();
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 4; j++){
            //预处理图片
            Mat scanImage = preprocessedImage(image, cannyValue[i], blurValue[j]);
            vector<vector<Point>> contours;
            //提取边框
            findContours(scanImage, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
            //按面积排序
            std::sort(contours.begin(), contours.end(), sortByAreaInt);
            if (!contours.empty()) {
                vector<Point> contour = contours[0];
                double arc = arcLength(contour, true);
                vector<Point> outDP;
                //多变形逼近
                approxPolyDP(Mat(contour), outDP, 0.01 * arc, true);
                //筛选去除相近的点
                vector<Point> selectedPoints = selectPoints(outDP);
                if (selectedPoints.size() != 4) {
                    //如果筛选出来之后不是四边形
                    continue;
                } else {
                    int widthMin = selectedPoints[0].x;
                    int widthMax = selectedPoints[0].x;
                    int heightMin = selectedPoints[0].y;
                    int heightMax = selectedPoints[0].y;
                    for (int k = 0; k < 4; k++) {
                        if (selectedPoints[k].x < widthMin) {
                            widthMin = selectedPoints[k].x;
                        }
                        if (selectedPoints[k].x > widthMax) {
                            widthMax = selectedPoints[k].x;
                        }
                        if (selectedPoints[k].y < heightMin) {
                            heightMin = selectedPoints[k].y;
                        }
                        if (selectedPoints[k].y > heightMax) {
                            heightMax = selectedPoints[k].y;
                        }
                    }
                    //选择区域外围矩形面积
                    int selectArea = (widthMax - widthMin) * (heightMax - heightMin);
                    int imageArea = scanImage.cols * scanImage.rows;
                    if (selectArea < (imageArea / 20)) {
                        result.clear();
                        //筛选出来的区域太小
                        continue;
                    } else {
                        result = selectedPoints;
                        if (result.size() != 4) {
                            Point2f p[4];
                            p[0] = Point2f(0, 0);
                            p[1] = Point2f(image.cols, 0);
                            p[2] = Point2f(image.cols, image.rows);
                            p[3] = Point2f(0, image.rows);
                            result.push_back(p[0]);
                            result.push_back(p[1]);
                            result.push_back(p[2]);
                            result.push_back(p[3]);
                        }
                        return result;
                    }
                }
            }
        }
    }
    //当没选出所需要区域时，如果还没做过直方图均衡化则尝试使用均衡化，但该操作只执行一次，若还无效，则判定为图片不能裁出有效区域，返回整张图
    if (!isHisEqual){
        isHisEqual = true;
        return scanPointDegrade();
    }
    if (result.size() != 4) {
        Point2f p[4];
        p[0] = Point2f(0, 0);
        p[1] = Point2f(image.cols, 0);
        p[2] = Point2f(image.cols, image.rows);
        p[3] = Point2f(0, image.rows);
        result.push_back(p[0]);
        result.push_back(p[1]);
        result.push_back(p[2]);
        result.push_back(p[3]);
    }
    return result;
}

Mat Scanner::resizeImage() {
    int width = srcBitmap.cols;
    int height = srcBitmap.rows;
    widthScale = float(width) / float(resizeThreshold);
    heightScale = float(height) / float(resizeThreshold);
    Size size(resizeThreshold, resizeThreshold);
    Mat resizedBitmap(size, CV_8UC3);
    resize(srcBitmap, resizedBitmap, size);
    return resizedBitmap;
}

Mat Scanner::preprocessedImage(Mat &image, int cannyValue, int blurValue) {
    Mat grayMat;
    cvtColor(image, grayMat, COLOR_BGR2GRAY);
    if (!canny) {
        return grayMat;
    }
    if (isHisEqual){
        equalizeHist(grayMat, grayMat);
    }
    Mat blurMat;
    GaussianBlur(grayMat, blurMat, Size(blurValue, blurValue), 0);
    Mat cannyMat;
    Canny(blurMat, cannyMat, 50, cannyValue, 3);
    Mat thresholdMat;
    threshold(cannyMat, thresholdMat, 0, 255, THRESH_OTSU);
    return thresholdMat;
}

vector<Point> Scanner::selectPoints(vector<Point> points) {
    if (points.size() > 4) {
        Point &p = points[0];
        int minX = p.x;
        int maxX = p.x;
        int minY = p.y;
        int maxY = p.y;
        //得到一个矩形去包住所有点
        for (int i = 1; i < points.size(); i++) {
            if (points[i].x < minX) {
                minX = points[i].x;
            }
            if (points[i].x > maxX) {
                maxX = points[i].x;
            }
            if (points[i].y < minY) {
                minY = points[i].y;
            }
            if (points[i].y > maxY) {
                maxY = points[i].y;
            }
        }
        //矩形中心点
        Point center = Point((minX + maxX) / 2, (minY + maxY) / 2);
        //分别得出左上，左下，右上，右下四堆中的结果点
        Point p0 = choosePoint(center, points, 0);
        Point p1 = choosePoint(center, points, 1);
        Point p2 = choosePoint(center, points, 2);
        Point p3 = choosePoint(center, points, 3);
        points.clear();
        //如果得到的点不是０，即是得到的结果点
        if (!(p0.x == 0 && p0.y == 0)){
            points.push_back(p0);
        }
        if (!(p1.x == 0 && p1.y == 0)){
            points.push_back(p1);
        }
        if (!(p2.x == 0 && p2.y == 0)){
            points.push_back(p2);
        }
        if (!(p3.x == 0 && p3.y == 0)){
            points.push_back(p3);
        }
    }
    return points;
}

//type代表左上，左下，右上，右下等方位
Point Scanner::choosePoint(const Point& center, std::vector<cv::Point> &points, int type) {
    int index = -1;
    int minDis = 0;
    //四个堆都是选择距离中心点较远的点
    if (type == 0) {
        for (int i = 0; i < points.size(); i++) {
            if (points[i].x < center.x && points[i].y < center.y) {
                int dis = static_cast<int>(sqrt(pow((points[i].x - center.x), 2) + pow((points[i].y - center.y), 2)));
                if (dis > minDis){
                    index = i;
                    minDis = dis;
                }
            }
        }
    } else if (type == 1) {
        for (int i = 0; i < points.size(); i++) {
            if (points[i].x < center.x && points[i].y > center.y) {
                int dis = static_cast<int>(sqrt(pow((points[i].x - center.x), 2) + pow((points[i].y - center.y), 2)));
                if (dis > minDis){
                    index = i;
                    minDis = dis;
                }
            }
        }
    } else if (type == 2) {
        for (int i = 0; i < points.size(); i++) {
            if (points[i].x > center.x && points[i].y < center.y) {
                int dis = static_cast<int>(sqrt(pow((points[i].x - center.x), 2) + pow((points[i].y - center.y), 2)));
                if (dis > minDis){
                    index = i;
                    minDis = dis;
                }
            }
        }

    } else if (type == 3) {
        for (int i = 0; i < points.size(); i++) {
            if (points[i].x > center.x && points[i].y > center.y) {
                int dis = static_cast<int>(sqrt(pow((points[i].x - center.x), 2) + pow((points[i].y - center.y), 2)));
                if (dis > minDis){
                    index = i;
                    minDis = dis;
                }
            }
        }
    }

    if (index != -1){
        return {points[index].x, points[index].y};
    }
    return {0, 0};
}

vector<Point> Scanner::sortPointClockwise(vector<Point> points) {
    if (points.size() != 4) {
        return points;
    }

    Point unFoundPoint(-1, -1);  // 初始化为-1，防止points中有（0,0),造成排序失败
    vector<Point> result = {unFoundPoint, unFoundPoint, unFoundPoint, unFoundPoint};

    long minDistance = -1;
    for(Point &point : points) {
        long distance = point.x * point.x + point.y * point.y;
        if(minDistance == -1 || distance < minDistance) {
            result[0] = point;
            minDistance = distance;
        }
    }
    if (result[0] != unFoundPoint) {
        Point &leftTop = result[0];
        points.erase(std::remove(points.begin(), points.end(), leftTop), points.end());
        if ((pointSideLine(leftTop, points[0], points[1]) * pointSideLine(leftTop, points[0], points[2])) < 0) {
            result[2] = points[0];
        } else if ((pointSideLine(leftTop, points[1], points[0]) * pointSideLine(leftTop, points[1], points[2])) < 0) {
            result[2] = points[1];
        } else if ((pointSideLine(leftTop, points[2], points[0]) * pointSideLine(leftTop, points[2], points[1])) < 0) {
            result[2] = points[2];
        }
    }
    if (result[0] != unFoundPoint && result[2] != unFoundPoint) {
        Point &leftTop = result[0];
        Point &rightBottom = result[2];
        points.erase(std::remove(points.begin(), points.end(), rightBottom), points.end());
        if (pointSideLine(leftTop, rightBottom, points[0]) > 0) {
            result[1] = points[0];
            result[3] = points[1];
        } else {
            result[1] = points[1];
            result[3] = points[0];
        }
    }

    if (result[0] != unFoundPoint && result[1] != unFoundPoint && result[2] != unFoundPoint && result[3] != unFoundPoint) {
        return result;
    }

    return points;
}

long long Scanner::pointSideLine(Point &lineP1, Point &lineP2, Point &point) {
    long x1 = lineP1.x;
    long y1 = lineP1.y;
    long x2 = lineP2.x;
    long y2 = lineP2.y;
    long x = point.x;
    long y = point.y;
    return (x - x1)*(y2 - y1) - (y - y1)*(x2 - x1);
}

Vec2d Scanner::extensionLine(Vec4i& vec) {
    int x1 = vec[0];
    int y1 = vec[1];
    int x2 = vec[2];
    int y2 = vec[3];
    double k = (y1 - y2)/(x1 - x2 + 0.0001);
    double b = y1 - k * x1;
    return {k, b};
}

void Scanner::mergeLines(vector<Vec4i>& lines, vector<Vec4i>& outArray) {
    if (lines.empty()) {
        return;
    }
    // 延长线
    vector<Vec2d> extensionLines;
    extensionLines.reserve(lines.size());
    for(auto & line : lines) {
        extensionLines.emplace_back(extensionLine(line));
    }
    // 合并延长线
    vector<int> mergedIndex;
    for (size_t i = 0; i < extensionLines.size() - 1; i++) {
        if (find(mergedIndex.begin(), mergedIndex.end(), i) != mergedIndex.end()) {
            continue;
        }
        Vec2d line1 = extensionLines[i];
        Vec4i originLine1 = lines[i];
        double k1 = line1[0];
        double b1 = line1[1];
        vector<Vec4i> preMerge;
        for (size_t j = i+1; j < extensionLines.size(); j++) {
            Vec2d line2 = extensionLines[j];

            double k2 = line2[0];
            double b2 = line2[1];
            if (abs(atan(k1) - atan(k2)) < CV_PI / 45 && (abs(b1 - b2) < 8 || abs(b1/k1 - b2/k2) < 8)) {
                preMerge.emplace_back(lines[j]);
                mergedIndex.emplace_back(j);
            }
        }
        if (!preMerge.empty()) {
            int minX = fmin(originLine1[0], originLine1[2]);
            int minY = fmin(originLine1[1], originLine1[3]);
            int maxX = fmax(originLine1[0], originLine1[2]);
            int maxY = fmin(originLine1[1], originLine1[3]);
            for(auto mergeLine : preMerge) {
                int x1 = mergeLine[0];
                int x2 = mergeLine[2];
                int y1 = mergeLine[1];
                int y2 = mergeLine[3];
                if (fmin(x1, x2) < minX) {
                    minX = fmin(x1, x2);
                }
                if (fmax(x1, x2) > maxX) {
                    maxX = fmax(x1, x2);
                }
                if (fmin(y1, y2) < minY) {
                    minY = fmin(y1, y2);
                }
                if (fmax(y1, y2) > maxY) {
                    maxY = fmax(y1, y2);
                }
            }
            double x1 = minX;
            double y1 = k1 * x1 + b1;
            double x2 = maxX;
            double y2 = k1 * x2 + b1;
            if (abs(k1) > 1) {
                y1 = minY;
                y2 = maxY;
                x1 = (y1 - b1) / k1;
                x2 = (y2 - b1) / k1;
            }
            Vec4i mergedResultLine(x1 ,y1, x2, y2);
            outArray.emplace_back(mergedResultLine);
        } else {
            outArray.emplace_back(originLine1);
        }
    }
}

void Scanner::linesLengthFilter(vector<Vec4i> &input, int minLength) {
    std::vector<Vec4i>::const_iterator it = input.begin();
    while(it != input.end()) {
        double x1 = (*it)[0];
        double y1 = (*it)[1];
        double x2 = (*it)[2];
        double y2 = (*it)[3];
        double distance = sqrt(pow((x2 - x1),2) + pow((y2 - y1),2));
        if (distance < minLength) {
            it = input.erase(it);
        } else {
            ++it;
        }
    }
}

vector<vector<Point2f>> Scanner::findQuadrilateral(vector<Vec4i> &input, vector<vector<Vec4i>> &output, int allowExtension) {
    vector<vector<Point2f>> result;
    int iSize = input.size();
    LOGD("input size: %d", input.size());
    for (int i = 0; i < iSize - 3; ++i) {
        for (int j = i + 1; j < iSize - 2; ++j) {
            for (int k = j + 1; k < iSize - 1; ++k) {
                for (int m = k + 1; m < iSize ; ++m) {
                    Vec4i line1 = input[i];
                    Vec4i line2 = input[j];
                    Vec4i line3 = input[k];
                    Vec4i line4 = input[m];
                    vector<Point2f> nodes = getLinesNode(line1, line2, line3, line4, allowExtension);
                    if (isConvexRect(nodes)) {
                        result.emplace_back(nodes);
                        vector<Vec4i> rec = {line1,line2,line3,line4};
                        output.emplace_back(rec);
                    }
                }
            }
        }
    }
    LOGD("output size: %d", output.size());
    return result;
}

vector<Point2f> Scanner::getLinesNode(Vec4i &line1, Vec4i &line2, Vec4i &line3, Vec4i &line4, int extension) {
    vector<Point2f> result;
    vector<Vec4i> lines;
    lines.emplace_back(line1);
    lines.emplace_back(line2);
    lines.emplace_back(line3);
    lines.emplace_back(line4);
    for (int i = 0; i < lines.size() - 1; ++i) {
        Vec4i linei = lines[i];
        for (int j = i+1; j < lines.size(); ++j) {
            Vec4i linej = lines[j];
            Point2d point = getNodePoint(linei, linej);
            if (point.x == -1 && point.y == -1) {
                continue;
            }
            if (checkPoint(linei, linej, point, extension)) {
                result.emplace_back(cv::Point2f((float)point.x, (float)point.y));
            }
        }
    }

    return result;
}

Point2d Scanner::getNodePoint(Vec4i &line1, Vec4i &line2) {
    double x1 = line1[0];
    double y1 = line1[1];
    double x2 = line1[2];
    double y2 = line1[3];
    double x3 = line2[0];
    double y3 = line2[1];
    double x4 = line2[2];
    double y4 = line2[3];
//    LOGD("line1: (%f, %f) -  (%f, %f)", x1, y1, x2, y2);
//    LOGD("line2: (%f, %f) -  (%f, %f)", x3, y3, x4, y4);

    double a1 = y2 - y1;
    double b1 = x1 - x2;
    double c1 = x2*y1-x1*y2;
    double a2 = y4 - y3;
    double b2 = x3 - x4;
    double c2 = x4*y3-x3*y4;

//    LOGD("a1:%f", a1);
//    LOGD("b1:%f", b1);
//    LOGD("c1:%f", c1);
//
//    LOGD("a2:%f", a2);
//    LOGD("b2:%f", b2);
//    LOGD("c2:%f", c2);

    double d = (a1 * b2 - a2 * b1);
//    LOGD("d:%f", d);
    if (abs(d) < EPS) {
        return {-1, -1};
    }

    double x = (c2 * b1 - c1 * b2) / d;
    double y = (c1 * a2 - c2 * a1) / d;
//    LOGD("point: (%f, %f)", x, y);
    return {x, y};
}

double Scanner::getPointsDistanceV2(Point2d& point1, Point2d& point2) {
    return getPointsDistance(point1.x, point1.y, point2.x, point2.y);
}

double Scanner::getPointsDistance(double x1, double y1, double x2, double y2) {
    return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
}

bool Scanner::checkPoint(Vec4i &line1, Vec4i &line2, const Point2d& point, int extension) {
    double x1 = line1[0];
    double y1 = line1[1];
    double x2 = line1[2];
    double y2 = line1[3];
    double x3 = line2[0];
    double y3 = line2[1];
    double x4 = line2[2];
    double y4 = line2[3];
    double x = point.x;
    double y = point.y;

    if (x < min(x1, x2) || x > max(x1, x2)) {
        double distance = min(getPointsDistance(x, y, x1, y1), getPointsDistance(x, y, x2, y2));
        if (distance > extension) {
            return false;
        }
    }
    if (x < min(x3, x4) || x > max(x3, x4)) {
        double distance = min(getPointsDistance(x, y, x3, y3), getPointsDistance(x, y, x4, y4));
        if (distance > extension) {
            return false;
        }
    }
    return true;
}

bool Scanner::isConvexRect(vector<Point2f> &points) {
    if (points.size() == 4) {
        Point2d point1 = points[0];
        Point2d point2 = points[1];
        Point2d point3 = points[2];
        Point2d point4 = points[3];
//        LOGD("point1: (%f, %f)", point1.x, point1.y);
//        LOGD("point2: (%f, %f)", point2.x, point2.y);
//        LOGD("point3: (%f, %f)", point3.x, point3.y);
//        LOGD("point4: (%f, %f)", point4.x, point4.y);
        double x1 = point1.x;
        double y1 = point1.y;
        double x2 = point2.x;
        double y2 = point2.y;
        double x3 = point3.x;
        double y3 = point3.y;
        double x4 = point4.x;
        double y4 = point4.y;
        double z1, z2, z3, z4;
        z1 = (x2 - x1)*(y4-y1) - (x4-x1)*(y2-y1);
        z2 = (x4 - x1)*(y3-y1) - (x3-x1)*(y4-y1);
        z3 = (x4 - x2)*(y3-y2) - (x3-x2)*(y4-y2);
        z4 = (x3 - x2)*(y1-y2) - (x1-x2)*(y3-y2);
        bool isContour = (z1*z2 > 0) && (z3 * z4 > 0);
        bool isTriangle = (isPointInLine(point1, point2, point3) || isPointInLine(point1, point2, point4)
                           || isPointInLine(point1, point3, point4) ||
                           isPointInLine(point2, point3, point4));
//        bool isContour = isContourConvex(Mat(points));
        if (isContour && !isTriangle) {
            double minLength = 50;
            if (getPointsDistanceV2(point1, point2) < minLength) {
                return false;
            }
            if (getPointsDistanceV2(point1, point3) < minLength) {
                return false;
            }
            if (getPointsDistanceV2(point1, point4) < minLength) {
                return false;
            }
            if (getPointsDistanceV2(point2, point3) < minLength) {
                return false;
            }
            if (getPointsDistanceV2(point2, point4) < minLength) {
                return false;
            }
            if (getPointsDistanceV2(point3, point4) < minLength) {
                return false;
            }
            return true;
        }

    }
    return false;
}

void Scanner::restoreLineSize(vector<Vec4i> &lines, vector<Vec4i> &output) {
    for (Vec4i line : lines) {
        float x1 = float(line[0]) * widthScale;
        float y1 = float(line[1]) * heightScale;
        float x2 = float(line[2]) * widthScale;
        float y2 = float(line[3]) * heightScale;
        Vec4i l(round(x1), round(y1), round(x2), round(y2));
        output.emplace_back(l);
    }
}

bool Scanner::isPointInLine(Point2d &point, Point2d &point1, Point2d &point2) {
    double x1 = point1.x;
    double y1 = point1.y;
    double x2 = point2.x;
    double y2 = point2.y;
    double a = y2 - y1;
    double b = x1 - x2;
    double c = x2*y1-x1*y2;
    double x = point.x;
    double y = point.y;
//    LOGD("isPointInLine: %f",  abs(a*x + b*y + c));
    return abs(a*x + b*y + c) < 0.01;
}



Mat Scanner::ImageSharp(Mat src,int nAmount)
{
    Mat dst(src.size(), CV_32FC1);
    double sigma = 3;
    // int threshold = 1;
    float amount = nAmount / 100.0f;
    Mat imgBlurred(src.size(), CV_32FC1);
    GaussianBlur(src, imgBlurred, Size(7,7), sigma, sigma,4);
    Mat temp_sub(src.size(), CV_32FC1);
    //Mat temp_abs= new Mat();
    subtract(src,imgBlurred,temp_sub);
    // Core.convertScaleAbs(temp_sub,temp_abs);
    // Mat lowContrastMask = new Mat();
    //Imgproc.threshold(temp_abs,lowContrastMask,threshold,255,1);
    //Mat temp_gen= new Mat();
    addWeighted(src,1,temp_sub,amount,0,dst);
    // dst = src+temp_sub*amount;
    //src.copyTo(dst, lowContrastMask);
    return dst;
}

Mat Scanner::ReduceBackGroundAlgorithm(Mat src,int flag) {
    Mat gauss(src.size(), CV_32FC1);
    Mat dst2(src.size(), CV_32FC1);
    Mat dst3(src.size(), CV_32FC1);
    if (flag==1) {
        GaussianBlur(src, gauss, Size(31, 31), 0, 0, 4);
    }
    else
    {
        blur(src, gauss, Size(101,101));
    }
    divide(src,gauss,dst2);
    dst2=ImageSharp(dst2, 101);
    //Imgproc.GaussianBlur(dst2, dst2, new Size(3,3), 0,0,4);//
    dst2.convertTo(dst3, CV_8UC1,255);
    return dst3;
}
