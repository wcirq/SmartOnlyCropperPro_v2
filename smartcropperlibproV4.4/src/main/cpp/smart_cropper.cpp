//
// Created by qiulinmin on 8/1/17.
//
#include <jni.h>
#include <string>
#include <algorithm>
#include <android_utils.h>
#include <Scanner.h>
#include <time.h>
#include <sys/time.h>
#include <android/log.h>
#define TAG "sss"
// 定义info信息
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,TAG,__VA_ARGS__)
// 定义debug信息
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, TAG, __VA_ARGS__)
// 定义error信息
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR,TAG,__VA_ARGS__)
using namespace std;
using namespace cv;

static const char* const kClassDocScanner = "com/auto/crop/SmartCropper";
//GP隐藏
static const char* RELEASE_PACKAGE = "me.pqpo.smartcropper";
static const char* RELEASE_PACKAGE2 = "";
static const char* RELEASE_PACKAGE_HW = "";
static const char* RELEASE_PACKAGE_GP = "";
static const char* RELEASE_PACKAGE_LIGHT = "";

static struct {
    jclass jClassPoint;
    jmethodID jMethodInit;
    jfieldID jFieldIDX;
    jfieldID jFieldIDY;
} gPointInfo;

static bool initSuccess = false;

static void initClassInfo(JNIEnv *env) {
    gPointInfo.jClassPoint = reinterpret_cast<jclass>(env -> NewGlobalRef(env -> FindClass("android/graphics/Point")));
    gPointInfo.jMethodInit = env -> GetMethodID(gPointInfo.jClassPoint, "<init>", "(II)V");
    gPointInfo.jFieldIDX = env -> GetFieldID(gPointInfo.jClassPoint, "x", "I");
    gPointInfo.jFieldIDY = env -> GetFieldID(gPointInfo.jClassPoint, "y", "I");
}

static jobject createJavaPoint(JNIEnv *env, const Point& point_) {
    return env -> NewObject(gPointInfo.jClassPoint, gPointInfo.jMethodInit, point_.x, point_.y);
}

static void init_image_detector(JNIEnv *env,  jclass type, jobject imageDetector) {
    jclass imageDeteClass = env->GetObjectClass(imageDetector);
    jfieldID contextId = env -> GetFieldID(imageDeteClass, "context", "Landroid/content/Context;");
    jobject context = env -> GetObjectField(imageDetector, contextId);
    jstring packageName = getPackageName(env, context);
    if (packageName == nullptr) {
        jclass je = env->FindClass("java/lang/Exception");
        env -> ThrowNew(je, "init error");
        env -> DeleteLocalRef(je);
        return;
    }
    const char* c_package = (char*)env->GetStringUTFChars(packageName, nullptr);
    if(strcmp(c_package, RELEASE_PACKAGE) != 0 ) {
        jclass je = env->FindClass("java/lang/Exception");
        env -> ThrowNew(je, "init error");
        env -> DeleteLocalRef(je);
        return;
    }

    jmethodID loadConfigMethod = env -> GetMethodID(imageDeteClass, "loadConfig", "(Ljava/lang/String;)V");
    if (loadConfigMethod == nullptr) {
        jclass je = env->FindClass("java/lang/Exception");
        env -> ThrowNew(je, "init error");
        env -> DeleteLocalRef(je);
        return;
    }
    jstring modelName = env -> NewStringUTF("model_1228.mnn");
    env -> CallVoidMethod(imageDetector, loadConfigMethod, modelName);
    env -> DeleteLocalRef(modelName);
    env -> DeleteLocalRef(imageDeteClass);
    env -> DeleteLocalRef(context);
    env -> DeleteLocalRef(packageName);
    initSuccess = true;
}

void print(JNIEnv * env, jobject clazz, String message){
    //① 获取字节码对象
    jclass claz =env->FindClass("com/auto/crop/SmartCropper");
    //② 获取Method对象
    jmethodID methodid =env->GetStaticMethodID(claz,"printString","(Ljava/lang/String;)V");
    //
    jstring result =env->NewStringUTF(message.c_str());
    env->CallStaticVoidMethod(static_cast<jclass>(clazz), methodid, result);
}

static void native_scan(JNIEnv *env, jclass type, jobject srcBitmap, jobjectArray outPoint_, jboolean canny,  jobject previewBitmap) {
    print(env, type, "enter native_scan!");
    if (!initSuccess) {
        jclass je = env->FindClass("java/lang/Exception");
        env -> ThrowNew(je, "init first");
        env -> DeleteLocalRef(je);
        return;
    }
    if (env -> GetArrayLength(outPoint_) != 4) {
        jclass je = env->FindClass("java/lang/Exception");
        env -> ThrowNew(je, "GetArrayLength(outPoint_) != 4");
        env -> DeleteLocalRef(je);
        return;
    }
    Mat srcBitmapMat;
    bitmap_to_mat(env, srcBitmap, srcBitmapMat);
    Mat bgrData(srcBitmapMat.rows, srcBitmapMat.cols, CV_8UC3);
    cvtColor(srcBitmapMat, bgrData, COLOR_RGBA2BGR);
    scanner::Scanner docScanner(bgrData, canny);
    Mat previewBitmapMat;
    if (previewBitmap != nullptr) {
        srcBitmapMat.copyTo(previewBitmapMat);
    }

    scanner::RectangularInfo rectangularInfo = docScanner.scanPoint2(previewBitmapMat);
    std::vector<Point> scanPoints = rectangularInfo.vertex;

    for (int i = 0; i < scanPoints.size(); ++i) {
        print(env, type, "Point" + to_string(i) + " " + to_string(scanPoints[i].x) + " " + to_string(scanPoints[i].y));
    }

    if (previewBitmap != nullptr && !previewBitmapMat.empty()) {
        mat_to_bitmap(env, previewBitmapMat, previewBitmap);
    }
    if (scanPoints.size() == 4) {
        for (int i = 0; i < 4; ++i) {
            env -> SetObjectArrayElement(outPoint_, i, createJavaPoint(env, scanPoints[i]));
        }
    }
}

//static jobject native_scan2(JNIEnv *env, jclass type, jobject srcBitmap, jobjectArray outPoint_, jboolean canny,  jobject previewBitmap) {
extern "C"
JNIEXPORT jobject JNICALL
Java_com_auto_crop_SmartCropper_nativeScan2(JNIEnv *env, jclass type, jobject srcBitmap,
                                            jobjectArray outPoint_, jboolean canny,
                                            jobject previewBitmap) {
    print(env, type, "enter native_scan!");
    if (!initSuccess) {
        jclass je = env->FindClass("java/lang/Exception");
        env -> ThrowNew(je, "init first");
        env -> DeleteLocalRef(je);

        //ArrayList Object
        jclass cls_ArrayList = env->FindClass("java/util/ArrayList");
        jmethodID construct = env->GetMethodID(cls_ArrayList,"<init>","()V");
        jobject obj_ArrayList = env->NewObject(cls_ArrayList,construct);
        return obj_ArrayList;
    }
    if (env -> GetArrayLength(outPoint_) != 4) {
        jclass je = env->FindClass("java/lang/Exception");
        env -> ThrowNew(je, "GetArrayLength(outPoint_) != 4");
        env -> DeleteLocalRef(je);

        //ArrayList Object
        jclass cls_ArrayList = env->FindClass("java/util/ArrayList");
        jmethodID construct = env->GetMethodID(cls_ArrayList,"<init>","()V");
        jobject obj_ArrayList = env->NewObject(cls_ArrayList,construct);
        return obj_ArrayList;
    }
    Mat srcBitmapMat;
    bitmap_to_mat(env, srcBitmap, srcBitmapMat);
    Mat bgrData(srcBitmapMat.rows, srcBitmapMat.cols, CV_8UC3);
    cvtColor(srcBitmapMat, bgrData, COLOR_RGBA2BGR);
    scanner::Scanner docScanner(bgrData, canny);
    Mat previewBitmapMat;
    if (previewBitmap != nullptr) {
        srcBitmapMat.copyTo(previewBitmapMat);
    }
    scanner::RectangularInfo rectangularInfo = docScanner.scanPoint2(previewBitmapMat);
    std::vector<Point> scanPoints = rectangularInfo.vertex;
    std::vector<Point> edges = rectangularInfo.edges;
//    for (int i = 0; i < scanPoints.size(); ++i) {
//        print(env, type, "scanPoints[" + to_string(i) + "]: " + to_string(scanPoints[i].x) + " " + to_string(scanPoints[i].y));
//    }

    if (previewBitmap != nullptr && !previewBitmapMat.empty()) {
        mat_to_bitmap(env, previewBitmapMat, previewBitmap);
    }

    if (scanPoints.size() == 4) {
        for (int i = 0; i < 4; ++i) {
            env -> SetObjectArrayElement(outPoint_, i, createJavaPoint(env, scanPoints[i]));
        }
    }

    /* 构建返回结果*/
    jclass cls_ArrayList = env->FindClass("java/util/ArrayList");
    jmethodID construct = env->GetMethodID(cls_ArrayList,"<init>","()V");
    jobject obj_ArrayList = env->NewObject(cls_ArrayList,construct);
    jmethodID arrayList_add = env->GetMethodID(cls_ArrayList,"add","(Ljava/lang/Object;)Z");

    for(int i=0;i<edges.size();i++){
        jobject obj_point = createJavaPoint(env, edges[i]);
        env->CallBooleanMethod(obj_ArrayList,arrayList_add,obj_point);
    }
    return obj_ArrayList;
}

static vector<Point> pointsToNative(JNIEnv *env, jobjectArray points_) {
    int arrayLength = env->GetArrayLength(points_);
    vector<Point> result;
    for(int i = 0; i < arrayLength; i++) {
        jobject point_ = env -> GetObjectArrayElement(points_, i);
        int pX = env -> GetIntField(point_, gPointInfo.jFieldIDX);
        int pY = env -> GetIntField(point_, gPointInfo.jFieldIDY);
        result.emplace_back(pX, pY);
    }
    return result;
}

static vector<Point> arrayListToNative(JNIEnv *env, jobject edges_) {
    vector<Point> edges;
    jclass cls_arraylist = env->GetObjectClass(edges_);
    jmethodID arraylist_get = env->GetMethodID(cls_arraylist,"get","(I)Ljava/lang/Object;");
    jmethodID arraylist_size = env->GetMethodID(cls_arraylist,"size","()I");
    jint len = env->CallIntMethod(edges_,arraylist_size);
    for(int i=0;i<len;i++){
        jobject obj_edges = env->CallObjectMethod(edges_,arraylist_get,i);
        jclass cls_edges = env->GetObjectClass(obj_edges);
        jfieldID xFieldID = env->GetFieldID(cls_edges, "x", "I");
        jint x = env->GetIntField(obj_edges, xFieldID);
        jfieldID yFieldID = env->GetFieldID(cls_edges, "y", "I");
        jint y = env->GetIntField(obj_edges, yFieldID);
        edges.push_back(Point(x, y));
    }
    return edges;
}

static void native_crop(JNIEnv *env, jclass type, jobject srcBitmap, jobjectArray points_, jobject outBitmap) {
    std::vector<Point> points = pointsToNative(env, points_);
    if (points.size() != 4) {
        return;
    }
    Point leftTop = points[0];
    Point rightTop = points[1];
    Point rightBottom = points[2];
    Point leftBottom = points[3];

    Mat srcBitmapMat;
    bitmap_to_mat(env, srcBitmap, srcBitmapMat);

    AndroidBitmapInfo outBitmapInfo;
    AndroidBitmap_getInfo(env, outBitmap, &outBitmapInfo);
    Mat dstBitmapMat;
    int newHeight = outBitmapInfo.height;
    int newWidth = outBitmapInfo.width;
    dstBitmapMat = Mat::zeros(newHeight, newWidth, srcBitmapMat.type());

    std::vector<Point2f> srcTriangle;
    std::vector<Point2f> dstTriangle;

    srcTriangle.emplace_back(leftTop.x, leftTop.y);
    srcTriangle.emplace_back(rightTop.x, rightTop.y);
    srcTriangle.emplace_back(leftBottom.x, leftBottom.y);
    srcTriangle.emplace_back(rightBottom.x, rightBottom.y);

    dstTriangle.emplace_back(0, 0);
    dstTriangle.emplace_back(newWidth, 0);
    dstTriangle.emplace_back(0, newHeight);
    dstTriangle.emplace_back(newWidth, newHeight);

    Mat transform = getPerspectiveTransform(srcTriangle, dstTriangle);
    warpPerspective(srcBitmapMat, dstBitmapMat, transform, dstBitmapMat.size());

    mat_to_bitmap(env, dstBitmapMat, outBitmap);
}

static void native_sign(JNIEnv *env, jclass type, jobject srcBitmap, jobjectArray points_, jobject outBitmap) {

}


//
static void native_shengmo(JNIEnv *env, jclass type, jobject srcBitmap, jobject outBitmap) {

}


static void native_heibai(JNIEnv *env, jclass type, jobject srcBitmap, jobject outBitmap) {

}

static void native_zengqiang(JNIEnv *env, jclass type, jobject srcBitmap, jobject outBitmap) {

}

static void native_zengqiang_v2(JNIEnv *env, jclass type, jobject srcBitmap, jobject outBitmap) {

}
static void native_zengqiang_v2_color(JNIEnv *env, jclass type, jobject srcBitmap, jobject outBitmap) {

}
static void native_zengliang(JNIEnv *env, jclass type, jobject srcBitmap, jobject outBitmap) {

}

//优化速度后的版本
static void native_tiaoseban(JNIEnv *env, jclass type, jobject srcBitmap, jobject outBitmap, jfloat a1, jfloat a2, jfloat a3) {
    Mat srcBitmapMat;
    bitmap_to_mat(env, srcBitmap, srcBitmapMat);

    Mat dstBitmapMat;
    cvtColor(srcBitmapMat, dstBitmapMat, COLOR_BGRA2BGR);

    // 调整亮度和对比度
    Mat img;
    dstBitmapMat.convertTo(img, -1, a1, a2);

    // 调整饱和度
    Mat dst = Mat::zeros(img.size(), img.type());
    addWeighted(img, 1 + a3, dst, 0, 0, dst);

    mat_to_bitmap(env, dst, outBitmap);
}



static void native_huidu(JNIEnv *env, jclass type, jobject srcBitmap, jobject outBitmap) {

}

static JNINativeMethod gMethods[] = {

        {
                "initImageDetector",
                "(Lcom/auto/crop/ImageDetector;)V",
                (void*)init_image_detector
        },

        {
                "nativeScan",
                "(Landroid/graphics/Bitmap;[Landroid/graphics/Point;ZLandroid/graphics/Bitmap;)V",
                (void*)native_scan
        },

//        {
//                "nativeScan2",
//                "(Landroid/graphics/Bitmap;[Landroid/graphics/Point;[Landroid/graphics/Point;ZLandroid/graphics/Bitmap;)jobject",
//                (void*)native_scan2
//        },

        {
                "nativeCrop",
                "(Landroid/graphics/Bitmap;[Landroid/graphics/Point;Landroid/graphics/Bitmap;)V",
                (void*)native_crop
        }

};

extern "C"
JNIEXPORT jint JNICALL
JNI_OnLoad(JavaVM* vm, void* reserved) {
    JNIEnv *env = NULL;
    if (vm->GetEnv((void **) &env, JNI_VERSION_1_4) != JNI_OK) {
        return JNI_FALSE;
    }
    jclass classDocScanner = env->FindClass(kClassDocScanner);
    if(env -> RegisterNatives(classDocScanner, gMethods, sizeof(gMethods)/ sizeof(gMethods[0])) < 0) {
        return JNI_FALSE;
    }
    initClassInfo(env);
    return JNI_VERSION_1_4;


}

std::vector<std::vector<cv::Point2f>> allocationEdge(std::vector<cv::Point> points, std::vector< cv::Point> contour)
{
    std::vector<int> vertexIndex;

    // 从所有边缘坐标中找到最接近矩形4个顶点的点
    for (size_t i = 0; i < points.size(); i++)
    {
        double minDistance = 999999;
        int minIndex = -1;
        for (size_t j = 0; j < contour.size(); j++)
        {
            double distance = sqrt(pow(points[i].x - contour[j].x, 2) + pow(points[i].y - contour[j].y, 2));
            if (minDistance>distance)
            {
                minDistance = distance;
                minIndex = j;
            }
        }
        vertexIndex.push_back(minIndex);
    }


    // 通过4顶点索引顺序判断边缘点的排序是顺时针还是逆时针
    bool isInverse = [](std::vector<int> vertexIndex) -> bool {
        int total = 0;
        for (size_t i = 0; i < vertexIndex.size()-1; i++)
        {
            total += vertexIndex[i] > vertexIndex[i + 1]?1:-1;
        }
        return total > 0 ? true : false;  // 大于0是逆时针
    }(vertexIndex);

    if (isInverse)
    {
        // 边缘点的排序是逆时针，而4顶点是顺时针,将4顶点变为逆时针
        std::sort(vertexIndex.begin(), vertexIndex.end());
    }

    std::vector< cv::Point> contour2(contour);
    contour2.insert(contour2.end(), contour.begin(), contour.begin()+ vertexIndex[0]);
    std::vector<std::vector<cv::Point2f>> edges;

    std::vector<cv::Point2f> part;
    std::vector<cv::Point>::const_iterator first;
    std::vector<cv::Point>::const_iterator last;

    for (int i = vertexIndex.size()-1; i >= 0; i--)
    {
        if (i== vertexIndex.size() - 1)
        {
            // 插入第一个顶点到第二个顶点的边缘
            first = contour2.begin() + vertexIndex[3];
            last = contour2.end();
        }
        else
        {
            //  插入第i个顶点到第i+1个顶点的边缘
            first = contour2.begin() + vertexIndex[i];
            last = contour2.begin() + vertexIndex[i+1];
        }
        part.insert(part.begin(), first, last);
        edges.push_back(part);
        part.clear();
    }
    std::reverse(edges[0].begin(), edges[0].end()); // 边缘点是逆序的，所以需要将第一个顶点到第二个顶点的边缘逆序
    std::reverse(edges[1].begin(), edges[1].end()); // 边缘点是逆序的，所以需要将第二个顶点到第三个顶点的边缘逆序



    return edges;
}

cv::Mat warpArbitrary(const cv::Mat& src, std::vector<cv::Point> points, std::vector<std::vector<cv::Point2f>> edges, cv::Size newSize, int flags = INTER_LINEAR)
{
    // 最近邻算法
    static auto nearestNeighbor = [](cv::Mat src, cv::Mat dst, cv::Point2f pos0, cv::Point2i pos1) {
        cv::Point2i vertex = cv::Point2i(floor(pos0.x+0.5), floor(pos0.y + 0.5)); // 左上角

        // 判断点是否超出图像尺寸
        if (vertex.x < 0) vertex.x = 0;
        if (vertex.y < 0) vertex.y = 0;
        if (vertex.x > src.cols-1) vertex.x = src.cols - 1;
        if (vertex.y > src.rows - 1) vertex.y = src.rows - 1;


        int channels = src.channels();
        if (1 == channels)
        {
            dst.at<uchar>(pos1.y, pos1.x) = src.at<uchar>(vertex.y, vertex.x);
        }
        else if (3 == channels)
        {
            dst.at<cv::Vec3b>(pos1.y, pos1.x) = src.at<cv::Vec3b>(vertex.y, vertex.x);
        }
        else if (4 == channels)
        {
            dst.at<cv::Vec4b>(pos1.y, pos1.x) = src.at<cv::Vec4b>(vertex.y, vertex.x);
        }
    };

    // 双线性插值算法
    static auto bilinearInterpolation = [](cv::Mat src, cv::Mat dst, cv::Point2f pos0, cv::Point2i pos1) {
        cv::Point2i vertex0 = cv::Point2i(floor(pos0.x), floor(pos0.y)); // 左上角
        cv::Point2i vertex1 = cv::Point2i(vertex0.x + 1, vertex0.y); // 右上角
        cv::Point2i vertex2 = cv::Point2i(vertex0.x, vertex0.y + 1); // 左下角
        cv::Point2i vertex3 = cv::Point2i(vertex0.x + 1, vertex0.y + 1); // 右下角

        float prop_x = pos0.x - vertex0.x;
        float prop_y = pos0.y - vertex0.y;

        // 判断点是否超出图像尺寸
        for (cv::Point2i vertex : {vertex0, vertex1, vertex2, vertex3})
        {
            if (vertex.x < 0) vertex.x = 0;
            if (vertex.y < 0) vertex.y = 0;
            if (vertex.x > src.cols - 1) vertex.x = src.cols - 1;
            if (vertex.y > src.rows - 1) vertex.y = src.rows - 1;
        }

        int channels = src.channels();
        if (1 == channels)
        {
            float rgb0 = src.at<uchar>(vertex0.y, vertex0.x); // 左上角点的值
            float rgb1 = src.at<uchar>(vertex1.y, vertex1.x); // 右上角点的值
            float rgb2 = src.at<uchar>(vertex2.y, vertex2.x); // 左下角点的值
            float rgb3 = src.at<uchar>(vertex3.y, vertex3.x); // 右下角点的值

            float temp01 = rgb0 * (1 - prop_x) + rgb1 * prop_x; // 左上角到右上角的插值
            float temp02 = rgb2 * (1 - prop_x) + rgb3 * prop_x; // 左下角到右下角的插值

            float temp = temp01 * (1 - prop_y) + temp02 * prop_y;  // 双线性插值结果

            dst.at<uchar>(pos1.y, pos1.x) = temp;
        }
        else if (3 == channels)
        {
            cv::Vec3f rgb0 = src.at<cv::Vec3b>(vertex0.y, vertex0.x); // 左上角点的值
            cv::Vec3f rgb1 = src.at<cv::Vec3b>(vertex1.y, vertex1.x); // 右上角点的值
            cv::Vec3f rgb2 = src.at<cv::Vec3b>(vertex2.y, vertex2.x); // 左下角点的值
            cv::Vec3f rgb3 = src.at<cv::Vec3b>(vertex3.y, vertex3.x); // 右下角点的值

            cv::Vec3f temp01 = rgb0 * (1 - prop_x) + rgb1 * prop_x; // 左上角到右上角的插值
            cv::Vec3f temp02 = rgb2 * (1 - prop_x) + rgb3 * prop_x; // 左下角到右下角的插值

            cv::Vec3f temp = temp01 * (1 - prop_y) + temp02 * prop_y;  // 双线性插值结果

            dst.at<cv::Vec3b>(pos1.y, pos1.x) = temp;
        }
        else if (4 == channels)
        {
            cv::Vec4f rgb0 = src.at<cv::Vec4b>(vertex0.y, vertex0.x); // 左上角点的值
            cv::Vec4f rgb1 = src.at<cv::Vec4b>(vertex1.y, vertex1.x); // 右上角点的值
            cv::Vec4f rgb2 = src.at<cv::Vec4b>(vertex2.y, vertex2.x); // 左下角点的值
            cv::Vec4f rgb3 = src.at<cv::Vec4b>(vertex3.y, vertex3.x); // 右下角点的值

            cv::Vec4f temp01 = rgb0 * (1 - prop_x) + rgb1 * prop_x; // 左上角到右上角的插值
            cv::Vec4f temp02 = rgb2 * (1 - prop_x) + rgb3 * prop_x; // 左下角到右下角的插值

            cv::Vec4f temp = temp01 * (1 - prop_y) + temp02 * prop_y;  // 双线性插值结果

            dst.at<cv::Vec4b>(pos1.y, pos1.x) = temp;
        }
    };

    // 获取纠正后图像点在原始图像上的位置
    static auto getOriginalPosition = [&](int row, int col) -> cv::Point2f {
        double propVer = (double)row / newSize.height;
        double propHor = (double)col / newSize.width;
        int indexStartVer = propVer * edges[3].size();
        int indexEndVer = propVer * edges[1].size();
        double start_x = edges[3][indexStartVer].x;
        double end_x = edges[1][indexEndVer].x;
        double x = start_x + (end_x - start_x) * propHor;

        int indexStartHor = propHor * edges[0].size();
        int indexEndHor = propHor * edges[2].size();
        double start_y = edges[0][indexStartHor].y;
        double end_y = edges[2][indexEndHor].y;
        double y = start_y + (end_y - start_y) * propVer;

        return cv::Point2f(x, y);
    };

    cv::Mat dst = cv::Mat::ones(newSize, src.type())*255;
    for (size_t i = 0; i < newSize.height; i++)
    {
        for (size_t j = 0; j < newSize.width; j++)
        {
            cv::Point2f pos1 = getOriginalPosition(i, j);  // 获取纠正后图像点在原始图像上的位置
            cv::Point2i pos2(j, i);  // 纠正后图像点位置
            if (INTER_LINEAR==flags)
            {
                // 双线性插值
                bilinearInterpolation(src, dst, pos1, pos2);
            }
            else
            {
                // 最近邻插值
                nearestNeighbor(src, dst, pos1, pos2);
            }
        }
    }
    return dst;
}

void getLinearPoints(cv::Point point1, cv::Point point2, float interval, std::vector<cv::Point>& points, JNIEnv *env, jclass type)
{
    print(env, type, "*1");
    if (interval<=0)
    {
        float distance = [](Point p1, Point p2)->float {
            return sqrt(pow(p1.x-p2.x, 2)+ pow(p1.y-p2.y, 2));
        }(point1, point2);
        interval = 1.f/distance;
    }

    auto getDirection = [&point1, &point2](float* Direction)
    {

        Direction[0] = point2.x - point1.x;
        Direction[1] = point2.y - point1.y;
    };

    auto getLength = [](float* Matrix)
    {
        float x = pow(Matrix[0], 2);
        float y = pow(Matrix[1], 2);

        return sqrt(x + y);
    };

    auto getNormVector = [](float* Matrix)
    {
        float x = pow(Matrix[0], 2);
        float y = pow(Matrix[1], 2);

        Matrix[0] = Matrix[0] / sqrt(x + y);
        Matrix[1] = Matrix[1] / sqrt(x + y);
    };

    static auto getEquationParameters = [&getDirection, &getLength, &getNormVector, &env, &type](float& dx, float& dy, float& t, int& m_NumPoints)
    {
        print(env, type, "**0 ");
        float Direction[2];
        getDirection(Direction); ///得到两点的方向向量
        print(env, type, "**1 ");
        float L_D = getLength(Direction); //得到方向向量的模长
        print(env, type, "**2 ");
        float std_D[2];
        memcpy(std_D, Direction, sizeof(Direction));
        getNormVector(std_D);  //得到单位向量
        print(env, type, "**3 ");
        dx = std_D[0];
        dy = std_D[1];

        float L_std_D = getLength(std_D);
        print(env, type, "**4 ");
        t = L_D / L_std_D;  //t = |d| / √d;
        print(env, type, "**5 ");
        m_NumPoints = int(L_D);
        print(env, type, "**6 ");
    };
    print(env, type, "*2 " + to_string(interval));
    float dx, dy, t;
    int m_NumPoints;
    getEquationParameters(dx, dy, t, m_NumPoints);
    print(env, type, "*3 " + to_string(interval) + " " + to_string(m_NumPoints));

    for (int i = 0; i < int(m_NumPoints / interval); i++) {
        float x = point1.x + dx * t * interval * i;
        float y = point1.y + dy * t * interval * i;

        float dis = sqrt(pow(point2.x - x, 2) + pow(point2.y - y, 2));
        if (dis < 1.0)
            break;

        points.push_back(cv::Point(x, y));
    }
    print(env, type, "*4");
}

extern "C"
JNIEXPORT void JNICALL
Java_com_auto_crop_SmartCropper_nativeCrop2(JNIEnv *env, jclass type, jobject srcBitmap,
                                            jobjectArray srcPoints_, jobjectArray points_, jobject edges_,
                                            jobject outBitmap) {

    auto getEuclideanDistance = [](Point p1, Point p2)->float {
        return sqrt(pow(p1.x-p2.x, 2)+ pow(p1.y-p2.y, 2));
    };

    std::vector<Point> srcPoints = pointsToNative(env, srcPoints_);
    std::vector<Point> points = pointsToNative(env, points_);
    std::vector<Point> edges = arrayListToNative(env, edges_);

    if (points.size() != 4) {
        return;
    }

    Mat srcBitmapMat;
    bitmap_to_mat(env, srcBitmap, srcBitmapMat);
    int srcWidth = srcBitmapMat.cols;
    int srcHeight = srcBitmapMat.rows;
//    if (edges.empty()) // 如果详细边缘为空，则之间返回原图
//    {
//        mat_to_bitmap(env, srcBitmapMat, outBitmap);
//        return;
//    }

    AndroidBitmapInfo outBitmapInfo;
    AndroidBitmap_getInfo(env, outBitmap, &outBitmapInfo);
    int newHeight = outBitmapInfo.height;
    int newWidth = outBitmapInfo.width;
    cv::Size newSize(newWidth, newHeight);


//    print(env, type, "debug: 00");
    if (edges.empty())
    {
//        print(env, type, "edges.empty())");
        // 直接将图像边缘进行分配
        Point leftTop = points[0];
        Point rightTop = points[1];
        Point rightBottom = points[2];
        Point leftBottom = points[3];

        auto alloc = [&getEuclideanDistance,&srcWidth,&srcHeight, &env, &type](Point p1, Point p2) -> std::vector<cv::Point>{
            std::vector<cv::Point> edge;

            float dist = getEuclideanDistance(p1, p2);
            getLinearPoints(p1, p2,1.f/dist, edge, env, type);
            for (int i = 0; i < edge.size(); ++i) {
                float x = edge[i].x;
                float y = edge[i].y;
                if (x<0) x=0;
                if (y<0) y=0;
                if (x>srcWidth-1.f) x=srcWidth-1.f;
                if (y>srcHeight-1.f) y=srcHeight-1.f;
                edge[i].x = x;
                edge[i].y = y;
            }
            return edge;
        };
        std::vector<cv::Point> temp = alloc(leftTop, leftBottom);
        edges.insert(edges.end(), temp.begin(), temp.end());

        temp = alloc(leftBottom, rightBottom);
        edges.insert(edges.end(), temp.begin(), temp.end());

        temp = alloc(rightBottom, rightTop);
        edges.insert(edges.end(), temp.begin(), temp.end());

        temp = alloc(rightTop, leftTop);
        edges.insert(edges.end(), temp.begin(), temp.end());
    }
//    print(env, type, "debug: 01");
    // 根据模型给的4个点分配边
    std::vector<std::vector<cv::Point2f>> edges4sided = allocationEdge(srcPoints, edges);

//    print(env, type, "edges size: " +  std::to_string(edges.size()));
//    print(env, type, "edges4sided[0]: " +  std::to_string(edges4sided[0].size()));
//    print(env, type, "edges4sided[1]: " +  std::to_string(edges4sided[1].size()));
//    print(env, type, "edges4sided[2]: " +  std::to_string(edges4sided[2].size()));
//    print(env, type, "edges4sided[3]: " +  std::to_string(edges4sided[3].size()));
//
//    for (int i = 0; i < edges4sided.size(); ++i) {
//        print(env, type, "edges4sided[0]: " +  std::to_string(edges4sided[i][0].x) + " " + std::to_string(edges4sided[i][0].y));
//        print(env, type, "edges4sided[0]: " +  std::to_string(edges4sided[i][edges4sided[i].size()-1].x) + " " + std::to_string(edges4sided[i][edges4sided[i].size()-1].y));
//    }

//    if (!edges.empty())
//    {
//        for (int j = 0; j < edges.size()-1; ++j) {
////            print(env, type, "debug: 0 " + to_string(j) + " " + to_string(edges.size()-1));
//            cv::line(srcBitmapMat, edges[j], edges[j+1], cv::Scalar((float)j/( edges.size()-1)*255,(float)j/( edges.size()-1)*255, (float)j/( edges.size()-1)*255), 100);
//        }
//    }

//    print(env, type, "debug: 1");
    float thresh = 2;
    // 比较调整后的4个点坐标与模型给出的4点坐标是否一致
    for (int i = 0; i < edges4sided.size(); ++i) {
//        print(env, type, "debug: 10");
        float dist1 = getEuclideanDistance(points[i%4], srcPoints[i%4]);
        float dist2 = getEuclideanDistance(points[(i+1)%4], srcPoints[(i+1)%4]);
        if (dist1<thresh&&dist2<thresh){
            // 小于阈值像素认为没有改变
            continue;
        }
//        print(env, type, "debug: 11");
        std::vector<cv::Point> pointSet;
        if(i%4<2) {
            print(env, type, "debug: 110 " + to_string(points[i%4].x) + " " + to_string(points[i%4].y));
            print(env, type, "debug: 110 " + to_string(points[(i+1)%4].x) + " " + to_string(points[(i+1)%4].y));
            // 由于第一和第二条边的点是顺时针
            getLinearPoints(points[i % 4], points[(i + 1) % 4], -1, pointSet, env, type);
            print(env, type, "debug: 110 ");
        }
        else
        {
            print(env, type, "debug: 111 " + to_string(points[i%4].x) + " " + to_string(points[i%4].y));
            print(env, type, "debug: 111 " + to_string(points[(i+1)%4].x) + " " + to_string(points[(i+1)%4].y));
            getLinearPoints(points[(i+1)%4], points[i%4],-1, pointSet, env, type);
            print(env, type, "debug: 110 ");
        }
        print(env, type, "debug: 12");
        edges4sided[i].clear();  // 清除原来的
        for (int j = 0; j < pointSet.size(); ++j) {
            edges4sided[i].push_back(cv::Point2f(pointSet[j].x, pointSet[j].y));
        }
    }
    print(env, type, "debug: 2");
    std::vector<std::vector<cv::Point2f>> edgesTiny(edges4sided);
    cv::RNG rng;
    for (size_t i = 0; i < edges4sided.size(); i++)
    {
//        // 绘制线条
//        std::vector<cv::Point> pts(edges4sided[i].size());
//        for (int j = 0; j < edges4sided[i].size()-1; ++j) {
//            cv::line(srcBitmapMat, edges4sided[i][j], edges4sided[i][j+1], cv::Scalar(0,0, (float)j/( edges4sided[i].size()-1)*255), 60);
//        }

        std::vector<cv::Point2f> temp;
        double dist = cv::arcLength(edges4sided[i], false);
        double epsilon = 0.02 * dist;
        cv::approxPolyDP(edges4sided[i], temp, epsilon, false);
        edgesTiny[i].clear();
        for (size_t j = 0; j < temp.size()-1; j++)
        {
            std::vector<cv::Point> edge;
            getLinearPoints(temp[j], temp[j + 1], -1, edge, env, type);

            edgesTiny[i].insert(edgesTiny[i].end(), edge.begin(), edge.end());
        }

//        // 绘制线条
//        for (int j = 0; j < edgesTiny[i].size()-1; ++j) {
//            cv::line(srcBitmapMat, edgesTiny[i][j], edgesTiny[i][j+1], cv::Scalar((float)j/( edgesTiny[i].size()-1)*255,0, 0), 20);
//        }
    }
//    print(env, type, "debug: 3");
//    for (int i = 0; i < points.size(); ++i) {
//        cv::circle(srcBitmapMat, points[i], 30, cv::Scalar(0, float(i)/points.size()*255, 0), -1);
//    }
//    for (int i = 0; i < srcPoints.size(); ++i) {
//        cv::circle(srcBitmapMat, srcPoints[i], 15, cv::Scalar(float(i)/srcPoints.size()*255, 0, float(i)/srcPoints.size()*255), -1);
//    }
//    print(env, type, "debug: 4");
//    cv::resize(srcBitmapMat, srcBitmapMat, newSize);
    cv::Mat dstBitmapMat = warpArbitrary(srcBitmapMat, points, edgesTiny, newSize, INTER_LINEAR);

    mat_to_bitmap(env, dstBitmapMat, outBitmap);
}