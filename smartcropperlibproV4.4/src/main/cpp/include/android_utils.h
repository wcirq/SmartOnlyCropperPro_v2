//
// Created by qiulinmin on 17-5-15.
//

#ifndef IMG_ANDROID_UTILS_H
#define IMG_ANDROID_UTILS_H

#include <android/bitmap.h>
#include <opencv2/opencv.hpp>

using namespace cv;

void bitmap_to_mat(JNIEnv *env, jobject &srcBitmap, Mat &srcMat);

void mat_to_bitmap(JNIEnv *env, Mat &srcMat, jobject &dstBitmap);

jstring getPackageName(JNIEnv *env, jobject context);

#endif //IMG_ANDROID_UTILS_H
