//
// Created by wcirq on 2023/2/28.
//

#include "model.h"
#include <jni.h>

Model *model = nullptr;

void bitmap_to_mat(JNIEnv *env, jobject &srcBitmap, cv::Mat &srcMat) {
    void *srcPixels = 0;
    AndroidBitmapInfo srcBitmapInfo;
    try {
        AndroidBitmap_getInfo(env, srcBitmap, &srcBitmapInfo);
        AndroidBitmap_lockPixels(env, srcBitmap, &srcPixels);
        uint32_t srcHeight = srcBitmapInfo.height;
        uint32_t srcWidth = srcBitmapInfo.width;
        srcMat.create(srcHeight, srcWidth, CV_8UC4);
        if (srcBitmapInfo.format == ANDROID_BITMAP_FORMAT_RGBA_8888) {
            cv::Mat tmp(srcHeight, srcWidth, CV_8UC4, srcPixels);
            tmp.copyTo(srcMat);
        } else {
            cv::Mat tmp = cv::Mat(srcHeight, srcWidth, CV_8UC2, srcPixels);
            cvtColor(tmp, srcMat, cv::COLOR_BGR5652RGBA);
        }
        AndroidBitmap_unlockPixels(env, srcBitmap);
        return;
    } catch (cv::Exception &e) {
        AndroidBitmap_unlockPixels(env, srcBitmap);
        jclass je = env->FindClass("java/lang/Exception");
        env -> ThrowNew(je, e.what());
        return;
    } catch (...) {
        AndroidBitmap_unlockPixels(env, srcBitmap);
        jclass je = env->FindClass("java/lang/Exception");
        env -> ThrowNew(je, "unknown");
        return;
    }
}

void mat_to_bitmap(JNIEnv *env, cv::Mat &srcMat, jobject &dstBitmap) {
    void *dstPixels = 0;
    AndroidBitmapInfo dstBitmapInfo;
    try {
        AndroidBitmap_getInfo(env, dstBitmap, &dstBitmapInfo);
        AndroidBitmap_lockPixels(env, dstBitmap, &dstPixels);
        uint32_t dstHeight = dstBitmapInfo.height;
        uint32_t dstWidth = dstBitmapInfo.width;
        if (dstBitmapInfo.format == ANDROID_BITMAP_FORMAT_RGBA_8888) {
            cv::Mat tmp(dstHeight, dstWidth, CV_8UC4, dstPixels);
            if(srcMat.type() == CV_8UC1) {
                cvtColor(srcMat, tmp, cv::COLOR_GRAY2RGBA);
            } else if (srcMat.type() == CV_8UC3) {
                cvtColor(srcMat, tmp, cv::COLOR_RGB2RGBA);
            } else if (srcMat.type() == CV_8UC4) {
                srcMat.copyTo(tmp);
            }
        } else {
            cv::Mat tmp = cv::Mat(dstHeight, dstWidth, CV_8UC2, dstPixels);
            if(srcMat.type() == CV_8UC1) {
                cvtColor(srcMat, tmp, cv::COLOR_GRAY2BGR565);
            } else if (srcMat.type() == CV_8UC3) {
                cvtColor(srcMat, tmp, cv::COLOR_RGB2BGR565);
            } else if (srcMat.type() == CV_8UC4) {
                cvtColor(srcMat, tmp, cv::COLOR_RGBA2BGR565);
            }
        }
        AndroidBitmap_unlockPixels(env, dstBitmap);
    }catch (cv::Exception &e) {
        AndroidBitmap_unlockPixels(env, dstBitmap);
        jclass je = env->FindClass("java/lang/Exception");
        env -> ThrowNew(je, e.what());
        return;
    } catch (...) {
        AndroidBitmap_unlockPixels(env, dstBitmap);
        jclass je = env->FindClass("java/lang/Exception");
        env -> ThrowNew(je, "unknown");
        return;
    }
}

extern "C"
JNIEXPORT void JNICALL
Java_com_dt_mnnlib_Model_detect(JNIEnv *env, jclass clazz, jobject image, jobject out_image) {
    // 模型推理
    cv::Mat srcBitmapMat;
    bitmap_to_mat(env, image, srcBitmapMat);
    cv::Mat result = model->detect(srcBitmapMat);
    mat_to_bitmap(env, result, out_image);
}
extern "C"
JNIEXPORT void JNICALL
Java_com_dt_mnnlib_Model_init(JNIEnv *env, jclass clazz, jstring model_path) {
    // 模型加载
    const char* p_model_path = env->GetStringUTFChars(model_path, NULL);
    model = new Model(p_model_path);
}
extern "C"
JNIEXPORT void JNICALL
Java_com_dt_mnnlib_Model_release(JNIEnv *env, jclass clazz) {
    // 模型资源释放
    delete model;
}