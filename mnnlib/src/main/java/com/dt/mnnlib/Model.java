package com.dt.mnnlib;

import android.content.res.AssetFileDescriptor;
import android.graphics.Bitmap;
import android.content.Context;

import java.io.IOException;

public class Model {
    
    private static Model instance = null;

    private Model()  {

    }

    public static Model getInstance() {
        if (instance == null) {
            synchronized (Model.class) {
                if (instance == null) {
                    instance = new Model();
                }
            }
        }
        return instance;
    }

    /**
     * 加载模型
     * @param modelPath 模型路径
     */
    public void loadModel(String modelPath) {
        init(modelPath);
    }

    /**
     * 模型推理
     * @param image 图片
     */
    public Bitmap predict(Bitmap image){
        Bitmap outImage = Bitmap.createBitmap(image.getWidth() , image.getHeight(), Bitmap.Config.ARGB_8888);
        detect(image, outImage);
        return outImage;
    }

    // 释放资源
    public void close(){
        release();
    }

    private static native void init(String modelPath);
    private static native void detect(Bitmap image, Bitmap outImage);
    private static native void release();

    static {
        System.loadLibrary("mnnlib");
    }

}
