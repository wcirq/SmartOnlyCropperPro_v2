package com.auto.crop;

import android.content.Context;
import android.content.res.AssetFileDescriptor;
import android.graphics.Bitmap;
import android.graphics.Point;
import android.util.Log;
import android.widget.Toast;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.MappedByteBuffer;
import java.nio.channels.FileChannel;

import com.auto.crop.utils.FileUtils;
import com.dt.mnnlib.Model;


public class ImageDetector {

    private static final String TAG = "ImageDetector";

    public final Context context;
    private Model model;

    ImageDetector(Context context) {
        this.context = context;
    }

    public void loadConfig(String modelName) {
        Toast.makeText(this.context, "load model", Toast.LENGTH_SHORT).show();
        // 将assets/weights的模型文件复制到缓存目录，得到缓存目录weights下的权重文件绝对路径
        String modeldir = "weights";
        String cacheDir = this.context.getCacheDir() + File.separator + modeldir;
        FileUtils.copyDirectoryFromAssets(this.context, modeldir, cacheDir);
        String modelPath = cacheDir + File.separator + modelName;

        // 加载模型
        model = Model.getInstance();
        model.loadModel(modelPath);
    }

    synchronized Bitmap detectImage(Bitmap bitmap) {
        if (bitmap == null) {
            Toast.makeText(this.context, "detect image is null", Toast.LENGTH_SHORT).show();
            return null;
        }
        Bitmap outBitmap =  model.predict(bitmap);
        return outBitmap;
    }

    void close() {
        Toast.makeText(this.context, "close model", Toast.LENGTH_SHORT).show();
        model.close();
    }
}
