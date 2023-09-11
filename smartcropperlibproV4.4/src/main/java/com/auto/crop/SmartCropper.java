package com.auto.crop;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Point;
import android.graphics.Rect;
import android.util.Log;

import com.auto.crop.utils.CropUtils;

import java.io.IOException;
import java.util.ArrayList;

/**
 * Created by qiulinmin on 8/1/17.
 */

public class SmartCropper {

    private static ImageDetector sImageDetector = null;
    public static int QIE_TU_LIMIT = 6000 * 4000;
    private static Context context = null;

    public static class RectangularInfo{
        Point vertex[];
        ArrayList<Point> edges;

        public ArrayList<Point> getEdges() {
            return edges;
        }

        public void setEdges(ArrayList<Point> edges) {
            this.edges = edges;
        }

        RectangularInfo(Point[] vertex, ArrayList<Point> edges)
        {
            this.vertex = vertex;
            this.edges = edges;
        }

        public Point[] getVertex() {
            return vertex;
        }

        public void setVertex(Point[] vertex) {
            this.vertex = vertex;
        }
    }

    public static void init(Context context) {
        SmartCropper.context = context;
        sImageDetector = new ImageDetector(context);
        initImageDetector(sImageDetector);
    }

    public static void releaseImageDetector() {
        if (sImageDetector != null) {
            sImageDetector.close();
        }
    }

    /**
     * 输入图片扫描边框顶点
     *
     * @param srcBmp 扫描图片
     * @return 返回顶点数组，以 左上，右上，右下，左下排序
     */
    public static RectangularInfo scan(Bitmap srcBmp, Bitmap previewBitmap) {
        if (srcBmp == null) {
            throw new IllegalArgumentException("srcBmp cannot be null");
        }
//        long startTime=System.nanoTime();
        long startTime=System.currentTimeMillis();
        Point[] outPoints = new Point[4];
        if (sImageDetector != null) {
            Bitmap bitmap = sImageDetector.detectImage(srcBmp);
            if (bitmap != null) {
                srcBmp = Bitmap.createScaledBitmap(bitmap, srcBmp.getWidth(), srcBmp.getHeight(), false);
                bitmap.recycle();
            }
        }
//        nativeScan(srcBmp, outPoints, sImageDetector == null, previewBitmap);

        ArrayList<Point> edgeList = nativeScan2(srcBmp, outPoints, sImageDetector == null, previewBitmap);
//        long endTime=System.nanoTime();
        long endTime=System.currentTimeMillis();;
//        Log.i("当前程序耗时：", (endTime-startTime)/1000000+"ms");  // 纳秒换算得到
        Log.i("当前程序耗时：", (endTime-startTime)+"ms");
        return new RectangularInfo(outPoints, edgeList);
    }

    /**
     * 输入图片扫描边框顶点
     *
     * @param srcBmp 扫描图片
     * @return 返回顶点数组，以 左上，右上，右下，左下排序
     */
    public static RectangularInfo scan(Bitmap srcBmp) {
        if (srcBmp == null) {
            throw new IllegalArgumentException("srcBmp cannot be null");
        }
        Point[] outPoints = new Point[4];
        if (sImageDetector != null) {
            Bitmap bitmap = sImageDetector.detectImage(srcBmp);
            if (bitmap != null) {
                srcBmp = Bitmap.createScaledBitmap(bitmap, srcBmp.getWidth(), srcBmp.getHeight(), false);
                bitmap.recycle();
            }
        }
//        nativeScan(srcBmp, outPoints, sImageDetector == null, null);
        ArrayList<Point> edgeList = nativeScan2(srcBmp, outPoints, sImageDetector == null, null);
        return new RectangularInfo(outPoints, edgeList);
    }

    /**
     * 裁剪图片
     *
     * @param srcBmp     待裁剪图片
     * @param cropPoints 裁剪区域顶点，顶点坐标以图片大小为准
     * @return 返回裁剪后的图片
     */
    public static Bitmap crop(Bitmap srcBmp, Point[] cropPoints) {
        if (srcBmp == null || cropPoints == null) {
            throw new IllegalArgumentException("srcBmp and cropPoints cannot be null");
        }
        if (cropPoints.length != 4) {
            throw new IllegalArgumentException("The length of cropPoints must be 4 , and sort by leftTop, rightTop, rightBottom, leftBottom");
        }
        Point leftTop = cropPoints[0];
        Point rightTop = cropPoints[1];
        Point rightBottom = cropPoints[2];
        Point leftBottom = cropPoints[3];

        int cropWidth = (int) ((CropUtils.getPointsDistance(leftTop, rightTop)
                + CropUtils.getPointsDistance(leftBottom, rightBottom)) / 2);
        int cropHeight = (int) ((CropUtils.getPointsDistance(leftTop, leftBottom)
                + CropUtils.getPointsDistance(rightTop, rightBottom)) / 2);

        Bitmap cropBitmap = Bitmap.createBitmap(cropWidth, cropHeight, Bitmap.Config.ARGB_8888);
        SmartCropper.nativeCrop(srcBmp, cropPoints, cropBitmap);
        return cropBitmap;
    }

    /**
     * 裁剪图片
     *
     * @param srcBmp     待裁剪图片
     * @param cropPoints 裁剪区域顶点，顶点坐标以图片大小为准
     * @param cropEdges 裁剪区域详细边缘
     * @return 返回裁剪后的图片
     */
    public static Bitmap crop(Bitmap srcBmp, Point[] srcCropPoints, Point[] cropPoints, ArrayList<Point> cropEdges) {
        if (srcBmp == null || cropPoints == null) {
            throw new IllegalArgumentException("srcBmp and cropPoints cannot be null");
        }
        if (cropPoints.length != 4) {
            throw new IllegalArgumentException("The length of cropPoints must be 4 , and sort by leftTop, rightTop, rightBottom, leftBottom");
        }
        Point leftTop = cropPoints[0];
        Point rightTop = cropPoints[1];
        Point rightBottom = cropPoints[2];
        Point leftBottom = cropPoints[3];

        int cropWidth = (int) ((CropUtils.getPointsDistance(leftTop, rightTop)
                + CropUtils.getPointsDistance(leftBottom, rightBottom)) / 2);
        int cropHeight = (int) ((CropUtils.getPointsDistance(leftTop, leftBottom)
                + CropUtils.getPointsDistance(rightTop, rightBottom)) / 2);

        Bitmap cropBitmap = Bitmap.createBitmap(cropWidth, cropHeight, Bitmap.Config.ARGB_8888);

        try {
            //        SmartCropper.nativeCrop(srcBmp, cropPoints, cropBitmap);  // 4点裁剪
            SmartCropper.nativeCrop2(srcBmp, srcCropPoints, cropPoints, cropEdges, cropBitmap);  // 详细边缘拉伸裁剪
        }
        catch (Exception e) {
            System.out.println(e);
        }

        return cropBitmap;
    }

    //C调用java中参数为string的方法
    public static void printString(String msg){
        Log.d("DEBUG: ", msg);
    }

    public static native void initImageDetector(ImageDetector imageDetector);

    // 返回4个顶点
    public static native void nativeScan(Bitmap srcBitmap, Point[] outPoints, boolean canny, Bitmap previewBitmap);

    // 返回边缘及4个顶点
    public static native ArrayList<Point> nativeScan2(Bitmap srcBitmap, Point[] outPoints, boolean canny, Bitmap previewBitmap);

    // 4点裁剪
    public static native void nativeCrop(Bitmap srcBitmap, Point[] points, Bitmap outBitmap);

    // 拉伸裁剪
    public static native void nativeCrop2(Bitmap srcBitmap, Point[] srcPoints, Point[] points, ArrayList<Point> edges, Bitmap outBitmap);


    static {
        System.loadLibrary("smart_cropper");
    }

}
