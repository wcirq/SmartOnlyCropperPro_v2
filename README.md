# SmartCropper

## [English](README_EN.md) | 中文

简单易用的智能图片裁剪库，适用于身份证，名片，文档等照片的裁剪。

## 支持特性

- 使用智能算法(基于opencv)识别图片中的边框  
- 支持拖动锚点，手动调节选区，放大镜效果提升定位体验
- 使用透视变换裁剪并矫正选区，还原正面图片
- 支持丰富的UI设置，如辅助线，蒙版，锚点，放大镜等

## 接入

导入对应包名的aar包

注意：由于使用了 JNI， 请**不要混淆**
```
-keep class me.pqpo.smartcropperlib.**{*;}
```  

## 使用（与开源版本大体一致）

 1. 在合适的地方初始化(比如在 Application.onCreate)：
 ```java
  SmartCropper.init(this);
 ```

 2. 页面销毁时释放资源
 ```java
   SmartCropper.releaseImageDetector(this);
  ```

### 1. 裁剪布局：  
```xml
<me.pqpo.smartcropperlib.view.CropImageView   
        android:id="@+id/iv_crop"  
        android:layout_width="match_parent" 
        android:layout_height="match_parent" />  
```  

注意： CropImageView 继承至 ImageView，但是 ScaleType 必须为居中类型，如果手动设置成 fit_end,fit_start,matrix 将会报错。  

### 2. 设置待裁剪图片：    
```java
ivCrop.setImageToCrop(selectedBitmap); 
```

该方法内部会使用 native 代码智能识别边框，并绘制图片与选区。在 native 层实现，大大的提高了运行效率，运行时间与图片大小成正比，在大图片的情况下，可以考虑在子线程执行，或者压缩传入的图片。

### 3. 裁剪选区内的图片：

```java  
Bitmap crop = ivCrop.crop();  
```  

根据选区裁剪出选区内的图片，并使用透视变换矫正成正面图片。  

注意：改方法主要逻辑也是位于 native 层，运行时间与图片大小成正比，在大图片的情况下，可以考虑在子线程执行，或者压缩传入的图片。

## Attributes

|name|format|description|
|:---:|:---:|:---:|
|civMaskAlpha|integer|选区外蒙版的透明度，取值范围 0-255|
|civShowGuideLine|boolean|是否显示辅助线，默认 true|
|civLineColor|color|选区线的颜色|
|civLineWidth|dimension|选区线的宽度|
|civShowMagnifier|boolean|在拖动的时候是否显示放大镜，默认 true|
|civMagnifierCrossColor|color|放大镜十字准心的颜色|
|civGuideLineWidth|dimension|辅助线宽度|
|civGuideLineColor|color|辅助线颜色|
|civPointFillColor|color|锚点内部区域填充颜色|
|civPointFillAlpha|integer|锚点内部区域填充颜色透明度|




