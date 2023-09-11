package me.pqpo.smartcropper;

import android.app.Application;

import com.auto.crop.SmartCropper;


public class App extends Application {

    @Override
    public void onCreate() {
        super.onCreate();
        SmartCropper.init(this);
    }

    @Override
    public void onTerminate() {
        super.onTerminate();
        SmartCropper.releaseImageDetector();
    }
}
