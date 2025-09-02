package com.example.potentiometer;

import javafx.application.Application;
import javafx.fxml.FXMLLoader;
import javafx.scene.*;
import javafx.scene.effect.Blend;
import javafx.scene.effect.BlendMode;
import javafx.scene.paint.Color;
import javafx.scene.transform.Rotate;
import javafx.stage.Stage;

import java.io.IOException;
import java.net.URL;
import java.util.List;


public class HelloApplication extends Application {

    /*
    X ekseni → sağ (+), sol (-)
    Y ekseni → aşağı (+), yukarı (-)
    Z ekseni → sahneden uzak (+), kameraya doğru (-)
     */

    @Override
    public void start(Stage stage) throws IOException {

        FXMLLoader fxmlLoader = new FXMLLoader();
        fxmlLoader.setLocation(getClass().getResource("/com/example/potentiometer/hello-view.fxml"));
        Parent loader = fxmlLoader.load();

        URL url = getClass().getResource("/com/example/potentiometer/hello-view.fxml");
        if (url == null) {
            System.err.println("URL for hello-view.fxml not found");
            return;
        }
        Group root = getGroup(loader);

        stage.setScene(new Scene(root, 600, 800, Color.GREY));
        stage.setTitle("FXyz3D Sample");
        stage.show();
    }

    private static Group getGroup(Parent loader) {
        PerspectiveCamera camera = new PerspectiveCamera();
        camera.setTranslateZ(-100);
        camera.setTranslateY(100);
        //Rotate init = new Rotate(180, Rotate.Y_AXIS);
        //camera.getTransforms().add(init);

        AmbientLight light = new AmbientLight(Color.WHITE);

        light.setBlendMode(BlendMode.SCREEN);

        AmbientLight spotLight = new AmbientLight(Color.ANTIQUEWHITE);

        PointLight p2 = new PointLight(Color.ANTIQUEWHITE);
        p2.setBlendMode(BlendMode.DARKEN);

        Group root = new Group(loader,camera,p2,light);
        return root;
    }

    @Override
    public void stop(){

    }

}