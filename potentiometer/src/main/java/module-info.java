module com.example.potentiometer {
    requires javafx.controls;
    requires javafx.fxml;
    requires com.fazecast.jSerialComm;
    requires eu.hansolo.medusa;
    requires org.fxyz3d.core;
    requires org.fxyz3d.importers;


    opens com.example.potentiometer to javafx.fxml;
    exports com.example.potentiometer;
}