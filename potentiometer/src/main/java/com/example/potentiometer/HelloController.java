package com.example.potentiometer;

import com.fazecast.jSerialComm.SerialPort;
import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.layout.StackPane;
import javafx.scene.transform.Rotate;
import org.fxyz3d.importers.Model3D;
import org.fxyz3d.importers.obj.ObjImporter;

import java.io.IOException;
import java.net.URL;
import java.util.Scanner;


public class HelloController {

    private float gyroBiasX = 0;
    private float gyroBiasY = 0;
    private float gyroBiasZ = 0;
    private boolean isCalibrated = false;
    private int calibrationSamples = 1000; // Number of samples for calibration
    @FXML
    private Model3D model;

    @FXML
    private StackPane rootPane;

    private double roll_der, pitch_der, yaw_der; //Der verileri güncellenmesi için

    private double roll_old= 0;
    private double pitch_old = 0;
    private double yaw_old = 0;

    private double alpha = 0.96;


    private Rotate rz = new Rotate(0, Rotate.Z_AXIS);
    private Rotate rx = new Rotate(0, Rotate.X_AXIS);
    private Rotate ry = new Rotate(0, Rotate.Y_AXIS);

    public void initialize() throws IOException {
        SerialPort port = SerialPort.getCommPort("COM6");

        if(port.openPort()){
            System.out.println("COM6 is opened");
            calibrateGyro(port); //eklnedi
        } else {
            System.out.println("COM6 not found");
        }

        port.setBaudRate(9600);
        port.setComPortTimeouts(SerialPort.TIMEOUT_READ_SEMI_BLOCKING,0,0);



        Group model = importModel(getClass().getResource("/com/example/potentiometer/models/SORS.obj"));
        model.setScaleX(100);
        model.setScaleY(100);
        model.setScaleZ(100);
        //model.getTransforms().add(new Rotate(180, Rotate.Z_AXIS));
        model.setTranslateX(300);
        model.setTranslateY(275);
        model.getTransforms().addAll(rx, ry, rz);

        Thread portThread = getThread(port);
        rootPane.getChildren().add(model);
        portThread.start();
    }

    private Group importModel(URL url) throws IOException {
        ObjImporter importer = new ObjImporter();
        model = importer.load(url);

        Group modelView = new Group();
        for(Node view : model.getMeshViews()){
            modelView.getChildren().add(view);
        }

        return modelView;
    }

    private Thread getThread(SerialPort port){
        Thread thread =new Thread(() ->{
            while(true){
                readFrom(port);
            }
        });
        thread.setDaemon(true);
        return thread;
    }

    private void readFrom(SerialPort port) {
        Scanner scanner = new Scanner(port.getInputStream());
        scanner.useDelimiter("\r\n");
        long lastTime = System.nanoTime();

        while (scanner.hasNextLine()) {
            long currentTime = System.nanoTime();
            double deltaT = (currentTime - lastTime) / 1e9;
            lastTime = currentTime;

            String line = scanner.nextLine();
            String[] parts = line.split(",");
            if (parts.length < 6) continue;


            float x_g = Float.parseFloat(parts[0]) / 16384;
            float y_g = Float.parseFloat(parts[1]) / 16384;
            float z_g = Float.parseFloat(parts[2]) / 16384;
            float gyroX = (Float.parseFloat(parts[3]) / 131.0f) - gyroBiasX; //eklendi
            float gyroY = (Float.parseFloat(parts[4]) / 131.0f) - gyroBiasY;
            float gyroZ = (Float.parseFloat(parts[5]) / 131.0f) - gyroBiasZ;

            // Accelerometer'dan roll ve pitch hesapla
            double accelRoll = Math.toDegrees(Math.atan2(y_g, z_g));
            double accelPitch = Math.toDegrees(Math.atan2(-x_g, Math.sqrt(y_g*y_g + z_g*z_g)));

            if(roll_old == 0 && pitch_old == 0){
              roll_old = Math.toRadians(accelRoll);
              pitch_old = Math.toRadians(accelPitch);
            }

            // Gyro verilerini radyan/saniye'ye çevir
            double p = Math.toRadians(gyroX);
            double q = Math.toRadians(gyroY);
            double r = Math.toRadians(gyroZ);

            rotationMatrix(roll_old, pitch_old, p, q, r); //accel mi old values mu
            double[] integrationResult = integration(deltaT);

            // Complementary filter
            double gyroRollDeg = Math.toDegrees(integrationResult[0]);
            double gyroPitchDeg = Math.toDegrees(integrationResult[1]);
            double gyroYawDeg = Math.toDegrees(integrationResult[2]);

            // Apply complementary filter
            double roll = alpha * gyroRollDeg + (1 - alpha) * accelRoll;
            double pitch = alpha * gyroPitchDeg + (1 - alpha) * accelPitch;
            double yaw = gyroYawDeg;

            roll_old = Math.toRadians(roll);
            pitch_old = Math.toRadians(pitch);
            yaw_old = Math.toRadians(yaw);

            System.out.printf("Accel[g]: X=%.3f, Y=%.3f, Z=%.3f | Gyro[°/s]: X=%.2f, Y=%.2f, Z=%.2f%n",
                    x_g, y_g, z_g, gyroX, gyroY, gyroZ);

            System.out.println("\r\nYAW: " + yaw + " PITCH: " + pitch + " ROLL" + roll);

            Platform.runLater(() -> {
                rx.setAngle(roll);
                ry.setAngle(pitch);
                rz.setAngle(yaw);
            });
        }
    }

    private void rotationMatrix(double roll, double pitch, double p, double q, double r){ //AMAÇ: alınan pqr,roll ve pitch değerleriyle türevleri bulmak
        roll_der  = p + q*Math.sin(roll)*Math.tan(pitch) + r*Math.cos(roll)*Math.tan(pitch);
        pitch_der = q*Math.cos(roll) - r*Math.sin(roll);
        yaw_der   = q*Math.sin(roll)/Math.cos(pitch) + r*Math.cos(roll)/Math.cos(pitch);

    }

     private double[] integration( double deltaT){
        double roll_new, pitch_new, yaw_new;
         roll_new = deltaT * roll_der + roll_old;
         pitch_new = deltaT * pitch_der + pitch_old;
         yaw_new = deltaT * yaw_der + yaw_old;

         double[] newValues = new double[3];
         newValues[0] = roll_new;
         newValues[1] = pitch_new;
         newValues[2] = yaw_new;

         return newValues;
         //integration(roll_new,pitch_new,yaw_new); //recursive
     }

    private void calibrateGyro(SerialPort port) {
        float sumX = 0, sumY = 0, sumZ = 0;
        Scanner scanner = new Scanner(port.getInputStream());
        scanner.useDelimiter("\r\n");
        for (int i = 0; i < calibrationSamples; i++) {
            if (scanner.hasNextLine()) {
                String line = scanner.nextLine();
                String[] parts = line.split(",");
                if (parts.length >= 6) {
                    float gx = Float.parseFloat(parts[3]) / 131.0f;
                    float gy = Float.parseFloat(parts[4]) / 131.0f;
                    float gz = Float.parseFloat(parts[5]) / 131.0f;
                    sumX += gx;
                    sumY += gy;
                    sumZ += gz;
                }
            }
        }
        gyroBiasX = sumX / calibrationSamples;
        gyroBiasY = sumY / calibrationSamples;
        gyroBiasZ = sumZ / calibrationSamples;
        isCalibrated = true;
        System.out.printf("Gyro bias calibrated: X=%.3f, Y=%.3f, Z=%.3f%n", gyroBiasX, gyroBiasY, gyroBiasZ);
    }


    /*public AnchorPane anchor1;
    public VBox hbox1;
    public HBox box2;
    @FXML
    public Gauge gauge;
    @FXML
    private ToggleButton button;
    @FXML
    private Circle leftC, rightC;
    @FXML
    private Label connect;
    private int isLedOn;

    public void initialize() {
        gauge = GaugeBuilder.create()
                .title("Potentiometer")
                .unit("ADC")
                .maxValue(4095)
                .animated(true)
                .build();

        hbox1.getChildren().add(gauge);
        button.setText("LED yakmak için basınız");


        SerialPort[] ports = SerialPort.getCommPorts();
        SerialPort port = null;
        for(SerialPort p: ports ){
            System.out.println(p.getSystemPortName()+ ": " + p + "\n");
            if(p.toString().contains("CP2102 USB to UART Bridge Controller")){ port = p; }
        }


        if(port.openPort()){
            System.out.println("Port is opened\n");
        }

        port.setBaudRate(115200);
        port.setComPortTimeouts(SerialPort.TIMEOUT_READ_SEMI_BLOCKING, 0, 0); //en az bir byte veri gelmesini bekle


        SerialPort finalPort = port;
        Thread portThread = getThread(finalPort);
        portThread.start();

        button.setOnAction(e -> {
            if(button.isSelected()) {
                sendUART("1", finalPort);

                //leftC.setFill(Color.CORNSILK);
                //rightC.setFill(Color.CORNSILK);
                //button.setText("LED söndürmek için basınız");
            }


            else{
                sendUART("0", finalPort);
                leftC.setFill(Color.GREY);
                rightC.setFill(Color.GREY);
                button.setText("LED yakmak için basınız");

            }
        });


    }

    private Thread getThread(SerialPort finalPort) {
        Thread portThread = new Thread(() -> {
            while (true) {
                readFrom(finalPort);

            }
        });

        portThread.setDaemon(true);
        return portThread;
    }

    private void readFrom(SerialPort port){

        Scanner scanner = new Scanner(port.getInputStream());
        scanner.useDelimiter("\r\n"); //satırları ayırmak için

        while(scanner.hasNext()){
            String line = scanner.next();
            char first = line.charAt(0);
            isLedOn = Character.getNumericValue(first);
            int value =( Integer.parseInt(line.trim())% 10000);
            System.out.println(line);
            Platform.runLater(() -> {
                gauge.setValue(value);
                if(isLedOn== 1){
                    leftC.setFill(Color.GREENYELLOW);
                    rightC.setFill(Color.GREENYELLOW);
                    connect.setText("Bağlantı Kuruldu");
                } else if(isLedOn ==2){
                    leftC.setFill(Color.RED);
                    rightC.setFill(Color.RED);
                    connect.setText("Bağlantı Kurulamadı");
                } else if (isLedOn == 0) {
                    leftC.setFill(Color.GREY);
                    rightC.setFill(Color.GREY);
                    connect.setText("Bağlantı Bekleniyor");
                }
            });
        }

    }

    private void sendUART(String val, SerialPort port)  {
        OutputStream outputStream = port.getOutputStream();

        try {
            outputStream.write(val.getBytes());
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        try {
            outputStream.flush();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }*/ //potentiometer kodu
}

