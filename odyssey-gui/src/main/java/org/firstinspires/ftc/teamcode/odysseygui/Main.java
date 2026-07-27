package org.firstinspires.ftc.teamcode.odysseygui;

import static org.firstinspires.ftc.teamcode.odysseygui.FieldCoordinates.fieldX_MM_TO_PX;
import static org.firstinspires.ftc.teamcode.odysseygui.FieldCoordinates.fieldX_PX_TO_MM;
import static org.firstinspires.ftc.teamcode.odysseygui.FieldCoordinates.fieldY_MM_TO_PX;
import static org.firstinspires.ftc.teamcode.odysseygui.FieldCoordinates.fieldY_PX_TO_MM;

import org.firstinspires.ftc.teamcode.odyssey.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.odyssey.path.BezierCurve;

import javafx.application.Application;
import javafx.scene.Scene;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.image.Image;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.scene.shape.Circle;
import javafx.stage.Stage;

public class Main extends Application {

    private Canvas canvas;
    private GraphicsContext gc;
    private Image fieldImage;
    Vector2d p0 = new Vector2d(500, 500);
    Vector2d p1 = new Vector2d(500, 3000);
    Vector2d p2 = new Vector2d(3000, 3000);
    Vector2d p3 = new Vector2d(3000, 500);
    @Override
    public void start(Stage stage) {
        canvas = new Canvas(650, 650);
        gc = canvas.getGraphicsContext2D();

        fieldImage = new Image(getClass().getResourceAsStream("/field.png"));


        Pane root = new Pane(canvas);

        Circle h0 = createHandle(p0, Color.GREEN, 1);
        Circle h1 = createHandle(p1, Color.BLUE, 2);
        Circle h2 = createHandle(p2, Color.BLUE, 3);
        Circle h3 = createHandle(p3, Color.GREEN, 4);

        root.getChildren().addAll(h0, h1, h2, h3);

        drawCurve();

        Scene scene = new Scene(root, 650, 650);
        stage.setScene(scene);
        stage.setTitle("Odyssey Path Editor");
        stage.show();
    }

    private Circle createHandle(Vector2d vector, Color color, int index) {
        Circle circle = new Circle(fieldX_MM_TO_PX(vector.getX()), fieldY_MM_TO_PX(vector.getY()), 8, color);

        final double[] dragDelta = new double[2];

        circle.setOnMousePressed(e -> {
            dragDelta[0] = circle.getCenterX() - e.getX();
            dragDelta[1] = circle.getCenterY() - e.getY();
        });

        circle.setOnMouseDragged(e -> {
            double newX = e.getX() + dragDelta[0];
            double newY = e.getY() + dragDelta[1];

            if (newX >= 0 && newX <= canvas.getWidth()) circle.setCenterX(newX);
            if (newY >= 0 && newY <= canvas.getHeight()) circle.setCenterY(newY);

            Vector2d updatedVector = new Vector2d(
                    fieldX_PX_TO_MM(circle.getCenterX()),
                    fieldY_PX_TO_MM(circle.getCenterY())
            );
            switch (index) {
                case 1:
                    p0 = updatedVector;
                    break;
                case 2:
                    p1 = updatedVector;
                    break;
                case 3:
                    p2 = updatedVector;
                    break;
                case 4:
                    p3 = updatedVector;
                    break;
            }

            drawCurve();
        });

        return circle;
    }

    private void drawCurve() {
        gc.drawImage(fieldImage, 0, 0, canvas.getWidth(), canvas.getHeight());

        gc.setStroke(Color.LIGHTGRAY);
        gc.setLineWidth(1);
        gc.strokeLine(fieldX_MM_TO_PX(p0.getX()), fieldY_MM_TO_PX(p0.getY()), fieldX_MM_TO_PX(p1.getX()), fieldY_MM_TO_PX(p1.getY()));
        gc.strokeLine(fieldX_MM_TO_PX(p1.getX()), fieldY_MM_TO_PX(p1.getY()), fieldX_MM_TO_PX(p2.getX()), fieldY_MM_TO_PX(p2.getY()));
        gc.strokeLine(fieldX_MM_TO_PX(p2.getX()), fieldY_MM_TO_PX(p2.getY()), fieldX_MM_TO_PX(p3.getX()), fieldY_MM_TO_PX(p3.getY()));

        BezierCurve curve = new BezierCurve(p0, p1, p2, p3);

        gc.setStroke(Color.YELLOW);
        gc.setLineWidth(3);

        Vector2d prevPoint = curve.getPoint(0);
        for (double t = 0.005; t <= 1; t+=0.005) {
            Vector2d nextPoint = curve.getPoint(t);
            gc.strokeLine(fieldX_MM_TO_PX(prevPoint.getX()), fieldY_MM_TO_PX(prevPoint.getY()), fieldX_MM_TO_PX(nextPoint.getX()), fieldY_MM_TO_PX(nextPoint.getY()));
            prevPoint = nextPoint;
        }
    }



    public static void main(String[] args) {
        launch(args);
    }
}