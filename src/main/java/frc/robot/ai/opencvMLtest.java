package frc.robot.ai;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.TermCriteria;
import org.opencv.highgui.HighGui;
import org.opencv.imgproc.Imgproc;
import org.opencv.ml.Ml;
import org.opencv.ml.ANN_MLP;

public class opencvMLtest {
    public static void main(String[] args) {
        // Load the native OpenCV library
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        // Set up training data
        float[] trainingData = { 50,61,115,156,207,280 };
        float[] labels = {4000, 4000, 4725, 4725, 4725, 4725};
        
        Mat trainingDataMat = new Mat(trainingData.length, 2, CvType.CV_32FC1);
        Mat labelsMat = new Mat(labels.length, 1, CvType.CV_32FC1);
        int row = 0;
        for (float f : trainingData) {
            float[] aug = {f, f*f};
            trainingDataMat.put(row, 0, aug);
            row += 1;
        }
        System.out.println(trainingDataMat.dump());

        labelsMat.put(0, 0, labels);

        ANN_MLP model = ANN_MLP.create();
        int[] layer_dim = {2,4,1};
        Mat layers = new Mat(layer_dim.length, 1, CvType.CV_32SC1);
        layers.put(0,0, layer_dim);
        model.setLayerSizes(layers);
        model.setActivationFunction(ANN_MLP.SIGMOID_SYM);
        TermCriteria criteria = new TermCriteria(TermCriteria.MAX_ITER+TermCriteria.EPS, 50, 0.0001);
        model.setTermCriteria(criteria);
        model.setTrainMethod(ANN_MLP.BACKPROP, 0.0001);

        model.train(trainingDataMat, Ml.ROW_SAMPLE, labelsMat);

        for(int i = 0; i < layer_dim.length; i++) {
            System.out.println(model.getWeights(i).dump());
        }
        

        // Data for visual representation
        int width = 512, height = 512;
        Mat image = Mat.zeros(height, width, CvType.CV_8UC3);

        // Show the training data
        int thickness = -1;
        int lineType = 1; //Core.lineLINE_8;
        Imgproc.circle(image, new Point(501, 10), 5, new Scalar(255, 0, 0), thickness, lineType, 0);
        Imgproc.circle(image, new Point(10, 10), 5, new Scalar(255, 0, 0), thickness, lineType, 0);
        for( int i = 0; i < width; i += 50) {
            float x = i;
            float x2 = x*x;
            Mat samples = new Mat(1, 2, CvType.CV_32FC1);
            samples.put(0, 0, x);
            samples.put(0, 1, x2);
            System.out.print(samples.dump());
            Mat result = new Mat();
            double p = model.predict(samples, result);
            System.out.print(result.dump());
            p = result.get(0,0)[0];
            int y = (int) (p/10.0);
            Imgproc.circle(image, new Point(i, 501-y), 5, new Scalar(0, 255, 0), thickness, lineType, 0);
            System.out.println(" Point: " + x + ", " + y + "; p = " + p);
        }
        for( int i = 0; i < labels.length; i++) {
            int x = (int) trainingData[i];
            int y = (int) (labels[i]/10.0);
            Imgproc.circle(image, new Point(x, 501-y), 5, new Scalar(0, 0, 255), thickness, lineType, 0);
            //System.out.println("Point: " + x + ", " + y);
        }
        // Show support vectors
        // thickness = 2;
        // Mat sv = svm.getUncompressedSupportVectors();
        // float[] svData = new float[(int) (sv.total() * sv.channels())];
        // sv.get(0, 0, svData);
        // for (int i = 0; i < sv.rows(); ++i) {
        //     Imgproc.circle(image, new Point(svData[i * sv.cols()], svData[i * sv.cols() + 1]), 6,
        //             new Scalar(128, 128, 128), thickness, lineType, 0);
        // }
 //       Imgcodecs.imwrite("result.png", image); // save the image
        HighGui.imshow("SVM Simple Example", image); // show it to the user
        HighGui.waitKey();
        System.exit(0);
    }
}