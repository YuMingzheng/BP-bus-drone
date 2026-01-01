import Algorithm.BranchAndPirce.MyColumnGen;
import IO.IO;
import Parameters.ExtendGraph;
import Problem.Route;
import ilog.concert.IloException;
import org.json.simple.parser.ParseException;

import javax.script.ScriptException;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * @author Yu Mingzheng
 * @date 2025/1/14 11:36
 * @description
 */
public class ThresholdMain {
    public static void main(String[] args) throws ScriptException, IOException, ParseException, IloException {
        List<File> files = IO.getFilesInDirectory("D:\\硕士毕设\\bus+drone\\bus-drone-code\\data\\threshold_instance");
        double[] thresholds = new double[]{-1 , 0.01 , 0.1 , 0.2 , 0.3 , 0.4 , 0.5 , 0.6 , 0.7 , 0.8 , 0.9};

        for (File file : files) {
            String fileName = file.getName();
            String instanceName = fileName.substring(0, fileName.lastIndexOf("."));
            System.out.println("Processing " + instanceName );
            if(Objects.equals(instanceName.split("_")[0], "110"))
                continue;
            for (double threshold : thresholds) {
                for (int i = 0; i < 5; i++) {
                    System.out.print("【Threshold 】" + threshold + " 【iter " + (i+1) + "】 ");
                    double start = System.currentTimeMillis();

                    String path = "D:\\硕士毕设\\bus+drone\\bus-drone-code\\data\\threshold_instance\\" + instanceName + ".json";

                    ExtendGraph extendGraph = new ExtendGraph(path , threshold, "lgb");

                    ArrayList<Route> bestRoutes = new ArrayList<>();
                    MyColumnGen cg = new MyColumnGen();
                    double obj = -1;
                    try{
                        obj = cg.computeColGen(extendGraph , bestRoutes );
                    }catch (NullPointerException e) {
                        String errorMessage = e.getMessage();
                        if (errorMessage != null && errorMessage.contains("未找到最短路径")) {
                            String[] result = new String[6];
                            result[0] = instanceName;
                            result[1] = String.valueOf(threshold);
                            result[2] = String.valueOf((i+1));
                            result[3] = "-";
                            result[4] = "-";
                            result[5] = "-";
                            IO.writeToFile(result , "D:\\硕士毕设\\bus+drone\\bus-drone-code\\algorithm\\Output\\threshold_output.txt" , true);
                        } else {
                            throw e;
                        }
                    }

                    System.out.print("Obj : " + obj);
                    double timeCost = (System.currentTimeMillis() - start) / 1000.0;
                    System.out.print("  Time consumption: " + timeCost);
//                for (Route bestRoute : bestRoutes)
//                    if (bestRoute.getQ() != 0)
//                        System.out.println(" Q : " + bestRoute.getQ() + " " + bestRoute);
//                System.out.println("  Path size: " + bestRoutes.size());
                    System.out.println();
                    String[] result = new String[6];
                    result[0] = instanceName;
                    result[1] = String.valueOf(threshold);
                    result[2] = String.valueOf((i+1));
                    result[3] = String.valueOf(timeCost);
                    result[4] = String.valueOf(obj);
                    result[5] = String.valueOf(bestRoutes.size());
                    IO.writeToFile(result , "D:\\硕士毕设\\bus+drone\\bus-drone-code\\algorithm\\Output\\threshold_output.txt" , true);
                }

                System.out.println("\n");
            }

        }
    }
}
