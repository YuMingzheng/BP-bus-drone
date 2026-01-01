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
import java.util.Objects;

/**
 * @author Yu Mingzheng
 * @date 2025/2/10 21:49
 * @description
 */
public class ChooseModelMain {
    public static void main(String[] args) throws ScriptException, IOException, ParseException, IloException {
        String nameTemp = "50_1_1_1_26166";
        String pathTemp = "D:\\硕士毕设\\bus+drone\\bus-drone-code\\data\\choose_model_instance\\" + nameTemp + ".json";
        ExtendGraph temp = new ExtendGraph(pathTemp , 0.3 , "lgb");


        List<File> files = IO.getFilesInDirectory(
                "D:\\硕士毕设\\bus+drone\\bus-drone-code\\data\\choose_model_instance"
        );

        for (File file : files) {
            String fileName = file.getName();
            String instanceName = fileName.substring(0, fileName.lastIndexOf("."));
            System.out.println("Processing " + instanceName );

            String[] modelNames = {"-1" , "rf" , "lr" , "lgb" ,  "xgb"};
            for (String modelName : modelNames) {
                System.out.print(" 【model " + modelName + "】 ");
                double threshold = 0.3;

                double start = System.currentTimeMillis();

                String path = "D:\\硕士毕设\\bus+drone\\bus-drone-code\\data\\choose_model_instance\\" + instanceName + ".json";

                ExtendGraph extendGraph;
                if(!modelName.equals("-1")){
                    extendGraph = new ExtendGraph(path , threshold , modelName);
                }else{
                    extendGraph = new ExtendGraph(path , -1 , modelName);
                }
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
                        result[2] = modelName;
                        result[3] = "-";
                        result[4] = "-";
                        result[5] = "-";
                        IO.writeToFile(result , "D:\\硕士毕设\\bus+drone\\bus-drone-code\\algorithm\\Output\\choose_model.txt" , true);
                    } else {
                        throw e;
                    }
                    continue;
                }

                System.out.print("Obj : " + obj);
                double timeCost = (System.currentTimeMillis() - start) / 1000.0;
                System.out.print("  Time consumption: " + timeCost);
                System.out.println();
                String[] result = new String[6];
                result[0] = instanceName;
                result[1] = String.valueOf(threshold);
                result[2] = modelName;
                result[3] = String.valueOf(timeCost);
                result[4] = String.valueOf(obj);
                result[5] = String.valueOf(bestRoutes.size());
                IO.writeToFile(result , "D:\\硕士毕设\\bus+drone\\bus-drone-code\\algorithm\\Output\\choose_model.txt" , true);


            System.out.println("\n");
            }

        }
    }

}
