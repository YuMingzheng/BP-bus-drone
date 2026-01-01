import Algorithm.BranchAndPirce.MyColumnGen;
import Problem.Route;
import Parameters.ExtendGraph;
import ilog.concert.IloException;
import org.json.simple.parser.ParseException;
import IO.IO;

import javax.script.ScriptException;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * @author Yu Mingzheng
 * @date 2024/8/29 15:26
 * @description
 */
public class Main {
    public static void main(String[] args) throws ScriptException, IOException, ParseException, IloException {
        List<File> files = IO.getFilesInDirectory("D:\\硕士毕设\\bus+drone\\bus-drone-code\\data\\train_model_instance");
        for (File file : files) {
            String fileName = file.getName();
            String instanceName = fileName.substring(0, fileName.lastIndexOf("."));
            System.out.println("==============================");
            System.out.println("Processing " + instanceName );
            double start = System.currentTimeMillis();

            String path = "D:\\硕士毕设\\bus+drone\\bus-drone-code\\data\\train_model_instance\\" + instanceName + ".json";

            ExtendGraph extendGraph = new ExtendGraph(path , -1, "lgb");

            ArrayList<Route> bestRoutes = new ArrayList<>();
            MyColumnGen cg = new MyColumnGen();
            double obj = cg.computeColGen(extendGraph , bestRoutes );


//            System.out.println("Obj : " + obj);
            System.out.println("Time consumption: " + (System.currentTimeMillis() - start) / 1000.0);

//            for (Route bestRoute : bestRoutes)
//                if (bestRoute.getQ() != 0)
//                    System.out.println("Q : " + bestRoute.getQ() + " " + bestRoute);
//            System.out.println("Path size: " + bestRoutes.size());


            Map<List<Integer>, Integer> pathFrequencyMap = getStringIntegerMap(bestRoutes);

            List<List<Integer>> result = new ArrayList<>();
            for (int i = 0; i < extendGraph.nodeNumExtend; i++) {
                for (int j = 0; j < extendGraph.nodeNumExtend; j++) {
                    if(extendGraph.c1[i][j] == 1 || extendGraph.c2[i][j] == 1){
                        List<Integer> value = new ArrayList<>();
                        value.add(i);
                        value.add(j);
                        value.add(pathFrequencyMap.getOrDefault(value, 0));

                        result.add(value);
                    }
                }
            }
            IO.writeToFile(result, "./Output/train_data/" + instanceName + ".txt" , true);
        }



    }

    private static Map<List<Integer>, Integer> getStringIntegerMap(ArrayList<Route> bestRoutes) {
        Map<List<Integer>, Integer> pathFrequencyMap = new HashMap<>();

        // 遍历每个 Route 对象
        for (Route route : bestRoutes) {
            ArrayList<Integer> path = route.path;

            // 遍历路径中的相邻节点，记录每个路径段（从 A 到 B）
            for (int i = 0; i < path.size() - 1; i++) {
                // 生成 A->B 的路径段
                List<Integer> segment = new ArrayList<>();
                segment.add(path.get(i));
                segment.add(path.get(i + 1));

                pathFrequencyMap.put(segment, pathFrequencyMap.getOrDefault(segment, 0) + 1);
            }
        }
        return pathFrequencyMap;
    }


}
