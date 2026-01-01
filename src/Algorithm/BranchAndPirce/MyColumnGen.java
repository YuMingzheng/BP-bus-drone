package Algorithm.BranchAndPirce;

import Algorithm.CW.CWAlgo;
import Algorithm.Labeling.MySPPRC;
import IO.IO;
import Parameters.ExtendGraph;
import Problem.Route;
import ilog.concert.*;
import ilog.cplex.IloCplex;
import org.json.simple.parser.ParseException;

import javax.script.ScriptException;
import java.io.File;
import java.io.IOException;
import java.text.DecimalFormat;
import java.util.*;
import java.io.BufferedReader;
import java.io.FileReader;

/**
 * @author Yu Mingzheng
 * @date 2024/9/26 20:11
 * @description
 */

public class MyColumnGen {

    static class IloNumVarArray{
        int num = 0;
        IloNumVar[] array = new IloNumVar[32];

        void addVar(IloNumVar var){
            if(num >= array.length){
                IloNumVar[] newArray = new IloNumVar[2*array.length];
                System.arraycopy(array , 0 , newArray , 0,num);
                array = newArray;
            }
            array[num++] = var;
        }
        IloNumVar getElement(int index){return array[index];}

        int getSize(){ return num; }
    }

    public double computeColGen(ExtendGraph extendGraph , ArrayList<Route> routes) throws IloException, IOException {
        int i, prevCity , city;
        double cost , obj;
        double[] pi , pi1 ;
        double pi2;
        boolean onceMore;

        // ---------------------------------------------------------
        // 构建RMP的模型
        // ---------------------------------------------------------
        IloCplex cplex = new IloCplex();
        IloObjective objFunc = cplex.addMinimize();

        // Set-partitioning模型中的约束，每个约束i对应一个lpMatrix[i]
        IloRange[] lpMatrix = new IloRange[extendGraph.orderNum + 1];
        for (i = 0; i < extendGraph.orderNum; i++) {
//            lpMatrix[i] = cplex.addRange(1.0 , Double.MAX_VALUE);
//            lpMatrix[i] = cplex.addRange(1.0 , 1.0);
            lpMatrix[i] = cplex.addEq(cplex.numExpr() , 1.0);
        }

        lpMatrix[i] = cplex.addLe(cplex.numExpr() , extendGraph.droneNum); // 添加车数量约束
//        lpMatrix[i] = cplex.addEq(cplex.numExpr() , extendGraph.droneNum); // 添加车数量约束

        IloNumVarArray y = new IloNumVarArray();

        for(Route r : routes){
            int v;
            cost = 0.0;
            prevCity = 0;
            for(i = 1; i < r.getPath().size(); i++){
                city = r.getPath().get(i);
                cost += extendGraph.distanceMatExtend[prevCity][city];
                prevCity = city;
            }
            r.setDistance(cost);

            // 对于每一列column，从“上”到“下”，依次设置系数【从obj的变量系数，到每一个约束中的变量系数】
            IloColumn column = cplex.column(objFunc , r.getDistance());       // 先设置obj的系数
            for(i = 0; i < r.throughOrder.size(); i++){
                v = r.throughOrder.get(i)-1;
                column = column.and(cplex.column(lpMatrix[v] , 1.0));  // 再设置约束中的系数。1.0表示主问题中的a_{ir}的取值
            }
            y.addVar(cplex.numVar(column , 0.0 , Double.MAX_VALUE));   // 两种relaxation的方式都可以
//            y.addVar(cplex.numVar(column , 0.0 , 1.0));
        }

        // 生成初始路径
        if(routes.size() < extendGraph.orderNum){
            CWAlgo cwAlgo = new CWAlgo(extendGraph);
            Route[] ini = cwAlgo.cw_algo();
            routes.addAll(Arrays.asList(ini));
//            System.out.println(Arrays.toString(ini));

            for(Route r : routes){
                cost = 0.0;
                for (int j = 0; j < r.path.size() - 1 ; j++) {
//                    if(extendGraph.c2[r.path.get(j)][r.path.get(j + 1)] == 1)
//                        continue;
                    cost += extendGraph.distanceMatExtend[r.path.get(j)][r.path.get(j + 1)];
                }
                IloColumn column = cplex.column(objFunc , cost);
                for (int j = 0; j < r.throughOrder.size(); j++) {
                    column = column.and(cplex.column(lpMatrix[r.throughOrder.get(j)-1] , 1.0));
                }

                column = column.and(cplex.column(lpMatrix[extendGraph.orderNum ] , 1.0));
                y.addVar(cplex.numVar(column , 0.0 , Double.MAX_VALUE));
                r.setDistance(cost);
            }
        }

        cplex.setOut(null);
        // ---------------------------------------------------------
        // 列生成，不断迭代，往RMP的Ω中添加新的列
        // ---------------------------------------------------------
        DecimalFormat df = new DecimalFormat("#0000.00");
        onceMore = true;
        double[] prevObj = new double[100]; // 记录每一次迭代的obj值
        int prevI = -1;
        int count = 0;
        while(onceMore){
            onceMore = false;

            // ---------------------------------------------------------
            // 求解当前RMP
            // ---------------------------------------------------------
            if(!cplex.solve()){
                System.out.println("CG: relaxation infeasible");
                return 1E10;
            }
            prevObj[(++prevI) % 100] = cplex.getObjValue();

            // ---------------------------------------------------------
            // 求解子问题，找到新的columns
            // ---------------------------------------------------------

            pi = cplex.getDuals(lpMatrix);
//            System.out.println(Arrays.toString(pi));
            pi1 = Arrays.copyOfRange(pi , 0 , pi.length-1);
            pi2 = pi[pi.length-1];
            Map<Integer,Double> lambda = new HashMap<>(extendGraph.orderNum+1);
            for (int k = 1; k <= extendGraph.orderNum; k++) {
                lambda.put(k, pi1[k-1]);
            }
            MySPPRC spprc = new MySPPRC(extendGraph);
            ArrayList<Route> routesSPPRC = new ArrayList<>();
            spprc.solve(lambda , routesSPPRC , pi2);

            if(!routesSPPRC.isEmpty()){
                for (Route r : routesSPPRC) {
                    ArrayList<Integer> route = r.getPath();
                    prevCity = route.get(1);
                    cost = extendGraph.distanceMatExtend[0][prevCity];
                    IloColumn column = null;
                    boolean ini = false;
                    if (1 <= prevCity && prevCity <= extendGraph.orderNum) {
                        column = cplex.column(lpMatrix[route.get(1) - 1] , 1.0);
                        ini = true;
                    }
                    for(i = 2 ; i < route.size() -1 ; i++){
                        city = route.get(i);
                        cost += extendGraph.distanceMatExtend[prevCity][city];
                        prevCity = city;
                        if (1 <= prevCity && prevCity <= 1 + extendGraph.orderNum) {
                            if(ini) {
                                column = column.and(cplex.column(lpMatrix[route.get(i) - 1], 1.0));
                            }else{
                                column = cplex.column(lpMatrix[route.get(i) - 1] , 1.0);
                                ini = true;
                            }
                        }
                    }


                    cost += extendGraph.distanceMatExtend[prevCity][extendGraph.nodeNumExtend - 1];
                    column = column.and(cplex.column(objFunc , cost));

                    column = column.and(cplex.column(lpMatrix[lpMatrix.length-1] , 1.0));


                    y.addVar(cplex.numVar(column , 0.0 ,Double.MAX_VALUE, "P"+count++));
                    r.setDistance(cost);
                    routes.add(r);
                    onceMore = true;
                }
//                System.out.print("\nCG Iter " + prevI + " Current cost: " + df.format(prevObj[prevI % 100]) + " " + routes.size()+ " routes   ");
//                System.out.flush();
            }else{
//                System.out.println("CG终止");


                cplex = new IloCplex();
                objFunc = cplex.addMinimize();

                // Set-partitioning模型中的约束，每个约束i对应一个lpMatrix[i]
                lpMatrix = new IloRange[extendGraph.orderNum + 1];
                for (i = 0; i < extendGraph.orderNum; i++) {
                    lpMatrix[i] = cplex.addEq(cplex.numExpr() , 1.0);
                }

                lpMatrix[i] = cplex.addLe(cplex.numExpr() , extendGraph.droneNum); // 添加车数量约束

                y = new IloNumVarArray();

                for (Route r : routes) {
                    ArrayList<Integer> route = r.getPath();
                    prevCity = route.get(1);
                    cost = extendGraph.distanceMatExtend[0][prevCity];
                    IloColumn column = null;
                    boolean ini = false;
                    if (1 <= prevCity && prevCity <= extendGraph.orderNum) {
                        column = cplex.column(lpMatrix[route.get(1) - 1] , 1.0);
                        ini = true;
                    }
                    for(i = 2 ; i < route.size() -1 ; i++){
                        city = route.get(i);
                        cost += extendGraph.distanceMatExtend[prevCity][city];
                        prevCity = city;
                        if (1 <= prevCity && prevCity <= 1 + extendGraph.orderNum) {
                            if(ini) {
                                column = column.and(cplex.column(lpMatrix[route.get(i) - 1], 1.0));
                            }else{
                                column = cplex.column(lpMatrix[route.get(i) - 1] , 1.0);
                                ini = true;
                            }
                        }
                    }


                    cost += extendGraph.distanceMatExtend[prevCity][extendGraph.nodeNumExtend - 1];
                    column = column.and(cplex.column(objFunc , cost));

                    column = column.and(cplex.column(lpMatrix[lpMatrix.length-1] , 1.0));


                    y.addVar(cplex.intVar(column ,0,1, "P"+count++));
                    r.setDistance(cost);
                }
//                System.out.print("\nCG IP Model " + " Current cost: " + df.format(prevObj[prevI % 100]) + " " + routes.size()+ " routes 【END】\n");
//                System.out.flush();

                cplex.setOut(null);
//                cplex.exportModel("./ExportModel/temp"+prevI+".lp");

                if(!cplex.solve()){
                    System.out.println("CG IP 无解");
                    return 1E10;
                }
            }
//            cplex.exportModel("./ExportModel/temp"+prevI+".lp");
        }


        for(i = 0; i < y.getSize() ; i++)
            routes.get(i).setQ(cplex.getValue(y.getElement(i)));
        obj = cplex.getObjValue();
        cplex.end();
        return obj;
    }

    public void initialRoute(ExtendGraph extendGraph , ArrayList<Route> routes){
        String filePath = "D:\\硕士毕设\\bus+drone\\bus-drone-code\\data\\initial_solution.txt";
        List<List<Integer>> allLines = new ArrayList<>();

        try (BufferedReader br = new BufferedReader(new FileReader(filePath))) {
            String line;
            while ((line = br.readLine()) != null) {
                List<Integer> numbers = new ArrayList<>();
                String[] tokens = line.split(","); // 根据逗号分割
                for (String token : tokens) {
                    numbers.add(Integer.parseInt(token.trim())); // 将字符串转换为整数并添加到列表
                }
                allLines.add(numbers);


                Route route = new Route();
                for(int i = 0 ; i < numbers.size() ; i++){
                    route.path.add(numbers.get(i));
                    if(1<=numbers.get(i) && numbers.get(i) <= extendGraph.orderNum){
                        route.throughOrder.add(numbers.get(i));
                    }
                    if(i != numbers.size()-1){
                        route.distance += extendGraph.distanceMatExtend[numbers.get(i)][numbers.get(i+1)];
                    }
                }
                routes.add(route);
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }


    public static void main(String[] args) throws IloException, ScriptException, IOException, ParseException {

        double start = System.currentTimeMillis();
        String instanceName = "50_1_1_1_26166";
        String path = "D:\\硕士毕设\\bus+drone\\bus-drone-code\\data\\choose_model_instance\\" + instanceName + ".json";

        ExtendGraph extendGraph = new ExtendGraph(path , -1 , "rf");
        ArrayList<Route> bestRoutes = new ArrayList<>();
        MyColumnGen cg = new MyColumnGen();
        double obj = cg.computeColGen(extendGraph , bestRoutes );

        System.out.print("Obj :     " + obj);
        System.out.println("      Time consumption:     " + (System.currentTimeMillis() - start) / 1000.0);
        for (Route bestRoute : bestRoutes)
            if (bestRoute.getQ() != 0)
                System.out.println(" Q : " + bestRoute.getQ() + " " + bestRoute);
        System.out.println("  Path size: " + bestRoutes.size());


//        for (File file : IO.getFilesInDirectory("D:\\硕士毕设\\bus+drone\\bus-drone-code\\data\\gurobi_algo_instance")) {
//            System.out.print("---------" + file.getName() + " --- ");
//            double start = System.currentTimeMillis();
////            String instanceName = "60_2_4_1_4009";
////            String path = "D:\\硕士毕设\\bus+drone\\bus-drone-code\\data\\gurobi_algo_instance\\50_1_1_1_8545.json";
////            String path = "D:\\硕士毕设\\bus+drone\\bus-drone-code\\data\\threshold_instance\\" + instanceName + ".json";
////            ExtendGraph extendGraph = new ExtendGraph(path , true);
////            ExtendGraph extendGraph = new ExtendGraph(path , -1);
//            ExtendGraph extendGraph = new ExtendGraph("D:\\硕士毕设\\bus+drone\\bus-drone-code\\data\\gurobi_algo_instance\\" + file.getName() , -1);
//            ArrayList<Route> bestRoutes = new ArrayList<>();
//            MyColumnGen cg = new MyColumnGen();
//            double obj = cg.computeColGen(extendGraph , bestRoutes );
//
//            System.out.print("Obj :     " + obj);
//            System.out.println("      Time consumption:     " + (System.currentTimeMillis() - start) / 1000.0);
////        for (Route bestRoute : bestRoutes)
////            if (bestRoute.getQ() != 0)
////                System.out.println(" Q : " + bestRoute.getQ() + " " + bestRoute);
////            System.out.println("  Path size: " + bestRoutes.size());
//        }


    }
}
