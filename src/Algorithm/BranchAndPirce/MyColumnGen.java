package Algorithm.BranchAndPirce;

import Algorithm.Labeling.MySPPRC;
import Parameters.ExtendGraph;
import Problem.Route;
import ilog.concert.*;
import ilog.cplex.IloCplex;
import org.json.simple.parser.ParseException;

import javax.script.ScriptException;
import java.io.IOException;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

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

    public double computeColGen(ExtendGraph extendGraph , ArrayList<Route> routes) throws IloException {
        int i, prevCity , city;
        double cost , obj;
        double[] pi;
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
            r.setCost(cost);

            // 对于每一列column，从“上”到“下”，依次设置系数【从obj的变量系数，到每一个约束中的变量系数】
            IloColumn column = cplex.column(objFunc , r.getCost());       // 先设置obj的系数
            for(i = 1; i < r.getPath().size()-1; i++){
                v = r.getPath().get(i) - 1;
                column = column.and(cplex.column(lpMatrix[v] , 1.0));  // 再设置约束中的系数。1.0表示主问题中的a_{ir}的取值
            }
            y.addVar(cplex.numVar(column , 0.0 , Double.MAX_VALUE));   // 两种relaxation的方式都可以
//            y.addVar(cplex.numVar(column , 0.0 , 1.0));
        }

        // 生成初始路径
        if(routes.size() < extendGraph.orderNum){
            initialRoute(extendGraph , routes);
            for(Route r : routes){
                cost = 0.0;
                for (int j = 0; j < r.path.size() - 1 ; j++) {
                    cost += extendGraph.distanceMatExtend[r.path.get(j)][r.path.get(j + 1)];
                }
                IloColumn column = cplex.column(objFunc , cost);
                for (int j = 0; j < r.throughOrder.size(); j++) {
                    column = column.and(cplex.column(lpMatrix[r.throughOrder.get(j)-1] , 1.0));
                }

                column = column.and(cplex.column(lpMatrix[extendGraph.orderNum ] , 1.0));
                y.addVar(cplex.numVar(column , 0.0 , Double.MAX_VALUE));
                r.setCost(cost);
            }
        }

        cplex.setOut(null);
        cplex.exportModel("new model.lp");
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
            System.out.println(Arrays.toString(pi));
            pi = Arrays.copyOfRange(pi , 0 , pi.length-1);
            Map<Integer,Double> lambda = new HashMap<>(extendGraph.orderNum+1);
            for (int k = 1; k <= extendGraph.orderNum; k++) {
                lambda.put(k, pi[k-1]);
            }
            MySPPRC spprc = new MySPPRC(extendGraph);
            ArrayList<Route> routesSPPRC = new ArrayList<>();
            spprc.solve(lambda , routesSPPRC);

            if(routesSPPRC.size() > 0){
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
                    r.setCost(cost);
                    routes.add(r);
                    onceMore = true;
                }
                System.out.print("\nCG Iter " + prevI + " Current cost: " + df.format(prevObj[prevI % 100]) + " " + routes.size()+ " routes   ");
                System.out.flush();
            }
            cplex.exportModel("./ExportModel/temp"+prevI+".lp");
        }

        System.out.println();

        for(i = 0; i < y.getSize() ; i++)
            routes.get(i).setQ(cplex.getValue(y.getElement(i)));
        obj = cplex.getObjValue();
        cplex.end();
        return obj;
    }

    public void initialRoute(ExtendGraph extendGraph , ArrayList<Route> routes){
        MySPPRC mySPPRC = new MySPPRC(extendGraph);
        Map<Integer, Double> dualPrices = new HashMap<>(extendGraph.orderNum);
        for (int k = 1; k <= extendGraph.orderNum; k++) {
            dualPrices.put(k, 999d);
        }
        mySPPRC.solve(dualPrices , routes);
    }

    public static void main(String[] args) throws IloException, ScriptException, IOException, ParseException {
        double start = System.currentTimeMillis();
        ExtendGraph extendGraph = new ExtendGraph();
        ArrayList<Route> bestRoutes = new ArrayList<>();
        MyColumnGen cg = new MyColumnGen();
        double obj = cg.computeColGen(extendGraph , bestRoutes);
        System.out.println("Obj : " + obj);
        System.out.println("Time consumption: " + (System.currentTimeMillis() - start) / 1000.0);

        for (int i = 0; i < bestRoutes.size(); i++)
            if(bestRoutes.get(i).getQ() != 0)
                System.out.println("Q : " + bestRoutes.get(i).getQ() + " " +  bestRoutes.get(i));
        System.out.println("Path size: " + bestRoutes.size());
        //-------------------------------------------

//        ExtendGraph extendGraph = new ExtendGraph();
//        ArrayList<Route> routes = new ArrayList<>();
//        MyColumnGen cg = new MyColumnGen();
//        cg.initialRoute(extendGraph, routes);
//        System.out.println(routes);
    }
}
