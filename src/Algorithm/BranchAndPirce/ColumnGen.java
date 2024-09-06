package Algorithm.BranchAndPirce;

import Algorithm.Labeling.SPPRC;
import Parameters.Parameters;
import VRP.Route;
import ilog.concert.*;
import ilog.cplex.IloCplex;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

/**
 * @author Yu Mingzheng
 * @date 2024/9/3 16:01
 * @description
 */
public class ColumnGen {
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

    public double computeColGen(Parameters parameters , ArrayList<Route> routes) throws IloException {
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
        IloRange[] lpMatrix = new IloRange[Parameters.numClient + 1];
        for (i = 0; i < Parameters.numClient; i++) {
            lpMatrix[i] = cplex.addRange(1.0 , Double.MAX_VALUE);
//            lpMatrix[i] = cplex.addRange(1.0 , 1.0);
//            lpMatrix[i] = cplex.addEq(cplex.numExpr() , 1.0);
        }
        lpMatrix[i] = cplex.addLe(cplex.numExpr() , 5+0.1); // 添加车数量约束

        IloNumVarArray y = new IloNumVarArray();

        for(Route r : routes){
            int v;
            cost = 0.0;
            prevCity = 0;
            for(i = 1; i < r.getPath().size(); i++){
                city = r.getPath().get(i);
                cost += parameters.dist[prevCity][city];
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
        if(routes.size() < Parameters.numClient){
            for(i = 0; i < Parameters.numClient; i++){
                cost = parameters.dist[0][i+1] + parameters.dist[i+1][Parameters.numClient +1];
                IloColumn column = cplex.column(objFunc , cost);
                column = column.and(cplex.column(lpMatrix[i] , 1.0));

                y.addVar(cplex.numVar(column , 0.0 , Double.MAX_VALUE));
                Route newRoute = new Route();
                newRoute.addCity(0);
                newRoute.addCity(i);
                newRoute.addCity(Parameters.numClient + 1);
                newRoute.setCost(cost);
                routes.add(newRoute);
            }
        }

        cplex.setOut(null);
//        cplex.exportModel("temp.lp");
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
            // solve the current RMP
            // ---------------------------------------------------------
            if(!cplex.solve()){
                System.out.println("CG: relaxation infeasible");
                return 1E10;
            }

            prevObj[(++prevI) % 100] = cplex.getObjValue();

            // ---------------------------------------------------------
            // solve the sub-problem to find new columns
            // ---------------------------------------------------------

            pi = cplex.getDuals(lpMatrix);
            pi = Arrays.copyOfRange(pi , 0 , pi.length-1);
            Map<Integer,Double> lambda = new HashMap<>(Parameters.numClient);
            for (int k = 1; k <= Parameters.numClient; k++) {
                lambda.put(k, pi[k-1]);
            }
            SPPRC spprc = new SPPRC(parameters);
            ArrayList<Route> routesSPPRC = new ArrayList<>();
            spprc.solve(lambda , routesSPPRC);

            if(routesSPPRC.size() > 0){
                for (Route r : routesSPPRC) {
                    ArrayList<Integer> route = r.getPath();
                    prevCity = route.get(1);
                    cost = parameters.dist[0][prevCity];
                    IloColumn column = cplex.column(lpMatrix[route.get(1) - 1] , 1.0);
                    for(i = 2 ; i < route.size() -1 ; i++){
                        city = route.get(i);
                        cost += parameters.dist[prevCity][city];
                        prevCity = city;
                        column = column.and(cplex.column(lpMatrix[route.get(i)-1],1.0));
                    }
                    cost += parameters.dist[prevCity][Parameters.numClient + 1];
                    column = column.and(cplex.column(objFunc , cost));

                    column = column.and(cplex.column(lpMatrix[lpMatrix.length-1] , 1.0));



                    y.addVar(cplex.numVar(column , 0.0 ,Double.MAX_VALUE, "P"+count++));
                    r.setCost(cost);
                    routes.add(r);
                    onceMore = true;
                }
                System.out.print("\nCG Iter " + prevI + " Current cost: " + df.format(prevObj[prevI % 100]) + " " + routes.size()+ " routes");
                System.out.flush();
            }
        }
        cplex.exportModel("temp.lp");
        System.out.println();

        for(i = 0; i < y.getSize() ; i++)
            routes.get(i).setQ(cplex.getValue(y.getElement(i)));
        obj = cplex.getObjValue();
        cplex.end();
        return obj;
    }

    public static void main(String[] args) throws IloException{
        double start = System.currentTimeMillis();

        Parameters parameters = new Parameters("C:\\Users\\31706\\Desktop\\exact-algorithm\\instances\\solomon_50\\c101.txt");

        ArrayList<Route> bestRoutes = new ArrayList<>();

        ColumnGen cg = new ColumnGen();
        double obj = cg.computeColGen(parameters , bestRoutes);

        System.out.println("Obj : " + obj);
        System.out.println("Time consumption: " + (System.currentTimeMillis() - start) / 1000.0);

        for (int i = 0; i < bestRoutes.size(); i++) {
            if(bestRoutes.get(i).getQ() != 0)
                System.out.println("Q : " + bestRoutes.get(i).getQ() + " " +  bestRoutes.get(i));
        }
        System.out.println("Path size: " + bestRoutes.size());
    }
}