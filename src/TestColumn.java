import ilog.concert.*;
import ilog.cplex.IloCplex;

/**
 * @author Yu Mingzheng
 * @date 2024/9/4 19:16
 * @description
 */
public class TestColumn {
    public static void main(String[] args) {
        try {
            // 创建CPLEX环境
            IloCplex cplex = new IloCplex();

            // 定义目标函数：最大化
            IloObjective obj = cplex.addMaximize();

            // 定义约束expr1: x1 + x2 <= 10
            IloRange expr1 = cplex.addLe(cplex.numExpr(), 10);

            // 定义约束expr2: x1 + 2*x2 <= 15
            IloRange expr2 = cplex.addLe(cplex.numExpr(), 15);

            // 创建变量x1的列：目标函数中系数为1.0，约束expr1中的系数为1.0
            IloColumn colX1 = cplex.column(obj, 1.0).and(cplex.column(expr1, 1.0));

            // 创建变量x2的列：目标函数中系数为1.0，约束expr1中的系数为1.0，约束expr2中的系数为2.0
            IloColumn colX2 = cplex.column(obj, 1.0).and(cplex.column(expr1, 1.0)).and(cplex.column(expr2, 2.0));

            // 使用列对象创建变量
            IloNumVar x1 = cplex.numVar(colX1, 0, Double.MAX_VALUE);
            IloNumVar x2 = cplex.numVar(colX2, 0, Double.MAX_VALUE);


            cplex.exportModel("TestModel.lp");
            // 求解模型
            if (cplex.solve()) {
                System.out.println("Objective Value: " + cplex.getObjValue());
                System.out.println("x1: " + cplex.getValue(x1));
                System.out.println("x2: " + cplex.getValue(x2));
            } else {
                System.out.println("Model not solved.");
            }

            // 释放CPLEX资源
            cplex.end();
        } catch (IloException e) {
            e.printStackTrace();
        }
    }
}
