package Model;

import Parameters.ExtendGraph;
import Problem.Node.TimeNode;
import com.gurobi.gurobi.*;
import org.json.simple.parser.ParseException;

import javax.script.ScriptException;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import IO.IO;

/**
 * @author Yu Mingzheng
 * @date 2025/1/27 15:58
 * @description
 */
public class GurobiModel {
    public static void main(String[] args) throws ScriptException, IOException, ParseException {

        GRBEnv env;//"D:\\硕士毕设\\bus+drone\\bus-drone-code\\data\\gurobi_algo_instance"
        for (File file : IO.getFilesInDirectory("D:\\硕士毕设\\bus+drone\\bus-drone-code\\data\\gurobi_algo_instance")) {
            System.out.print("---------" + file.getName() + " --- ");
            String path = "D:\\硕士毕设\\bus+drone\\bus-drone-code\\data\\gurobi_algo_instance\\" + file.getName();
            ExtendGraph extendGraph = new ExtendGraph(path, -1, "lgb");
            try {
                // 创建Gurobi环境
//                GRBEnv env = new GRBEnv();
                env = new GRBEnv();
                GRBModel model = new GRBModel(env);

                // 定义常量
                int M = 99999;
                int drone_num = extendGraph.droneNum; // 无人机数量
                int node_num_extend = extendGraph.nodeNumExtend; // 节点数量
                int order_num = extendGraph.orderNum; // 订单数量
                double R = extendGraph.R; // 最大续航里程
                double T = extendGraph.T; // 最大时间
                int[][] c1 = extendGraph.c1; // 飞行通过边的矩阵
                int[][] c2 = extendGraph.c2; // 乘车通过边的矩阵
                double[][] distance_mat_extend = extendGraph.distanceMatExtend; // 距离矩阵
                List<TimeNode> all_node_extend = extendGraph.allNodeExtend;
                double[] alpha_i = new double[node_num_extend]; // 服务时间
                for (int i = 1; i < 1 + order_num * 2; i++) {
                    alpha_i[i] = 1;
                }
                ArrayList<double[]> time_window = extendGraph.timeWindowExtend; // 时间窗
                double[] q_i = new double[node_num_extend]; // 负载
                for (int i = 1; i <= order_num * 2; i++) {
                    if ((i - 1) / order_num == 0) {
                        q_i[i] = 1;
                    }
                    if ((i - 1) / order_num == 1) {
                        q_i[i] = -1;
                    }
                }

                // 定义变量
                GRBVar[][][] x_ij_u = new GRBVar[node_num_extend][node_num_extend][drone_num];
                GRBVar[][][] y_ij_u = new GRBVar[node_num_extend][node_num_extend][drone_num];
                GRBVar[][] a_i_u = new GRBVar[node_num_extend][drone_num];
                GRBVar[][] s_i_u = new GRBVar[node_num_extend][drone_num];
                GRBVar[][] p_i_u = new GRBVar[node_num_extend][drone_num];
                GRBVar[][] g_i_u = new GRBVar[node_num_extend][drone_num];

                // 添加变量
                for (int u = 0; u < drone_num; u++) {
                    for (int i = 0; i < node_num_extend; i++) {
                        for (int j = 0; j < node_num_extend; j++) {
                            if (c1[i][j] == 1) {
                                x_ij_u[i][j][u] = model.addVar(0, 1, 0, GRB.BINARY, "x_" + i + "_" + j + "_" + u);
                            }
                            if (c2[i][j] == 1) {
                                y_ij_u[i][j][u] = model.addVar(0, 1, 0, GRB.BINARY, "y_" + i + "_" + j + "_" + u);
                            }
                        }
                        a_i_u[i][u] = model.addVar(0, T, 0, GRB.CONTINUOUS, "a_" + i + "_" + u);
                        s_i_u[i][u] = model.addVar(0, T, 0, GRB.CONTINUOUS, "s_" + i + "_" + u);
                        p_i_u[i][u] = model.addVar(0, 1, 0, GRB.BINARY, "p_" + i + "_" + u);
                        g_i_u[i][u] = model.addVar(0, R, 0, GRB.CONTINUOUS, "g_" + i + "_" + u);
                    }
                }

                // 设置目标函数
                GRBLinExpr obj = new GRBLinExpr();
                for (int u = 0; u < drone_num; u++) {
                    for (int i = 0; i < node_num_extend - 1; i++) {
                        for (int j = 1; j < node_num_extend; j++) {
                            if (c1[i][j] == 1) {
                                obj.addTerm(distance_mat_extend[i][j], x_ij_u[i][j][u]);
                            }
                            if (c2[i][j] == 1) {
                                obj.addTerm(distance_mat_extend[i][j], y_ij_u[i][j][u]);
                            }
                        }
                    }
                }
                model.setObjective(obj, GRB.MINIMIZE);

                // 添加约束
                // 1. 流量平衡约束
                for (int u = 0; u < drone_num; u++) {
                    for (int i = 1; i < node_num_extend - 1; i++) {
                        GRBLinExpr expr1 = new GRBLinExpr();
                        GRBLinExpr expr2 = new GRBLinExpr();
                        for (int j = 1; j < node_num_extend; j++) {
                            if (c1[i][j] == 1) {
                                expr1.addTerm(1, x_ij_u[i][j][u]);
                            }
                            if (c2[i][j] == 1) {
                                expr1.addTerm(1, y_ij_u[i][j][u]);
                            }
                        }
                        for (int j = 0; j < node_num_extend - 1; j++) {
                            if (c1[j][i] == 1) {
                                expr2.addTerm(1, x_ij_u[j][i][u]);
                            }
                            if (c2[j][i] == 1) {
                                expr2.addTerm(1, y_ij_u[j][i][u]);
                            }
                        }
                        model.addConstr(expr1, GRB.EQUAL, expr2, "flow_balance_" + i + "_" + u);
                    }
                }

                // 2. 客户点必访问
                for (int j = 1; j < order_num * 2 + 1; j++) {
                    GRBLinExpr expr = new GRBLinExpr();
                    for (int u = 0; u < drone_num; u++) {
                        for (int i = 0; i < node_num_extend - 1; i++) {
                            if (c1[i][j] == 1) {
                                expr.addTerm(1, x_ij_u[i][j][u]);
                            }
                        }
                    }
                    model.addConstr(expr, GRB.EQUAL, 1, "visit_" + j);
                }

                // 3. P D 由同一个无人机服务
                for (int u = 0; u < drone_num; u++) {
                    for (int j = 1; j < order_num + 1; j++) {
                        GRBLinExpr expr1 = new GRBLinExpr();
                        GRBLinExpr expr2 = new GRBLinExpr();
                        for (int i = 0; i < node_num_extend - 1; i++) {
                            if (c1[i][j] == 1) {
                                expr1.addTerm(1, x_ij_u[i][j][u]);
                            }
                        }
                        for (int i = 0; i < node_num_extend - 1; i++) {
                            if (c1[i][j + order_num] == 1) {
                                expr2.addTerm(1, x_ij_u[i][j + order_num][u]);
                            }
                        }
                        model.addConstr(expr1, GRB.EQUAL, expr2, "same_drone_" + j + "_" + u);
                    }
                }

                // 4. 点访问次数的约束
                for (int u = 0; u < drone_num; u++) {
                    for (int j = 0; j < node_num_extend - 1; j++) {
                        GRBLinExpr expr = new GRBLinExpr();
                        for (int i = 0; i < node_num_extend - 1; i++) {
                            if (c1[i][j] == 1) {
                                expr.addTerm(1, x_ij_u[i][j][u]);
                            }
                            if (c2[i][j] == 1) {
                                expr.addTerm(1, y_ij_u[i][j][u]);
                            }
                        }
                        model.addConstr(expr, GRB.LESS_EQUAL, 1, "visit_once_" + j + "_" + u);
                    }
                }

                // 5. 起点是0，终点是n+1
                for (int u = 0; u < drone_num; u++) {
                    GRBLinExpr expr1 = new GRBLinExpr();
                    GRBLinExpr expr2 = new GRBLinExpr();
                    for (int j = 1; j < node_num_extend - 1; j++) {
                        if (c1[0][j] == 1) {
                            expr1.addTerm(1, x_ij_u[0][j][u]);
                        }
                    }
                    for (int i = 1; i < node_num_extend - 1; i++) {
                        if (c1[i][node_num_extend - 1] == 1) {
                            expr2.addTerm(1, x_ij_u[i][node_num_extend - 1][u]);
                        }
                    }
                    model.addConstr(expr1, GRB.EQUAL, expr2, "start_end_" + u);
                    model.addConstr(expr1, GRB.LESS_EQUAL, 1, "start_end_limit_" + u);
                }

                // 时间约束
                // 1. 到达时间与prev点的服务时间
                for (int u = 0; u < drone_num; u++) {
                    for (int i = 0; i < node_num_extend - 1; i++) {
                        for (int j = 1; j < node_num_extend; j++) {
                            if (c1[i][j] == 1) {
                                GRBLinExpr expr1 = new GRBLinExpr();
                                expr1.addTerm(1.0, s_i_u[i][u]);
                                expr1.addConstant(distance_mat_extend[i][j] / extendGraph.velD);
                                expr1.addConstant(-M);
                                expr1.addTerm(M, x_ij_u[i][j][u]);
                                expr1.addConstant(alpha_i[i] * extendGraph.stopTime);
                                model.addConstr(a_i_u[j][u], GRB.GREATER_EQUAL, expr1, "arrival_time_1_" + i + "_" + j + "_" + u);

                                GRBLinExpr expr2 = new GRBLinExpr();
                                expr2.addTerm(1.0, s_i_u[i][u]);
                                expr2.addConstant(distance_mat_extend[i][j] / extendGraph.velD);
                                expr2.addConstant(M);
                                expr2.addTerm(-M, x_ij_u[i][j][u]);
                                expr2.addConstant(alpha_i[i] * extendGraph.stopTime);
                                model.addConstr(a_i_u[j][u], GRB.LESS_EQUAL, expr2, "arrival_time_2_" + i + "_" + j + "_" + u);
                            }
                            if (c2[i][j] == 1) {
                                GRBLinExpr expr3 = new GRBLinExpr();
                                expr3.addTerm(1.0, a_i_u[i][u]);
                                expr3.addConstant(-M);
                                expr3.addTerm(M, y_ij_u[i][j][u]);
                                model.addConstr(a_i_u[j][u], GRB.GREATER_EQUAL, expr3, "arrival_time_3_" + i + "_" + j + "_" + u);
                            }
                        }
                    }
                }

                // 2. max
                for (int u = 0; u < drone_num; u++) {
                    for (int i = 1; i < node_num_extend; i++) {
                        GRBVar[] vars = {a_i_u[i][u]};
                        model.addGenConstrMax(s_i_u[i][u], vars, all_node_extend.get(i).arrivalTime, "max_time_" + i + "_" + u);
                    }
                }

                // 3. 点上的时间
                for (int u = 0; u < drone_num; u++) {
                    for (int i = 0; i < node_num_extend - 1; i++) {
                        for (int j = 1; j < node_num_extend; j++) {
                            if (c2[i][j] == 1) {
                                //                            model.addConstr(s_i_u[j][u], GRB.GREATER_EQUAL, all_node_extend.get(j).arrivalTime - M * (1 - y_ij_u[i][j][u]), "node_time_1_" + i + "_" + j + "_" + u);
                                //                            model.addConstr(s_i_u[j][u], GRB.LESS_EQUAL, all_node_extend.get(j).arrivalTime + M * (1 - y_ij_u[i][j][u]), "node_time_2_" + i + "_" + j + "_" + u);

                                GRBLinExpr expr1 = new GRBLinExpr();
                                expr1.addConstant(all_node_extend.get(j).arrivalTime);
                                expr1.addConstant(-M);
                                expr1.addTerm(M, y_ij_u[i][j][u]);
                                model.addConstr(s_i_u[j][u], GRB.GREATER_EQUAL, expr1, "node_time_1_" + i + "_" + j + "_" + u);

                                GRBLinExpr expr2 = new GRBLinExpr();
                                expr2.addConstant(all_node_extend.get(j).arrivalTime);
                                expr2.addConstant(M);
                                expr2.addTerm(-M, y_ij_u[i][j][u]);
                                model.addConstr(s_i_u[j][u], GRB.LESS_EQUAL, expr2, "node_time_2_" + i + "_" + j + "_" + u);
                            }
                        }
                    }
                }

                // 4. PD点 时间窗
                for (int u = 0; u < drone_num; u++) {
                    for (int j = 1; j < order_num * 2 + 1; j++) {

                        //                    model.addConstr(s_i_u[j][u], GRB.GREATER_EQUAL, time_window.get(j - 1)[0] - M * (1 - expr), "time_window_1_" + j + "_" + u);
                        //                    model.addConstr(s_i_u[j][u], GRB.LESS_EQUAL, time_window.get(j - 1)[1] + M * (1 - expr), "time_window_2_" + j + "_" + u);

                        GRBLinExpr expr1 = new GRBLinExpr();
                        expr1.addConstant(time_window.get(j - 1)[0]);
                        expr1.addConstant(-M);
                        for (int i = 0; i < node_num_extend - 1; i++) {
                            if (c1[i][j] == 1) {
                                expr1.addTerm(M, x_ij_u[i][j][u]);
                            }
                        }
                        model.addConstr(s_i_u[j][u], GRB.GREATER_EQUAL, expr1, "time_window_1_" + j + "_" + u);

                        GRBLinExpr expr2 = new GRBLinExpr();
                        expr2.addConstant(time_window.get(j - 1)[1]);
                        expr2.addConstant(M);
                        for (int i = 0; i < node_num_extend - 1; i++) {
                            if (c1[i][j] == 1) {
                                expr2.addTerm(-M, x_ij_u[i][j][u]);
                            }
                        }
                        model.addConstr(s_i_u[j][u], GRB.LESS_EQUAL, expr2, "time_window_2_" + j + "_" + u);
                    }
                }

                // 5. 初始点
                for (int u = 0; u < drone_num; u++) {
                    model.addConstr(a_i_u[0][u], GRB.EQUAL, 0, "initial_arrival_" + u);
                    model.addConstr(s_i_u[0][u], GRB.EQUAL, 0, "initial_service_" + u);
                }

                // 续航约束
                // 1. 初始续航
                for (int u = 0; u < drone_num; u++) {
                    model.addConstr(g_i_u[0][u], GRB.EQUAL, R, "initial_range_" + u);
                }

                // 2. 飞行通过边
                for (int u = 0; u < drone_num; u++) {
                    for (int i = 0; i < node_num_extend - 1; i++) {
                        for (int j = 1; j < node_num_extend; j++) {
                            if (c1[i][j] == 1) {
                                //                            model.addConstr(g_i_u[j][u], GRB.GREATER_EQUAL, g_i_u[i][u] - distance_mat_extend[i][j] - M * (1 - x_ij_u[i][j][u]), "range_1_" + i + "_" + j + "_" + u);
                                //                            model.addConstr(g_i_u[j][u], GRB.LESS_EQUAL, g_i_u[i][u] - distance_mat_extend[i][j] + M * (1 - x_ij_u[i][j][u]), "range_2_" + i + "_" + j + "_" + u);

                                GRBLinExpr expr1 = new GRBLinExpr();
                                expr1.addTerm(1.0, g_i_u[i][u]);
                                expr1.addConstant(-distance_mat_extend[i][j]);
                                expr1.addConstant(-M);
                                expr1.addTerm(M, x_ij_u[i][j][u]);
                                model.addConstr(g_i_u[j][u], GRB.GREATER_EQUAL, expr1, "range_1_" + i + "_" + j + "_" + u);

                                GRBLinExpr expr2 = new GRBLinExpr();
                                expr2.addTerm(1.0, g_i_u[i][u]);
                                expr2.addConstant(-distance_mat_extend[i][j]);
                                expr2.addConstant(M);
                                expr2.addTerm(-M, x_ij_u[i][j][u]);
                                model.addConstr(g_i_u[j][u], GRB.LESS_EQUAL, expr2, "range_2_" + i + "_" + j + "_" + u);
                            }
                            if (c2[i][j] == 1) {
                                //                            model.addConstr(g_i_u[j][u], GRB.GREATER_EQUAL, R - M * (1 - y_ij_u[i][j][u]), "range_3_" + i + "_" + j + "_" + u);
                                //                            model.addConstr(g_i_u[j][u], GRB.LESS_EQUAL, R + M * (1 - y_ij_u[i][j][u]), "range_4_" + i + "_" + j + "_" + u);

                                GRBLinExpr expr3 = new GRBLinExpr();
                                expr3.addConstant(R);
                                expr3.addConstant(-M);
                                expr3.addTerm(M, y_ij_u[i][j][u]);
                                model.addConstr(g_i_u[j][u], GRB.GREATER_EQUAL, expr3, "range_3_" + i + "_" + j + "_" + u);

                                GRBLinExpr expr4 = new GRBLinExpr();
                                expr4.addConstant(R);
                                expr4.addConstant(M);
                                expr4.addTerm(-M, y_ij_u[i][j][u]);
                                model.addConstr(g_i_u[j][u], GRB.LESS_EQUAL, expr4, "range_4_" + i + "_" + j + "_" + u);
                            }
                        }
                    }
                }
                //
                // Payload约束
                // 1. 初始
                for (int u = 0; u < drone_num; u++) {
                    model.addConstr(p_i_u[0][u], GRB.EQUAL, 0, "initial_payload_" + u);
                }

                // 2. 通过边
                for (int u = 0; u < drone_num; u++) {
                    for (int i = 0; i < node_num_extend - 1; i++) {
                        for (int j = 1; j < node_num_extend; j++) {
                            if (c1[i][j] == 1) {
                                //                            model.addConstr(p_i_u[j][u], GRB.GREATER_EQUAL, p_i_u[i][u] + q_i[j] - M * (1 - x_ij_u[i][j][u]), "payload_1_" + i + "_" + j + "_" + u);
                                //                            model.addConstr(p_i_u[j][u], GRB.LESS_EQUAL, p_i_u[i][u] + q_i[j] + M * (1 - x_ij_u[i][j][u]), "payload_2_" + i + "_" + j + "_" + u);

                                GRBLinExpr expr1 = new GRBLinExpr();
                                expr1.addTerm(1.0, p_i_u[i][u]);
                                expr1.addConstant(q_i[j]);
                                expr1.addConstant(-M);
                                expr1.addTerm(M, x_ij_u[i][j][u]);
                                model.addConstr(p_i_u[j][u], GRB.GREATER_EQUAL, expr1, "payload_1_" + i + "_" + j + "_" + u);

                                GRBLinExpr expr2 = new GRBLinExpr();
                                expr2.addTerm(1.0, p_i_u[i][u]);
                                expr2.addConstant(q_i[j]);
                                expr2.addConstant(M);
                                expr2.addTerm(-M, x_ij_u[i][j][u]);
                                model.addConstr(p_i_u[j][u], GRB.LESS_EQUAL, expr2, "payload_2_" + i + "_" + j + "_" + u);
                            }
                            if (c2[i][j] == 1) {
                                //                            model.addConstr(p_i_u[j][u], GRB.GREATER_EQUAL, p_i_u[i][u] + q_i[j] - M * (1 - y_ij_u[i][j][u]), "payload_3_" + i + "_" + j + "_" + u);
                                //                            model.addConstr(p_i_u[j][u], GRB.LESS_EQUAL, p_i_u[i][u] + q_i[j] + M * (1 - y_ij_u[i][j][u]), "payload_4_" + i + "_" + j + "_" + u);

                                GRBLinExpr expr3 = new GRBLinExpr();
                                expr3.addTerm(1.0, p_i_u[i][u]);
                                expr3.addConstant(q_i[j]);
                                expr3.addConstant(-M);
                                expr3.addTerm(M, y_ij_u[i][j][u]);
                                model.addConstr(p_i_u[j][u], GRB.GREATER_EQUAL, expr3, "payload_3_" + i + "_" + j + "_" + u);

                                GRBLinExpr expr4 = new GRBLinExpr();
                                expr4.addTerm(1.0, p_i_u[i][u]);
                                expr4.addConstant(q_i[j]);
                                expr4.addConstant(M);
                                expr4.addTerm(-M, y_ij_u[i][j][u]);
                                model.addConstr(p_i_u[j][u], GRB.LESS_EQUAL, expr4, "payload_4_" + i + "_" + j + "_" + u);
                            }
                        }
                    }
                }


                // 禁用求解过程输出
                model.set(GRB.IntParam.OutputFlag, 0);
                // 设置求解时间限制为 300 秒
                model.set(GRB.DoubleParam.TimeLimit, 300);


                double start = System.currentTimeMillis();
                // 优化模型
                model.optimize();
                System.out.print("Time: " + (System.currentTimeMillis() - start) / 1000.0);

                // 输出结果
                System.out.println("Obj: " + model.get(GRB.DoubleAttr.ObjVal));

                // 释放资源
                model.dispose();
                env.dispose();

            } catch (GRBException e) {
                System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
            }

        }
    }
}
