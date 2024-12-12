import Algorithm.BranchAndPirce.MyColumnGen;
import Parameters.ExtendGraph;
import Problem.Route;
import ilog.concert.IloException;
import org.json.simple.parser.ParseException;

import javax.script.ScriptException;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;

/**
 * @author Yu Mingzheng
 * @date 2024/11/13 14:41
 * @description
 */
public class Test {
    public static void main(String[] args) throws ScriptException, IOException, ParseException, IloException {
        ExtendGraph extendGraph = new ExtendGraph();
        ArrayList<Route> bestRoutes = new ArrayList<>();
        MyColumnGen cg = new MyColumnGen();
        double obj = cg.computeColGen(extendGraph , bestRoutes);
        System.out.println("Obj : " + obj);

        for (int i = 0; i < bestRoutes.size(); i++)
            if(bestRoutes.get(i).getQ() != 0)
                System.out.println("Q : " + bestRoutes.get(i).getQ() + " " +  bestRoutes.get(i));
        System.out.println("Path size: " + bestRoutes.size());

        for (int i = 0; i < extendGraph.orderNum; i++) {
            Arrays.fill(extendGraph.twoOrder[i] , 0);
        }
        for(Route r : bestRoutes){
            int[] throughOrder = r.throughOrder.stream().mapToInt(k->k).toArray();
            for (int k = 0; k < throughOrder.length; k++) {
                for (int l = k+1; l < throughOrder.length; l++) {
                    extendGraph.twoOrder[throughOrder[k]-1][throughOrder[l]-1]+=r.getQ();
                }
            }
        }
        for (int i = 0; i < extendGraph.twoOrder.length; i++) {
            for (int j = i+1; j < extendGraph.twoOrder.length; j++) {
                if(extendGraph.twoOrder[i][j]>0.5) extendGraph.twoOrder[i][j]-=0.5;
                else extendGraph.twoOrder[i][j]=0.5-extendGraph.twoOrder[i][j];
            }
        }
        double minValue = extendGraph.twoOrder[0][1];
        int row=0,column=1;
        for(int i=0;i<extendGraph.twoOrder.length;i++){
            for(int j=i+1;j<extendGraph.twoOrder.length;j++){
                if(extendGraph.twoOrder[i][j]<minValue){
                    minValue=extendGraph.twoOrder[i][j];
                    row=i;
                    column=j;
                }
            }
        }
        System.out.println("该二维数组中的最大值是"+minValue);
        System.out.println("最大值的在数组中的下标是:"+row+" "+column);

        int i =0;
    }
}
