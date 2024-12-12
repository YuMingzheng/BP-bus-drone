import Algorithm.BranchAndPirce.MyBranchAndBound;
import Parameters.ExtendGraph;
import Problem.Route;
import ilog.concert.IloException;
import org.json.simple.parser.ParseException;

import javax.script.ScriptException;
import java.io.IOException;
import java.util.ArrayList;

/**
 * @author Yu Mingzheng
 * @date 2024/11/11 14:26
 * @description
 */
public class MainBP {
    public static void main(String[] args) throws ScriptException, IOException, ParseException, IloException {
        double start = System.currentTimeMillis();

        MyBranchAndBound bp = new MyBranchAndBound();
        ExtendGraph extendGraph = new ExtendGraph();

        ArrayList<Route> initRoutes = new ArrayList<>();
        ArrayList<Route> bestRoutes = new ArrayList<>();

        bp.BBNode(extendGraph, initRoutes, null, bestRoutes	, 0);


        double optCost = 0;
        System.out.println();
        System.out.println("solution >>>");
        int nv = 0;
        for (Route bestRoute : bestRoutes) {
            System.out.println(bestRoute.distance + "  " + bestRoute.path);
            optCost += bestRoute.distance;
            nv++;
        }

        System.out.println("\nbest Cost = " + optCost);
        System.out.println("\nnumber of vhc = " + nv);

        double end = System.currentTimeMillis();
        System.out.println("\ntime " + (end-start)/1000.0);

    }
}
