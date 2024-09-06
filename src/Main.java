import Algorithm.Labeling.SPPRC;
import Parameters.Parameters;
import VRP.Route;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

/**
 * @author Yu Mingzheng
 * @date 2024/8/29 15:26
 * @description
 */
public class Main {
    public static void main(String[] args) {
        double start = System.currentTimeMillis();

        Parameters parameters = new Parameters("C:\\Users\\31706\\Desktop\\exact-algorithm\\instances\\solomon_25\\c106.txt");

        ArrayList<Route> bestRoutes = new ArrayList<>();

        Map<Integer, Double> dualPrices = new HashMap<>(Parameters.numClient);
        for (int i = 1; i <= Parameters.numClient; i++)
            dualPrices.put(i, 7d);


        SPPRC spprc = new SPPRC(parameters);
        spprc.solve(dualPrices , bestRoutes);


        System.out.println("Time consumption: " + (System.currentTimeMillis() - start) / 1000.0);
        System.out.println("Path: " + bestRoutes);
        System.out.println("Path size: " + bestRoutes.size());

    }
}
