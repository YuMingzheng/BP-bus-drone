package Old;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;

/**
 * @author Yu Mingzheng
 * @date 2024/8/29 15:26
 * @description
 */
public class Parameters {
    public int numVehicle;
    static public int numClient;
    public int vehCapacity;
    public double[][] cost;
    public double[][] distBase;
    public double[][] dist;
    public double[][] time;
    public double[][] edges;
    public double[] posX , posY , demand ;
    public int[] a , b , s;
    public static double bigM  = 1E5;;
    public double speed;
    public static double gap = 1e-5;
    public static double EPS = 1e-5;
    public static double maxRange = 100; // 无人机满电下的飞行里程
    public double maxLength;
    String[] nodeId;

    public Parameters(String path){
        speed = 1;

        initParams(path);
    }

    public void initParams(String path){
        int i ,j;
        numClient = Integer.parseInt(path.split("\\\\")[6].split("_")[1]);
        try{
            BufferedReader br = new BufferedReader(new FileReader(path));
            String line = "";
            for (i = 0; i < 5; i++) {
                line = br.readLine();
            }
            String[] tokens = line.split("\\s+");
            numVehicle = Integer.parseInt(tokens[1]);
            vehCapacity= Integer.parseInt(tokens[2]);

            nodeId = new String[numClient+2];
            demand = new double[numClient+2];
            a = new int[numClient+2];
            b = new int[numClient+2];
            s = new int[numClient+2];
            posX = new double[numClient+2];
            posY = new double[numClient+2];
            distBase = new double[numClient+2][numClient+2];
            cost = new double[numClient+2][numClient+2];
            dist = new double[numClient+2][numClient+2];
            time = new double[numClient+2][numClient+2];

            for(i = 0 ; i < 4 ; i++)
                br.readLine();

            for(i = 0; i < numClient+1; i++){
                line = br.readLine();
                tokens = line.split("\\s+");
                nodeId[i] = tokens[1];
                posX[i] = Double.parseDouble(tokens[2]);
                posY[i] = Double.parseDouble(tokens[3]);
                demand[i] = Double.parseDouble(tokens[4]);
                a[i] = Integer.parseInt(tokens[5]);
                b[i] = Integer.parseInt(tokens[6]);
                s[i] = Integer.parseInt(tokens[7]);
            }
            br.close();

            nodeId[numClient+1] = nodeId[0];
            demand[numClient+1] = 0.0;
            a[numClient + 1] = a[0];
            b[numClient + 1] = b[0];
            s[numClient + 1] = 0;
            posX[numClient+1] = posX[0];
            posY[numClient+1] = posY[0];

            // 计算dist矩阵
            double max;
            maxLength = 0f;
            for(i = 0;i < numClient+2;i++){
                max = 0f;
                for(j = 0 ; j < numClient+2; j++){
                    distBase[i][j] = ((int)(10 * Math.sqrt(
                      Math.pow(posX[i] - posX[j] , 2) +
                      Math.pow(posY[i] - posY[j] , 2)
                    ))) / 10.0;
                    if (max < distBase[i][j]) max=distBase[i][j];
                }
                maxLength+=max;
            }
            for (i = 0; i < numClient + 2; i++) {
                distBase[i][0] = bigM;              // 所有点到0的距离均为M
                distBase[numClient + 1][i] = bigM;  // n+1到所有点的距离均为M
                distBase[i][i] = bigM;              // i点到i点（对角线）的距离均为M
            }

            for (i = 0; i < numClient + 2; i++)
                for (j = 0; j < numClient + 2; j++)
                    dist[i][j] = distBase[i][j];

            for(i = 0 ; i < numClient+2;i++)
                for(j=0 ; j < numClient+2;j++)
                    time[i][j] = distBase[i][j] / speed;

        }catch (IOException e){
            System.out.println("Error : " + e);
        }

        edges = new double[numClient+2][numClient+2];
    }
}
