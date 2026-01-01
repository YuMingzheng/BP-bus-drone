package IO;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * @author Yu Mingzheng
 * @date 2024/12/22 17:00
 * @description
 */
public class IO {
    // 将结果写入指定文件（支持追加或者新建文件）
    public static void writeToFile(Object data, String filePath, boolean append) {
        try {
            createFileIfNotExists(filePath);

            try (BufferedWriter writer = new BufferedWriter(new FileWriter(filePath, append))) {
                if (data instanceof int[]) {
                    int[] array = (int[]) data;
                    for (int i = 0; i < array.length; i++) {
                        writer.write(array[i] + (i < array.length - 1 ? ", " : ""));
                    }
                } else if (data instanceof int[][]) {
                    int[][] array2D = (int[][]) data;
                    for (int[] ints : array2D) {
                        for (int j = 0; j < ints.length; j++) {
                            writer.write(ints[j] + (j < ints.length - 1 ? ", " : ""));
                        }
                        writer.newLine();
                    }
                }else if (data instanceof String[]) {
                    String[] array = (String[]) data;
                    for (int i = 0; i < array.length; i++) {
                        writer.write(array[i] + (i < array.length - 1 ? ", " : ""));
                    }
                } else if (data instanceof ArrayList) {
                    if(((ArrayList<?>) data).get(0) instanceof Integer){
                        ArrayList<?> list = (ArrayList<?>) data;
                        for (Object item : list) {
                            writer.write(item.toString() + ", ");
                        }
                    }else if(((ArrayList<?>) data).get(0) instanceof ArrayList){
                        ArrayList<?> list = (ArrayList<?>) data;
                        for (Object item : list) {
                            ArrayList<?> list2 = (ArrayList<?>) item;
                            for(Object iterm2 : list2){
                                writer.write(iterm2.toString() + ", ");
                            }
                            writer.newLine();  // 写入换行符
                        }
                    }
                }
                else {
                    writer.write(data.toString());
                }
                writer.newLine();  // 写入换行符
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    // 确保文件的路径存在，如果路径不存在则创建
    private static void createFileIfNotExists(String filePath) throws IOException {
        File file = new File(filePath);
        File parentDir = file.getParentFile();

        if (parentDir != null && !parentDir.exists()) {
            parentDir.mkdirs();  // 创建目录
        }

        if (!file.exists()) {
            file.createNewFile();  // 创建文件
        }
    }


    public static List<File> getFilesInDirectory(String directoryPath) {
        File directory = new File(directoryPath);
        List<File> fileList = new ArrayList<>();

        if (directory.exists() && directory.isDirectory()) {
            File[] files = directory.listFiles();
            if (files != null) {
                for (File file : files) {
                    if (file.isFile()) {
                        fileList.add(file);  // 只加入文件，不包括目录
                    }
                }
            }
        } else {
            System.out.println("路径无效或不是目录");
        }

        return fileList;
    }

    public static void main(String[] args) {
        List<File> files = IO.getFilesInDirectory("D:\\硕士毕设\\bus+drone\\bus-drone-code\\data\\threshold_instance");
        int i = 0 ;
    }

}