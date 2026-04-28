import fr.emse.fayol.maqit.simulator.configuration.IniFile;
import fr.emse.fayol.maqit.simulator.configuration.SimProperties;
import simulator.MySimFactory;

import java.util.Random;
import java.io.FileWriter;
import java.io.PrintWriter;
import java.io.IOException;
import java.util.concurrent.*;

public class Main {
    // ------------------------------------------------------------------ //
    //  Point d'entrée principal
    // ------------------------------------------------------------------ //
    public static void main(String[] args) throws Exception {

        // Arguments :
        // - fastMode : si true, pas d'affichage graphique et les robots ne font pas de pause entre les étapes
        // - seed : graine pour la génération aléatoire (obstacles, positions, etc.)
        //
        boolean fastMode = false;
        boolean testMode = false;
        int seed = 0; // par défaut, graine aléatoire
        for (String arg : args) {
            if (arg.equalsIgnoreCase("fastMode=true")) {
                fastMode = true;
            } else if (arg.equalsIgnoreCase("testMode=true")) {
                testMode = true;
                fastMode = true; // forcer le mode rapide en test
            } else if (arg.startsWith("seed=")) {
                try {
                    seed = Integer.parseInt(arg.substring(5));
                } catch (NumberFormatException e) {
                    System.err.println("Invalid seed value, using current time as seed.");
                }
            }
        }
        if (testMode) {
            // Paramètres à tester (exemple : nombre de robots et d'obstacles)
            int[] robotsRange = {6, 8, 10, 12}; // à adapter selon besoin
            int[] obstaclesRange = {2, 4, 6, 8};
            int seedsPerCombo = 100;
            double[][][] results = new double[robotsRange.length][obstaclesRange.length][2]; // [efficacité, durée]
            ExecutorService executor = Executors.newFixedThreadPool(Runtime.getRuntime().availableProcessors());
            try {
                for (int i = 0; i < robotsRange.length; i++) {
                    for (int j = 0; j < obstaclesRange.length; j++) {
                        int robots = robotsRange[i];
                        int obstacles = obstaclesRange[j];
                        java.util.List<Future<double[]>> futures = new java.util.ArrayList<>();
                        for (int s = 0; s < seedsPerCombo; s++) {
                            final int testSeed = (int) (System.currentTimeMillis() + s * 1000 + i * 100 + j * 10);
                            futures.add(executor.submit(() -> {
                                IniFile ifile    = new IniFile("parameters/configuration.ini");
                                IniFile ifileEnv = new IniFile("parameters/environment.ini");
                                SimProperties sp = new SimProperties(ifile);
                                sp.simulationParams();
                                sp.displayParams();
                                SimProperties envProp = new SimProperties(ifileEnv);
                                envProp.loadObstaclePositions();
                                envProp.loadStartZonePositions();
                                envProp.loadTransitZones();
                                envProp.loadExitZonePositions();
                                envProp.loadGoalPositions();
                                sp.obstaclePositions  = envProp.obstaclePositions;
                                sp.startZonePositions = envProp.startZonePositions;
                                sp.transitZoneData    = envProp.transitZoneData;
                                sp.exitZonePositions  = envProp.exitZonePositions;
                                sp.goalPositions      = envProp.goalPositions;
                                sp.nbrobot = robots;
                                sp.nbobstacle = obstacles;
                                sp.seed = testSeed;
                                MySimFactory sim = new MySimFactory(sp);
                                sim.setNbPackages(sp.nbrobot * 3);
                                sim.numberOfWorkers = sp.nbobstacle / 2;
                                sim.rnd = new Random(sp.seed);
                                sim.fastMode = true;
                                sim.testMode = true;
                                sim.createEnvironment();
                                sim.createObstacle();
                                sim.createGoal();
                                sim.createStartZones();
                                sim.createTransitZones();
                                sim.createExitZones();
                                sim.createWorker();
                                sim.createRobot();
                                sim.loadRechargeZones(ifileEnv);
                                long start = System.currentTimeMillis();
                                sim.schedule();
                                long duration = System.currentTimeMillis() - start;
                                int delivered = sim.getDeliveredCount();
                                int steps = sim.getTotalSteps();
                                double efficiency = (steps > 0) ? ((double) delivered / steps) : 0.0;
                                return new double[]{efficiency, duration / 1000.0};
                            }));
                        }
                        double sumEfficiency = 0;
                        double sumDuration = 0;
                        for (Future<double[]> f : futures) {
                            try {
                                double[] res = f.get();
                                sumEfficiency += res[0];
                                sumDuration += res[1];
                            } catch (Exception e) {
                                System.err.println("Erreur dans un test : " + e.getMessage());
                            }
                        }
                        results[i][j][0] = sumEfficiency / seedsPerCombo;
                        results[i][j][1] = sumDuration / seedsPerCombo;
                    }
                }
            } finally {
                executor.shutdown();
                executor.awaitTermination(1, TimeUnit.HOURS);
            }
            // Affichage du tableau de résultats
            System.out.println("\nRésultats des tests d'efficacité (efficacité = paquets/étape, durée en s) :");
            for (int i = 0; i < robotsRange.length; i++) {
                for (int j = 0; j < obstaclesRange.length; j++) {
                    System.out.printf("Robots: %d, Obstacles: %d => Efficacité: %.3f, Durée: %.2f s\n",
                        robotsRange[i], obstaclesRange[j], results[i][j][0], results[i][j][1]);
                }
            }
            // Export CSV
            try (PrintWriter writer = new PrintWriter(new FileWriter("resultats_tests.csv"))) {
                writer.println("robots,obstacles,efficacite,duree_s");
                for (int i = 0; i < robotsRange.length; i++) {
                    for (int j = 0; j < obstaclesRange.length; j++) {
                        writer.printf("%d,%d,%.3f,%.2f\n",
                            robotsRange[i], obstaclesRange[j], results[i][j][0], results[i][j][1]);
                    }
                }
                System.out.println("\nFichier CSV généré : resultats_tests.csv");
            } catch (IOException e) {
                System.err.println("Erreur lors de l'écriture du CSV : " + e.getMessage());
            }
            return;
        }
        IniFile ifile    = new IniFile("parameters/configuration.ini");
        IniFile ifileEnv = new IniFile("parameters/environment.ini");

        SimProperties sp = new SimProperties(ifile);
        sp.simulationParams();
        sp.displayParams();

        SimProperties envProp = new SimProperties(ifileEnv);
        envProp.loadObstaclePositions();
        envProp.loadStartZonePositions();
        envProp.loadTransitZones();
        envProp.loadExitZonePositions();
        envProp.loadGoalPositions();

        sp.obstaclePositions  = envProp.obstaclePositions;
        sp.startZonePositions = envProp.startZonePositions;
        sp.transitZoneData    = envProp.transitZoneData;
        sp.exitZonePositions  = envProp.exitZonePositions;
        sp.goalPositions      = envProp.goalPositions;

        sp.seed = seed;

        System.out.println("Grille : " + sp.rows + "x" + sp.columns);
        System.out.println("Robots : " + sp.nbrobot);

        MySimFactory sim = new MySimFactory(sp);
        sim.setNbPackages(sp.nbrobot * 3);   // 3× le nb de robots
        sim.numberOfWorkers       = sp.nbobstacle / 2;
        sim.rnd                   = new Random(sp.seed);

        sim.fastMode = fastMode;


        sim.createEnvironment();
        sim.createObstacle();
        sim.createGoal();
        sim.createStartZones();
        sim.createTransitZones();
        sim.createExitZones();
        sim.createWorker();
        sim.createRobot();

        sim.loadRechargeZones(ifileEnv);

        if (!sim.fastMode) {
            sim.initializeGW();
        }
        sim.schedule();
    }
}
