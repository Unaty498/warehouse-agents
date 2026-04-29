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
        // - testMode : si true, exécute une série de tests d'efficacité pour différentes configurations de robots et d'obstacles
        boolean fastMode = false;
        boolean testMode = false;
        for (String arg : args) {
            if (arg.equalsIgnoreCase("fastMode=true")) {
                fastMode = true;
            } else if (arg.equalsIgnoreCase("testMode=true")) {
                testMode = true;
                fastMode = true; // forcer le mode rapide en test
            }
        }
        if (testMode) {
            int[] robotsRange = {6, 8, 10, 12};
            int[] obstaclesRange = {1, 3, 5, 7};
            int seedsPerCombo = 20;
            double[][][] results = new double[robotsRange.length][obstaclesRange.length][6];

            for (int i = 0; i < robotsRange.length; i++) {
                for (int j = 0; j < obstaclesRange.length; j++) {
                    int robots = robotsRange[i];
                    int obstacles = obstaclesRange[j];

                    double sumEfficiency = 0;
                    double sumDuration = 0;
                    double sumIdleRatio = 0;
                    double sumRechargeUtil = 0;
                    double sumSteps = 0;
                    double sumSingleRobotEfficiency = 0;

                    System.out.printf("\n[Test] Robots: %d, Obstacles: %d (%d graines)\n", robots, obstacles, seedsPerCombo);

                    for (int s = 0; s < seedsPerCombo; s++) {
                        final int testSeed = (int) (System.currentTimeMillis() + s * 1000 + i * 100 + j * 10);

                        try {
                            IniFile ifile    = new IniFile("parameters/configuration.ini");
                            IniFile ifileEnv = new IniFile("parameters/environment.ini");
                            SimProperties sp = new SimProperties(ifile);
                            sp.simulationParams();
                            // sp.displayParams(); // Optionnel : encombre la console en test séquentiel
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

                            if (sp.colorobstacle == null) sp.colorobstacle = java.awt.Color.DARK_GRAY;
                            if (sp.colorgoal == null) sp.colorgoal = java.awt.Color.YELLOW;
                            if (sp.colorpackage == null) sp.colorpackage = java.awt.Color.ORANGE;
                            if (sp.colorstartzone == null) sp.colorstartzone = java.awt.Color.CYAN;
                            if (sp.colortransitzone == null) sp.colortransitzone = java.awt.Color.LIGHT_GRAY;
                            if (sp.colorexit == null) sp.colorexit = java.awt.Color.GREEN;
                            if (sp.colorrobot == null) sp.colorrobot = java.awt.Color.RED;
                            if (sp.colorother == null) sp.colorother = java.awt.Color.MAGENTA;
                            MySimFactory sim = new MySimFactory(sp);
                            sim.setNbPackages(20);
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

                            double[] stats = sim.getStats();

                            sumEfficiency += stats[3];
                            sumSingleRobotEfficiency += stats[4];
                            sumDuration += (duration / 1000.0);
                            sumIdleRatio += stats[1];
                            sumRechargeUtil += stats[0];
                            sumSteps += sim.getTotalSteps();

                        } catch (Exception e) {
                            System.err.println("Erreur dans un test : " + e.getMessage());
                        }
                    }

                    results[i][j][0] = sumEfficiency / seedsPerCombo;
                    results[i][j][1] = sumDuration / seedsPerCombo;
                    results[i][j][2] = sumIdleRatio / seedsPerCombo;
                    results[i][j][3] = sumRechargeUtil / seedsPerCombo;
                    results[i][j][4] = sumSteps / seedsPerCombo;
                    results[i][j][5] = sumSingleRobotEfficiency / seedsPerCombo;
                }
            }

            // --- Affichage et Export CSV ---
            System.out.println("\nRésultats des tests d'efficacité (efficacité = paquets/étape, durée en s) :");
            for (int i = 0; i < robotsRange.length; i++) {
                for (int j = 0; j < obstaclesRange.length; j++) {
                    System.out.printf("Robots: %d, Obstacles: %d => Efficacité: %.3f, Durée: %.2f s\n",
                            robotsRange[i], obstaclesRange[j], results[i][j][0], results[i][j][1]);
                }
            }

            try (PrintWriter writer = new PrintWriter(new FileWriter("resultats_tests.csv"))) {
                writer.println("robots;obstacles;efficacite;efficacite_unitaire;total_steps;duree_s;idle_ratio;recharge_utilization");
                for (int i = 0; i < robotsRange.length; i++) {
                    for (int j = 0; j < obstaclesRange.length; j++) {
                        writer.printf("%d;%d;%.3f;%.3f;%.2f;%.2f;%.4f;%.4f\n",
                                robotsRange[i], obstaclesRange[j], results[i][j][5], results[i][j][0], results[i][j][4],results[i][j][1], results[i][j][2], results[i][j][3]);
                    }
                }
                System.out.println("\nFichier CSV : resultats_tests.csv");
            } catch (IOException e) {
                System.err.println("Erreur lors de l'écriture du CSV : " + e.getMessage());
            }
            return;
        }

        // Mode normal

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
        
        if (sp.colorobstacle == null) sp.colorobstacle = java.awt.Color.DARK_GRAY;
        if (sp.colorgoal == null) sp.colorgoal = java.awt.Color.YELLOW;
        if (sp.colorpackage == null) sp.colorpackage = java.awt.Color.ORANGE;
        if (sp.colorstartzone == null) sp.colorstartzone = java.awt.Color.CYAN;
        if (sp.colortransitzone == null) sp.colortransitzone = java.awt.Color.LIGHT_GRAY;
        if (sp.colorexit == null) sp.colorexit = java.awt.Color.GREEN;
        if (sp.colorrobot == null) sp.colorrobot = java.awt.Color.RED;
        if (sp.colorother == null) sp.colorother = java.awt.Color.MAGENTA;

        System.out.println("Grille : " + sp.rows + "x" + sp.columns);
        System.out.println("Robots : " + sp.nbrobot);

        MySimFactory sim = new MySimFactory(sp);
        sim.setNbPackages(20);   // 3× le nb de robots
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
