package simulator;

import fr.emse.fayol.maqit.simulator.components.*;
import fr.emse.fayol.maqit.simulator.configuration.IniFile;
import fr.emse.fayol.maqit.simulator.configuration.SimProperties;

import java.awt.Color;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

import fr.emse.fayol.maqit.simulator.environment.Cell;
import fr.emse.fayol.maqit.simulator.environment.ColorCell;
import fr.emse.fayol.maqit.simulator.environment.ColorGoal;
import fr.emse.fayol.maqit.simulator.environment.ColorGridEnvironment;

/**
 * Cette classe permet de realiser la simulation
 */
public class MySimFactory extends SimFactory {

    // Pour le suivi de l’occupation des stations de recharge
    private int[] rechargeOccupiedSteps = null; // taille = nb de stations
    private int rechargeTotalSteps = 0;

    private final Map<String, ColorStartZone>   startZonesMap  = new HashMap<>();
    private final Map<String, ColorTransitZone> transitZonesMap = new HashMap<>();

    /** Positions des stations de recharge chargées depuis environment.ini */
    private final List<int[]> rechargePositions = new ArrayList<>();

    private int deliveredCount = 0;

    /** Compteur de livraisons par goal (goalId → nb livrés). */
    private final Map<Integer, Integer> deliveredPerGoal = new HashMap<>();

    int nbPackages;
    int nbNotGeneratedPackages;
    public int numberOfWorkers;
    public Random rnd;
    int totalSteps    = 0;
    /** Nombre total de colis injectés dans les zones de départ depuis le début. */
    private int totalGenerated = 0;
    private long simStartTime;
    public boolean fastMode = false;
    public boolean testMode = false;

    // ── Constantes ANSI ──────────────────────────────────────────────── //
    private static final String RESET  = "\033[0m";
    private static final String BOLD   = "\033[1m";
    private static final String GREEN  = "\033[32m";
    private static final String YELLOW = "\033[33m";
    private static final String RED    = "\033[31m";
    private static final String CYAN   = "\033[36m";
    private static final String BLUE   = "\033[34m";

    public MySimFactory(SimProperties sp) {
        super(sp);
    }


    public void setNbPackages(int nbPackages) {
        this.nbPackages = nbPackages;
        this.nbNotGeneratedPackages = nbPackages;
        this.deliveredCount = 0;
        this.deliveredPerGoal.clear();
    }
    public int getDeliveredCount() {
        return deliveredCount;
    }

    public Map<Integer, Integer> getDeliveredPerGoal() {
        return deliveredPerGoal;
    }

    public int getNbPackages() {
        return nbPackages;
    }

    public int getTotalSteps() {
        return totalSteps;
    }

    // ------------------------------------------------------------------ //
    //  Création de l'environnement
    // ------------------------------------------------------------------ //
    @Override
    public void createEnvironment() {
        environment = new ColorGridEnvironment(sp.rows, sp.columns, sp.debug, sp.seed);
        environment.initializeGrid();
    }

    // ------------------------------------------------------------------ //
    //  Obstacles statiques
    // ------------------------------------------------------------------ //
    @Override
    public void createObstacle() {
        int maxObstacles = Math.min(sp.nbobstacle, sp.obstaclePositions.length);
        for (int i = 0; i < maxObstacles; i++) {
            int[] pos = sp.obstaclePositions[i];
            ColorObstacle obstacle = new ColorObstacle(pos, new int[]{
                sp.colorobstacle.getRed(),
                sp.colorobstacle.getGreen(),
                sp.colorobstacle.getBlue()
            });
            addNewComponent(obstacle);
        }
    }

    // ------------------------------------------------------------------ //
    //  Goals visuels (marqueurs dans les cellules)
    // ------------------------------------------------------------------ //
    @Override
    public void createGoal() {
        int[] z1Pos = sp.goalPositions.get(1);
        int[] z2Pos = sp.goalPositions.get(2);
        ((ColorCell) environment.getGrid()[z1Pos[0]][z1Pos[1]])
            .setGoal(new ColorGoal(1, new int[]{
                sp.colorgoal.getRed(), sp.colorgoal.getGreen(), sp.colorgoal.getBlue()
            }));
        ((ColorCell) environment.getGrid()[z2Pos[0]][z2Pos[1]])
            .setGoal(new ColorGoal(2, new int[]{
                sp.colorgoal.getRed(), sp.colorgoal.getGreen(), sp.colorgoal.getBlue()
            }));
    }

    // ------------------------------------------------------------------ //
    //  Paquets
    // ------------------------------------------------------------------ //
    /**
     * Génère n paquets et les répartit aléatoirement sur les zones de départ.
     */
    public void createPackages(int n) {
        String[] startZones = {"A1", "A2", "A3"};
        for (int i = 0; i < n; i++) {
            if (totalGenerated >= nbPackages) break;
            int destinationId = rnd.nextInt(2) + 1;
            int ts = 0;
            int randomStartZone = rnd.nextInt(startZones.length);
            String zone = startZones[randomStartZone];
            int[] position = {-1, -1};
            ColorPackage pack = new ColorPackage(
                position,
                new int[]{
                    sp.colorpackage.getRed(),
                    sp.colorpackage.getGreen(),
                    sp.colorpackage.getBlue()
                },
                destinationId,
                ts,
                zone
            );
            ColorStartZone startZone = getStartZoneById(zone);
            if (startZone != null) {
                startZone.addPackage(pack);
                totalGenerated++;
            } else {
                System.out.println("Zone de départ introuvable : " + zone);
            }
        }
    }

    public ColorStartZone getStartZoneById(String id) {
        return startZonesMap.get(id);
    }

    // ------------------------------------------------------------------ //
    //  Zones de départ
    // ------------------------------------------------------------------ //
    public void createStartZones() {
        for (Map.Entry<String, int[]> entry : sp.startZonePositions.entrySet()) {
            String zoneId = entry.getKey();
            int[] pos = entry.getValue();
            ColorStartZone zone = new ColorStartZone(pos, new int[]{
                sp.colorstartzone.getRed(),
                sp.colorstartzone.getGreen(),
                sp.colorstartzone.getBlue()
            });
            addNewComponent(zone);
            startZonesMap.put(zoneId, zone);
        }
    }

    // ------------------------------------------------------------------ //
    //  Zones de transit
    // ------------------------------------------------------------------ //
    public void createTransitZones() {
        for (int[] data : sp.transitZoneData) {
            int x = data[0], y = data[1], capacity = data[2];
            ColorTransitZone tz = new ColorTransitZone(
                new int[]{x, y},
                new int[]{
                    sp.colortransitzone.getRed(),
                    sp.colortransitzone.getGreen(),
                    sp.colortransitzone.getBlue()
                },
                capacity
            );
            addNewComponent(tz);
            transitZonesMap.put("T" + (transitZonesMap.size() + 1), tz);
        }
    }


    // ------------------------------------------------------------------ //
    //  Zones de sortie physiques (portes)
    // ------------------------------------------------------------------ //
    public void createExitZones() {
        for (int[] pos : sp.exitZonePositions) {
            ColorExitZone exitZone = new ColorExitZone(pos, new int[]{
                sp.colorexit.getRed(), sp.colorexit.getGreen(), sp.colorexit.getBlue()
            });
            addNewComponent(exitZone);
        }
    }

    // ------------------------------------------------------------------ //
    //  Stations de recharge – chargement depuis environment.ini
    // ------------------------------------------------------------------ //
    /**
     * Charge les positions des stations de recharge depuis la section [rechargeZones].
     * Format : Rx = row,col
     * Les stations sont marquées visuellement par setGoal() (pas addNewComponent)
     * pour que leurs cellules restent VIDES et franchissables par les robots.
     */
    public void loadRechargeZones(IniFile ifile) {
        rechargePositions.clear();
        int idx = 1;
        while (true) {
            String val = ifile.getStringValue("rechargeZones", "R" + idx);
            if (val == null || val.isEmpty()) break;
            String[] parts = val.split(",");
            int row = Integer.parseInt(parts[0].trim());
            int col = Integer.parseInt(parts[1].trim());
            rechargePositions.add(new int[]{row, col});

            // Marqueur visuel uniquement : on utilise setGoal pour ne PAS
            // occuper la cellule (addNewComponent la bloquerait pour A* et le déplacement)
            ((ColorCell) environment.getGrid()[row][col])
                .setGoal(new ColorGoal(-idx, new int[]{255, 0, 255})); // magenta

            idx++;
        }

        // Initialisation du compteur d’occupation
        rechargeOccupiedSteps = new int[rechargePositions.size()];
        rechargeTotalSteps = 0;

        System.out.println("Stations de recharge chargées : " + rechargePositions.size());
    }

    // ------------------------------------------------------------------ //
    //  Travailleurs (obstacles mobiles)
    // ------------------------------------------------------------------ //
    public void createWorker() {
        for (int i = 0; i < numberOfWorkers; i++) {
            int[] pos = environment.getPlace();
            Worker worker = new Worker(
                "Worker" + i, sp.field, sp.debug, pos,
                new Color(sp.colorother.getRed(), sp.colorother.getGreen(), sp.colorother.getBlue()),
                sp.rows, sp.columns, sp.seed
            );
            addNewComponent(worker);
        }
    }

    // ------------------------------------------------------------------ //
    //  Robots AMR
    // ------------------------------------------------------------------ //
    @Override
    public void createRobot() {
        for (int i = 0; i < sp.nbrobot; i++) {
            int[] pos = environment.getPlace();
            MyRobot robot = new MyRobot(
                "Robot" + i, sp.field, sp.debug, pos,
                new Color(sp.colorrobot.getRed(), sp.colorrobot.getGreen(), sp.colorrobot.getBlue()),
                sp.rows, sp.columns, (ColorGridEnvironment) environment, sp.seed,
                startZonesMap,
                transitZonesMap,
                sp.goalPositions,
                rechargePositions,
                this
            );
            addNewComponent(robot);
        }

    }
    public synchronized void incrDelivered(int goalId) {
        deliveredCount++;
        deliveredPerGoal.merge(goalId, 1, Integer::sum);
    }

    // ------------------------------------------------------------------ //
    //  Distribution des messages broadcast entre robots
    // ------------------------------------------------------------------ //
    private void distributeMessages(List<Robot> robots) {
        for (Robot sender : robots) {
            if (!(sender instanceof InteractionRobot)) continue;
            List<Message> sent = ((InteractionRobot) sender).popSentMessages();
            for (Message msg : sent) {
                for (Robot target : robots) {
                    if (target != sender && target instanceof InteractionRobot) {
                        ((InteractionRobot) target).receiveMessage(msg);
                    }
                }
            }
        }
    }

    // ------------------------------------------------------------------ //
    //  Boucle de simulation
    // ------------------------------------------------------------------ //
    @Override
    public void schedule() {
        List<Robot> robots = environment.getRobot();
        simStartTime = System.currentTimeMillis();

        if (fastMode) {
            System.out.println("\nDémarrage de la simulation en mode RAPIDE...");
            System.out.print("Calcul en cours ");
        }

        for (int i = 0; i < sp.step; i++) {
            totalSteps++;

            // Comptage occupation des stations de recharge
            if (rechargeOccupiedSteps != null && !rechargePositions.isEmpty()) {
                for (int s = 0; s < rechargePositions.size(); s++) {
                    int[] pos = rechargePositions.get(s);
                    boolean occupee = false;
                    for (Robot r : robots) {
                        if (r.getX() == pos[0] && r.getY() == pos[1]) {
                            occupee = true;
                            break;
                        }
                    }
                    if (occupee) rechargeOccupiedSteps[s]++;
                }
                rechargeTotalSteps++;
            }

            // Génération de paquets par lots (tous les 10 pas)
            if (nbNotGeneratedPackages > 0 && validGeneration()) {
                int currentNBPacket;
                if (nbNotGeneratedPackages > 2)
                    currentNBPacket = rnd.nextInt(nbNotGeneratedPackages / 2 + 1);
                else
                    currentNBPacket = 2;
                createPackages(currentNBPacket);
                nbNotGeneratedPackages -= currentNBPacket;
            }

            // Activation des robots
            for (Robot r : robots) {
                int[] prevPos = r.getLocation();
                Cell[][] perception = environment.getNeighbor(r.getX(), r.getY(), r.getField());
                r.updatePerception(perception);
                r.step();
                updateEnvironment(prevPos, r.getLocation());
            }

            // Distribution en fin de cycle
            distributeMessages(robots);

            // --- GESTION DU MODE ---
            if (fastMode) {
                if (totalSteps % 50 == 0) {
                    System.out.print(".");
                }
            } else {
                refreshGW();
                printLiveDashboard(robots, totalSteps);
                try {
                    Thread.sleep(sp.waittime);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

            // Condition de victoire
            if (deliveredCount >= nbPackages) {
                if (fastMode) System.out.println(" Objectif de livraison atteint !");
                break;
            }
        }

        if (fastMode) System.out.println("\n✅ Simulation terminée.");

        // Rapport final affiché dans les deux modes
        printStatistics(robots);
    }

    private boolean validGeneration() {
        return totalSteps % 2 == 0;
    }

    /**
     * Retourne le taux d’utilisation global des stations de recharge (0..1)
     * = somme des occupations / (nb de stations × nb de cycles)
     */
    public double getRechargeUtilization() {
        if (rechargeOccupiedSteps == null || rechargeTotalSteps == 0 || rechargeOccupiedSteps.length == 0) return 0.0;
        int sum = 0;
        for (int occ : rechargeOccupiedSteps) sum += occ;
        return (double) sum / (rechargeOccupiedSteps.length * rechargeTotalSteps);
    }

    /**
     * Calcule la moyenne du ratio idle des robots (0..1)
     */
    public double getAverageIdleRatio() {
        List<Robot> robots = environment.getRobot();
        if (robots == null || robots.isEmpty()) return 0.0;
        double sum = 0.0;
        int count = 0;
        for (Robot r : robots) {
            if (r instanceof simulator.MyRobot mr) {
                sum += mr.getIdleRatio();
                count++;
            }
        }
        return count > 0 ? sum / count : 0.0;
    }

    // ------------------------------------------------------------------ //
    //  Dashboard live (mis à jour chaque cycle)
    // ------------------------------------------------------------------ //
    private void printLiveDashboard(List<Robot> robots, int step) {
        long elapsed = System.currentTimeMillis() - simStartTime;
        StringBuilder sb = new StringBuilder();

        final int W = 60;
        String sep = BOLD + "═".repeat(W) + RESET;

        // ── En-tête ──────────────────────────────────────────────────────
        sb.append(sep).append("\n");
        sb.append(BOLD).append(String.format(
                "   ENTREPÔT SIMULÉ  –  Étape %4d / %-4d   (%ds écoulées)",
                step, sp.step, elapsed / 1000)).append(RESET).append("\n");
        sb.append(sep).append("\n\n");

        // ── Paquets globaux ──────────────────────────────────────────────
        sb.append(BOLD).append(CYAN)
          .append("── Paquets ").append("─".repeat(W - 11))
          .append(RESET).append("\n");
        double pct = 100.0 * deliveredCount / Math.max(nbPackages, 1);
        int filled = (int)(20.0 * deliveredCount / Math.max(nbPackages, 1));
        String bar = GREEN + "█".repeat(filled) + RESET + "░".repeat(20 - filled);
        sb.append(String.format("  Livrés : %d / %d  [%s]  %.1f %%\n",
                deliveredCount, nbPackages, bar, pct));

        // ── Zones de départ ───────────────────────────────────────────────
        sb.append("\n").append(BOLD).append(CYAN)
          .append("── Zones de départ ").append("─".repeat(W - 19))
          .append(RESET).append("\n");
        // Trier par clé pour un affichage stable
        List<Map.Entry<String, ColorStartZone>> startEntries =
                new ArrayList<>(startZonesMap.entrySet());
        startEntries.sort(Map.Entry.comparingByKey());
        for (Map.Entry<String, ColorStartZone> e : startEntries) {
            ColorStartZone z = e.getValue();
            int n = z.getPackages().size();
            String boxes = (n > 0 ? YELLOW : "") + "■".repeat(Math.min(n, 8))
                    + RESET + "░".repeat(Math.max(0, 8 - n));
            sb.append(String.format("  %-4s (%2d,%2d)  : [%s]  %d colis%n",
                    e.getKey(), z.getX(), z.getY(), boxes, n));
        }

        // ── Zones de transit ──────────────────────────────────────────────
        sb.append("\n").append(BOLD).append(CYAN)
          .append("── Zones de transit ").append("─".repeat(W - 20))
          .append(RESET).append("\n");
        List<Map.Entry<String, ColorTransitZone>> transitEntries =
                new ArrayList<>(transitZonesMap.entrySet());
        transitEntries.sort(Map.Entry.comparingByKey());
        for (Map.Entry<String, ColorTransitZone> e : transitEntries) {
            ColorTransitZone z = e.getValue();
            int n   = z.getPackages().size();
            int cap = z.getCapacity();
            String statusColor = z.isFull() ? RED : (n > 0 ? YELLOW : GREEN);
            String status      = z.isFull() ? "PLEIN" : (n > 0 ? n + "/" + cap : "VIDE ");
            sb.append(String.format("  %-4s (%2d,%2d)  : %d/%d  %s%s%s%n",
                    e.getKey(), z.getX(), z.getY(), n, cap,
                    statusColor, status, RESET));
        }

        // ── Zones de livraison ────────────────────────────────────────────
        sb.append("\n").append(BOLD).append(CYAN)
          .append("── Zones de livraison ").append("─".repeat(W - 22))
          .append(RESET).append("\n");
        int perGoalExpected = Math.max(nbPackages / Math.max(sp.goalPositions.size(), 1), 1);
        List<Map.Entry<Integer, int[]>> goalEntries =
                new ArrayList<>(sp.goalPositions.entrySet());
        goalEntries.sort(Map.Entry.comparingByKey());
        for (Map.Entry<Integer, int[]> e : goalEntries) {
            int gid  = e.getKey();
            int[] gp = e.getValue();
            int del  = deliveredPerGoal.getOrDefault(gid, 0);
            int gfilled = (int)(20.0 * del / Math.max(perGoalExpected, 1));
            gfilled = Math.min(gfilled, 20);
            String gbar = GREEN + "█".repeat(gfilled) + RESET + "░".repeat(20 - gfilled);
            sb.append(String.format("  Z%-3d (%2d,%2d)  : [%s]  %d livré(s)%n",
                    gid, gp[0], gp[1], gbar, del));
        }

        // ── Robots ────────────────────────────────────────────────────────
        sb.append("\n").append(BOLD).append(CYAN)
          .append("── Robots ").append("─".repeat(W - 10))
          .append(RESET).append("\n");
        sb.append(String.format("  %-10s  %-22s  %-8s  %-7s  %s%n",
                "Nom", "État", "Position", "Batt.", "Colis"));
        sb.append("  ").append("─".repeat(W - 2)).append("\n");
        for (Robot r : robots) {
            if (!(r instanceof MyRobot mr)) continue;
            int bat = mr.chargeLevel;
            String batColor = bat >= 60 ? GREEN : (bat >= 25 ? YELLOW : RED);
            String batStr   = batColor + String.format("%3d%%", bat) + RESET;
            String colisStr = mr.carriedPackage != null
                    ? YELLOW + "→Z" + mr.carriedPackage.getDestinationGoalId() + RESET
                    : "  —  ";
            sb.append(String.format("  %-10s  %-22s  (%2d,%2d)   %s   %s%n",
                    mr.getName(),
                    mr.getEtatString(),
                    mr.getX(), mr.getY(),
                    batStr,
                    colisStr));
        }

        sb.append("\n").append(sep).append("\n");
        System.out.print(sb);
    }

    // ------------------------------------------------------------------ //
    //  Rapport de simulation final
    // ------------------------------------------------------------------ //
    private void printStatistics(List<Robot> robots) {
        long elapsed   = System.currentTimeMillis() - simStartTime;
        boolean done   = deliveredCount >= nbPackages;
        String sep     = "═".repeat(54);

        System.out.println("\n" + sep);
        System.out.println("          RAPPORT DE SIMULATION FINAL");
        System.out.println(sep);

        System.out.println("\n── Paramètres d'entrée ────────────────────────────────");
        System.out.printf("  Grille              : %d × %d cellules%n", sp.rows, sp.columns);
        System.out.printf("  Robots AMR          : %d%n", sp.nbrobot);
        System.out.printf("  Workers (mobiles)   : %d%n", numberOfWorkers);
        System.out.printf("  Obstacles statiques : %d%n",
                sp.obstaclePositions != null ? sp.obstaclePositions.length : 0);
        System.out.printf("  Zones de départ     : %d  %s%n",
                sp.startZonePositions != null ? sp.startZonePositions.size() : 0,
                sp.startZonePositions != null ? sp.startZonePositions.keySet() : "");
        System.out.printf("  Zones de transit    : %d  (cap. 1 chacune)%n",
                sp.transitZoneData != null ? sp.transitZoneData.size() : 0);
        System.out.printf("  Stations recharge   : %d%n", rechargePositions.size());
        System.out.printf("  Goals de livraison  : %d%n",
                sp.goalPositions != null ? sp.goalPositions.size() : 0);
        System.out.printf("  Seed                : %d%n", sp.seed);
        System.out.printf("  Pas max configuré   : %d%n", sp.step);
        System.out.printf("  Délai par cycle     : %d ms%n", sp.waittime);
        System.out.printf("  Objectif paquets    : %d  (= %d robots × 3)%n",
                nbPackages, sp.nbrobot);

        System.out.println("\n── Résultats ──────────────────────────────────────────");
        System.out.printf("  Statut              : %s%n",
                done ? "COMPLÉTÉ ✓" : "INTERROMPU — pas max atteint");
        System.out.printf("  Paquets livrés      : %d / %d  (%.1f %%)%n",
                deliveredCount, nbPackages, 100.0 * deliveredCount / Math.max(nbPackages, 1));
        System.out.printf("  Étapes simulées     : %d / %d%n", totalSteps, sp.step);
        System.out.printf("  Durée réelle        : %.1f s%n", elapsed / 1000.0);
        if (deliveredCount > 0 && totalSteps > 0) {
            System.out.printf("  Efficacité          : %.3f paquet / étape%n",
                    (double) deliveredCount / totalSteps);
            System.out.printf("  Moy. étapes/paquet  : %.1f étapes%n",
                    (double) totalSteps / deliveredCount);
            System.out.printf("  Moy. étapes*robot/paquet   : %.1f étapes%n",
                    (double) totalSteps * robots.size() / deliveredCount);
        }

        System.out.println("\n── Par robot ──────────────────────────────────────────");
        System.out.printf("  %-12s  %s  %s%n", "Robot", "Batterie finale", "Livraisons");
        System.out.println("  " + "─".repeat(42));
        int totalDelivered = 0;
        for (Robot r : robots) {
            if (r instanceof MyRobot mr) {
                int bat  = mr.chargeLevel;
                int del  = mr.getDeliveredByThisRobot();
                totalDelivered += del;
                System.out.printf("  %-12s  %3d / %3d (%5.1f %%)  %d livraison(s)%n",
                        mr.getName(), bat, MyRobot.MAX_CHARGE,
                        100.0 * bat / MyRobot.MAX_CHARGE, del);
            }
        }
        System.out.printf("  %-12s  %s               %d livraison(s)%n",
                "TOTAL", "─".repeat(16), totalDelivered);

        System.out.println("\n" + sep + "\n");
    }

    public double[] getStats() {
        double rechargeUtil = getRechargeUtilization();
        double avgIdleRatio = getAverageIdleRatio();
        double stepsPerPackage = deliveredCount > 0 ? (double) totalSteps / deliveredCount : 0.0;
        double robotStepsPerPackage = deliveredCount > 0 ? (double) totalSteps * environment.getRobot().size() / deliveredCount : 0.0;
        return new double[]{rechargeUtil, avgIdleRatio, stepsPerPackage, robotStepsPerPackage};
    }
}

