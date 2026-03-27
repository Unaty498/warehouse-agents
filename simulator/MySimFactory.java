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

    private final Map<String, ColorStartZone>   startZonesMap  = new HashMap<>();
    private final Map<Integer, ColorExitZone>   exitZonesMap   = new HashMap<>();
    private final Map<String, ColorTransitZone> transitZonesMap = new HashMap<>();

    /** Positions des stations de recharge chargées depuis environment.ini */
    private final List<int[]> rechargePositions = new ArrayList<>();

    public static int deliveredCount = 0;
    int nbPackages;
    int nbNotGeneratedPackets;
    int numberOfWorkers;
    Random rnd;
    int totalSteps = 0;

    public MySimFactory(SimProperties sp) {
        super(sp);
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
        for (int[] pos : sp.obstaclePositions) {
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
    public void createPackages() {
        String[] startZones = {"A1", "A2", "A3"};
        for (String s : startZones) {
            int destinationId = rnd.nextInt(2) + 1;
            int[] position = {-1, -1};
            ColorPackage pack = new ColorPackage(
                    position,
                    new int[]{
                            sp.colorpackage.getRed(),
                            sp.colorpackage.getGreen(),
                            sp.colorpackage.getBlue()
                    },
                    destinationId, 0, s
            );
            ColorStartZone startZone = getStartZoneById(s);
            if (startZone != null) {
                startZone.addPackage(pack);
            } else {
                System.out.println("Zone de départ introuvable : " + s);
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
    //  Zones de goal (ColorExitZone avec ID entier, non utilisées par défaut)
    // ------------------------------------------------------------------ //
    public void createGoalZones() {
        for (Map.Entry<Integer, int[]> entry : sp.goalPositions.entrySet()) {
            int goalId = entry.getKey();
            int[] pos = entry.getValue();
            ColorExitZone exitZone = new ColorExitZone(pos, new int[]{
                sp.colorgoal.getRed(), sp.colorgoal.getGreen(), sp.colorgoal.getBlue()
            });
            addNewComponent(exitZone);
            exitZonesMap.put(goalId, exitZone);
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
                rechargePositions
            );
            addNewComponent(robot);
        }
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

        int currentNBPacket;
        for (int i = 0; i < sp.step; i++) {
            totalSteps++;

            // Génération de paquets
            createPackages();

            // Activation des robots
            for (Robot r : robots) {
                int[] prevPos = r.getLocation();
                Cell[][] perception = environment.getNeighbor(r.getX(), r.getY(), r.getField());
                r.updatePerception(perception);
                r.step();
                updateEnvironment(prevPos, r.getLocation());
            }

            // Distribution en fin de cycle (contrainte du modèle : synchrone)
            // Les messages du cycle N sont reçus au cycle N+1.
            distributeMessages(robots);

            refreshGW();

            if (MySimFactory.deliveredCount >= nbPackages) {
                System.out.println("Tous les paquets livrés en " + totalSteps + " étapes.");
                break;
            }

            try {
                Thread.sleep(sp.waittime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    private boolean validGeneration() {
        return totalSteps % 2 == 0;
    }

    // ------------------------------------------------------------------ //
    //  Point d'entrée principal
    // ------------------------------------------------------------------ //
    public static void main(String[] args) throws Exception {
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

        System.out.println("Grille : " + sp.rows + "x" + sp.columns);
        System.out.println("Robots : " + sp.nbrobot);

        MySimFactory sim = new MySimFactory(sp);
        sim.nbPackages            = sp.nbrobot * 3;   // 3× le nb de robots
        sim.nbNotGeneratedPackets = sim.nbPackages;
        sim.numberOfWorkers       = sp.nbobstacle / 2;
        sim.rnd                   = new Random(sp.seed);


        sim.createEnvironment();
        sim.createObstacle();
        sim.createGoal();
        sim.createStartZones();
        sim.createTransitZones();
        sim.createExitZones();
        sim.createWorker();
        sim.createRobot();

        sim.loadRechargeZones(ifileEnv);

        sim.initializeGW();
        sim.schedule();
    }
}

