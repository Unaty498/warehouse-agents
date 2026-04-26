package simulator;

import fr.emse.fayol.maqit.simulator.components.*;
import fr.emse.fayol.maqit.simulator.environment.Cell;
import fr.emse.fayol.maqit.simulator.environment.ColorGridEnvironment;
import pathfind.PathFinder;

import java.awt.Color;
import java.util.*;

/**
 * Robot AMR (Autonomous Mobile Robot) pour la simulation d'entrepôt.
 *
 * Machine à états :
 *   IDLE              – cherche une tâche
 *   MOVING_TO_PACKAGE – se déplace vers une zone source (départ ou transit)
 *   TRANSPORT_TO_GOAL – transporte un colis vers sa destination (transit ou sortie)
 *   MOVING_TO_CHARGE  – se dirige vers une station de recharge
 *   CHARGING          – en cours de recharge
 * Décision (cf. Modèle Conceptuel) :
 *   1. Assez de batterie pour trajet complet (départ → sortie → recharge) → trajet direct
 *   2. Assez pour trajet initial (départ → transit → recharge) et transit non plein → dépôt en transit
 *   3. Assez pour trajet final (transit → sortie → recharge) et transit non vide → ramasse en transit
 *   4. Sinon → recharge
 */
public class MyRobot extends ColorInteractionRobot {

    // ------------------------------------------------------------------ //
    //  États
    // ------------------------------------------------------------------ //
    private enum Etat {
        IDLE,
        MOVING_TO_PACKAGE,
        TRANSPORT_TO_GOAL,
        MOVING_TO_CHARGE,
        CHARGING
    }

    // ------------------------------------------------------------------ //
    //  État courant
    // ------------------------------------------------------------------ //
    private Etat etat;

    // Colis transporté
    public ColorPackage carriedPackage;

    // Destination de navigation (coordonnées cibles)
    protected int destX;
    protected int destY;


    // Référence à l'environnement global
    protected ColorGridEnvironment env;

    // ------------------------------------------------------------------ //
    //  Batterie
    // ------------------------------------------------------------------ //
    protected int chargeLevel;
    public static final int MAX_CHARGE       = 100;
    private  static final int CHARGE_COST    = 1;   // consommation par pas de déplacement
    private  static final int CHARGE_GAIN    = 5;   // recharge par tick à la station
    private  static final int SAFETY_MARGIN = 15; // seuil d'urgence

    // ------------------------------------------------------------------ //
    //  Connaissances (zones)
    // ------------------------------------------------------------------ //
    protected Map<String, ColorStartZone>   startZonesMap;
    protected Map<String, ColorTransitZone> transitZonesMap;
    protected Map<Integer, int[]>           goalPositions;   // id → [x,y]
    protected List<int[]>                   rechargePositions;

    // ------------------------------------------------------------------ //
    //  Réservations (Robot_ID → Zone_ID)
    // ------------------------------------------------------------------ //
    private final Map<Integer, String> robotToStartZone      = new HashMap<>();
    private final Map<Integer, String> robotToTransitPickup  = new HashMap<>();
    private final Map<Integer, String> robotToTransitDeposit = new HashMap<>();
    private final Map<Integer, String> robotToCharge         = new HashMap<>();

    private final Map<Integer, Integer> robotChargePriority = new HashMap<>();


    // ------------------------------------------------------------------ //
    //  Suivi de mission courante
    // ------------------------------------------------------------------ //
    private String  targetSourceId;
    private String  targetDestTransitId;
    private boolean targetSourceIsTransit;
    private boolean targetDestIsTransit;
    private int     targetGoalId;

    // ------------------------------------------------------------------ //
    //  Priorités de mission
    // ------------------------------------------------------------------ //
    private int mySourcePriority  = Integer.MIN_VALUE;
    private int myDepositPriority = Integer.MIN_VALUE;
    private int myChargePriority  = Integer.MIN_VALUE;


    /** Compteur de cycles consécutifs sans mouvement (anti-blocage). */
    private int stuckCounter = 0;

    /** Nombre de cycles consécutifs en IDLE sans mission trouvée. */
    private int idleNoMissionCycles = 0;

    /**
     * Seuil au-delà duquel les maps de réservation sont purgées pour casser
     * un éventuel deadlock dû à des réservations périmées.
     */
    private static final int IDLE_STALE_THRESHOLD = 20;

    /** Nombre de livraisons accomplies par ce robot. **/
    private int deliveredByThisRobot = 0;
    public int getDeliveredByThisRobot() { return deliveredByThisRobot; }

    /** Expose l'état courant sous forme de chaîne (utilisé par le dashboard). */
    public String getEtatString() { return etat.name(); }

    // ------------------------------------------------------------------ //
    //  Issue 1 : Cache de chemin A*
    // ------------------------------------------------------------------ //
    private int[][] cachedPath      = null;
    private int     cachedPathIndex = 0;
    private int     lastTargetX     = -1;
    private int     lastTargetY     = -1;

    // ------------------------------------------------------------------ //
    //  Issue 2 : Détection d'oscillation
    // ------------------------------------------------------------------ //
    private final LinkedList<int[]> positionHistory = new LinkedList<>();
    private static final int  POSITION_HISTORY_SIZE = 6;
    private int               oscillationWaitCounter = 0;

    // ------------------------------------------------------------------ //
    //  Issues 3 & 8 : Cellule bloquante + obstacles temporaires
    // ------------------------------------------------------------------ //
    private int[] blockedCell              = null;
    private int   mutualBlockTimer         = 0;
    private static final int MUTUAL_BLOCK_THRESHOLD = 3;

    // ------------------------------------------------------------------ //
    //  Blocage de départ et de ramassage en transit en cas de surcharge
    // ------------------------------------------------------------------ //
    private boolean BlockedStart=false;
    private boolean BlockedTransitPickup=false;

    // ================================================================== //
    //  Constructeur
    // ================================================================== //
    public MyRobot(String name, int field, int debug, int[] pos, Color color,
                   int rows, int columns, ColorGridEnvironment env, long seed,
                   Map<String, ColorStartZone>   startZonesMap,
                   Map<String, ColorTransitZone> transitZonesMap,
                   Map<Integer, int[]>           goalPositions,
                   List<int[]>                   rechargePositions) {
        super(name, field, debug, pos, color, rows, columns, seed);
        this.env               = env;
        this.etat              = Etat.IDLE;
        this.carriedPackage    = null;
        this.chargeLevel       = MAX_CHARGE;
        this.startZonesMap     = startZonesMap;
        this.transitZonesMap   = transitZonesMap;
        this.goalPositions     = goalPositions;
        this.rechargePositions = rechargePositions;
        randomOrientation();
    }

    // ================================================================== //
    //  Utilitaires – distances / positions
    // ================================================================== //

    protected int distanceManhattan(int x1, int y1, int x2, int y2) {
        return Math.abs(x2 - x1) + Math.abs(y2 - y1);
    }

    private boolean isAt(int x, int y) {
        return getX() == x && getY() == y;
    }

    /**
     * Vérifie si le robot est assez proche d'une cellule cible pour interagir.
     * Accepte distance Manhattan ≤ 1.
     */
    protected boolean isNearZone(int row, int col) {
        return distanceManhattan(getX(), getY(), row, col) <= 1;
    }


    // ================================================================== //
    //  Recherche de zones
    // ================================================================== //

    protected ColorStartZone findBestStartZone() {
        ColorStartZone best = null;
        int minDist = Integer.MAX_VALUE;
        for (Map.Entry<String, ColorStartZone> e : startZonesMap.entrySet()) {
            ColorStartZone z = e.getValue();
            String zoneId = e.getKey();
            // Compter combien de robots se dirigent vers cette zone
            long resCount = robotToStartZone.values().stream().filter(id -> id.equals(zoneId)).count();

            // Si la zone a plus de colis que de robots en route, on peut y aller !
            if (z.getPackages().size() > resCount) {
                int d = distanceManhattan(getX(), getY(), z.getX(), z.getY());
                if (d < minDist) { minDist = d; best = z; targetSourceId = zoneId; }
            }
        }
        return best;
    }

    protected ColorTransitZone findBestTransitZoneWithPackages() {
        ColorTransitZone best = null;
        int minDist = Integer.MAX_VALUE;
        for (Map.Entry<String, ColorTransitZone> e : transitZonesMap.entrySet()) {
            ColorTransitZone z = e.getValue();
            String zoneId = e.getKey();
            long resCount = robotToTransitPickup.values().stream().filter(id -> id.equals(zoneId)).count();

            if (z.getPackages().size() > resCount) {
                int d = distanceManhattan(getX(), getY(), z.getX(), z.getY());
                if (d < minDist) { minDist = d; best = z; targetSourceId = zoneId; }
            }
        }
        return best;
    }

    protected ColorTransitZone findBestTransitZoneWithCapacity() {
        ColorTransitZone best = null;
        int minDist = Integer.MAX_VALUE;
        for (Map.Entry<String, ColorTransitZone> e : transitZonesMap.entrySet()) {
            ColorTransitZone z = e.getValue();
            String zoneId = e.getKey();
            long incomingCount = robotToTransitDeposit.values().stream().filter(id -> id.equals(zoneId)).count();

            // Le transit a une capacité de 1. Personne ne doit être en route vers lui et il ne doit pas être plein.
            if (!z.isFull() && incomingCount == 0) {
                int d = distanceManhattan(getX(), getY(), z.getX(), z.getY());
                if (d < minDist) { minDist = d; best = z; targetDestTransitId = zoneId; }
            }
        }
        return best;
    }

    /**
     * Station de recharge la plus proche – utilisée pour l'ESTIMATION du coût batterie.
     */
    protected int[] findClosestRechargePosition() {
        int[] closest = null;
        int minDist = Integer.MAX_VALUE;
        for (int[] pos : rechargePositions) {
            int d = distanceManhattan(getX(), getY(), pos[0], pos[1]);
            if (d < minDist) { minDist = d; closest = pos; }
        }
        return closest;
    }

    /**
     * Meilleure station disponible pour la NAVIGATION réelle.
     */
    protected int[] findBestAvailableRechargePosition() {
        int[] best = null;
        int minDist = Integer.MAX_VALUE;

        for (int[] pos : rechargePositions) {
            String key = pos[0] + "," + pos[1];
            // Combien de robots ciblent cette station précise ?
            long count = robotToCharge.values().stream().filter(v -> v.equals(key)).count();

            // On accepte 2 robots MAX (1 qui charge, 1 qui fait la queue derrière)
            if (count < 2) {
                int d = distanceManhattan(getX(), getY(), pos[0], pos[1]);
                if (d < minDist) { minDist = d; best = pos; }
            }
        }
        return best; // Renverra null si tout est saturé !
    }

    private boolean isPhysicallyFree(int[] pos) {
        Cell c = env.getGrid()[pos[0]][pos[1]];
        boolean physicallyFree = true;

        // Si la case contient quelque chose, on vérifie si c'est un agent
        if (c != null && c.getContent() != null) {
            SituatedComponent comp = c.getContent();
            if (comp instanceof Robot || comp.getClass().getSimpleName().contains("Worker")) {
                physicallyFree = false; // Un robot bloque la place
            }
        }
        return physicallyFree;
    }

    // ================================================================== //
    //  Carte d'obstacles pour A*
    // ================================================================== //

    /**
     * Carte des obstacles STATIQUES pour A*.
     * Inclut : ColorObstacle, ColorStartZone, ColorTransitZone, ColorExitZone.
     * Exclut volontairement les autres robots.
     */
    private boolean[][] buildStaticObstacleMap() {
        boolean[][] obs = new boolean[rows][columns];
        Cell[][] g = env.getGrid();
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < columns; j++) {
                if (g[i][j] != null && g[i][j].getContent() != null) {
                    SituatedComponent c = g[i][j].getContent();
                    if (c instanceof ColorObstacle
                            || c instanceof ColorStartZone
                            || c instanceof ColorTransitZone
                            || c instanceof ColorExitZone) {
                        obs[i][j] = true;
                    }
                }

                // Retirer les stations de recharge
                for (int[] rechargePos : rechargePositions) {
                    if (i == rechargePos[0] && j == rechargePos[1]) {
                        obs[i][j] = false;
                        break;
                    }
                }
            }
        }
        return obs;
    }


    /**
     * Enregistre la position courante dans l'historique,
     */
    private void updatePositionHistory() {
        int[] current = {getX(), getY()};
        if (!positionHistory.isEmpty()) {
            int[] last = positionHistory.getLast();
            if (last[0] == current[0] && last[1] == current[1]) return; // pas bougé
        }
        positionHistory.add(current);
        if (positionHistory.size() > POSITION_HISTORY_SIZE) {
            positionHistory.removeFirst();
        }
    }

    /**
     * Détecte une oscillation de période 2 ou 3 (pattern A-B-A-B ou A-B-C-A-B-C).
     * @return true si une oscillation est détectée
     */
    private boolean detectOscillation() {
        int n = positionHistory.size();
        if (n < 4) return false;
        for (int period = 2; period <= 3 && period * 2 <= n; period++) {
            boolean isPeriodic = true;
            for (int i = n - 1; i >= period; i--) {
                int[] p1 = positionHistory.get(i);
                int[] p2 = positionHistory.get(i - period);
                if (p1[0] != p2[0] || p1[1] != p2[1]) { isPeriodic = false; break; }
            }
            if (isPeriodic) return true;
        }
        return false;
    }

    /**
     * Vérifie si ce robot est celui qui a la plus haute priorité (batterie la plus faible)
     * parmi tous ceux qui ciblent la MÊME station de recharge.
     */
    private boolean hasChargePriority() {
        String myTarget = destX + "," + destY;

        for (Map.Entry<Integer, String> entry : robotToCharge.entrySet()) {
            int otherId = entry.getKey();
            String otherTarget = entry.getValue();

            // Si un autre robot cible la même station...
            if (otherId != getId() && otherTarget.equals(myTarget)) {
                int otherPri = robotChargePriority.getOrDefault(otherId, 0);

                // ... et qu'il est plus urgent que nous (ou égal mais ID inférieur)
                if (otherPri > myChargePriority || (otherPri == myChargePriority && otherId < getId())) {
                    return false; // On n'a pas la priorité, on doit attendre
                }
            }
        }
        return true; // La voie est libre pour nous !
    }

    // ================================================================== //
    //  Déplacement
    // ================================================================== //

    /**
     * Déplace le robot d'un pas vers (targetX, targetY) via A*.
     * Implémente un cache de chemin pour éviter de recalculer à chaque tick, avec invalidation intelligente.
     */
    protected void moveOneStepTo(int targetX, int targetY) {
        if (isAt(targetX, targetY)) { cachedPath = null; return; }

        // --- INJECTION DEBUG ---
        boolean isOnRechargeZone = false;
        for (int[] r : rechargePositions) {
            if (r[0] == getX() && r[1] == getY()) {
                isOnRechargeZone = true; break;
            }
        }
        boolean isDebug = ((etat == Etat.MOVING_TO_PACKAGE || chargeLevel == 100) && isOnRechargeZone);

        if (isDebug) {
            System.out.println("\n[DEBUG] " + getName() + " (Bat:" + chargeLevel + "%) est SUR UNE STATION. Cible : (" + targetX + "," + targetY + ")");
            System.out.println("   -> cachedPath actuel : " + (cachedPath == null ? "NULL" : "Valide (idx: " + cachedPathIndex + "/" + cachedPath.length + ")"));
        }
        // -----------------------

        if (oscillationWaitCounter > 0) {
            if (isDebug) System.out.println("   -> Bloqué par oscillationWaitCounter");
            oscillationWaitCounter--;
            return;
        }

        if (targetX != lastTargetX || targetY != lastTargetY) {
            if (isDebug) System.out.println("   -> Invalidation du chemin (changement de cible)");
            cachedPath = null;
            lastTargetX = targetX;
            lastTargetY = targetY;
        }

        if (cachedPath != null && (cachedPathIndex >= cachedPath.length
                || cachedPath[cachedPathIndex][0] != getX()
                || cachedPath[cachedPathIndex][1] != getY())) {
            if (isDebug) System.out.println("   -> Invalidation du chemin (désynchronisation GPS/Physique)");
            cachedPath = null;
        }

        if (cachedPath == null) {
            if (isDebug) System.out.println("   -> Lancement du PathFinder A*...");
            boolean[][] obs = buildStaticObstacleMap();
            List<int[]> extra = (blockedCell != null) ? Collections.singletonList(blockedCell) : null;

            boolean targetIsObstacle = targetX >= 0 && targetX < rows
                    && targetY >= 0 && targetY < columns && obs[targetX][targetY];

            if (targetIsObstacle) {
                List<int[]> adjList = new ArrayList<>();
                for (int[] d : new int[][]{{-1,0},{1,0},{0,-1},{0,1}}) {
                    int nx = targetX + d[0], ny = targetY + d[1];
                    if (nx >= 0 && nx < rows && ny >= 0 && ny < columns && !obs[nx][ny])
                        adjList.add(new int[]{nx, ny});
                }
                if (adjList.isEmpty()) {
                    if (isDebug) System.out.println("   -> ERREUR : La cible est un obstacle et n'a aucune case adjacente libre !");
                    return;
                }
                cachedPath = PathFinder.aStarToAny(
                        new int[]{getX(), getY()},
                        adjList.toArray(new int[0][]),
                        rows, columns, obs, extra);
            } else {
                cachedPath = PathFinder.aStar(
                        new int[]{getX(), getY()},
                        new int[]{targetX, targetY},
                        rows, columns, obs, extra);
            }
            cachedPathIndex = 0;

            if (isDebug) {
                if (cachedPath == null) System.out.println("   -> A* a retourné NULL ! Le robot est emmuré vivant pour l'algorithme.");
                else System.out.println("   -> A* a trouvé un chemin de " + cachedPath.length + " pas.");
            }
        }

        if (cachedPath == null || cachedPath.length < 2 || cachedPathIndex + 1 >= cachedPath.length) {
            if (isDebug) System.out.println("   -> Chemin inutilisable. Increment stuckCounter: " + (stuckCounter + 1));
            cachedPath = null;
            if (++stuckCounter > 3) {
                if (isDebug) System.out.println("   -> stuckCounter > 3 : Tentative de tryRandomStep !");
                stuckCounter  = 0;
                blockedCell   = null;
                tryRandomStep(buildStaticObstacleMap());
            }
            return;
        }

        int nextX = cachedPath[cachedPathIndex + 1][0];
        int nextY = cachedPath[cachedPathIndex + 1][1];

        if (isDebug) System.out.println("   -> Le prochain pas demandé est : (" + nextX + "," + nextY + ")");

        // Orientation
        int dx = nextX - getX(), dy = nextY - getY();
        if      (dx == -1) orientation = Orientation.up;
        else if (dx ==  1) orientation = Orientation.down;
        else if (dy == -1) orientation = Orientation.left;
        else if (dy ==  1) orientation = Orientation.right;

        // Vérification d'obstacle
        Cell nextCell = env.getGrid()[nextX][nextY];
        if (nextCell != null && nextCell.getContent() != null) {
            SituatedComponent comp = nextCell.getContent();
            boolean isAgent = (comp instanceof Robot) || comp.getClass().getSimpleName().contains("Worker");

            if (isDebug) System.out.println("   -> OBSTACLE DÉTECTÉ sur le prochain pas : " + comp.getClass().getSimpleName() + " (isAgent=" + isAgent + ")");

            if (isAgent) {
                blockedCell = new int[]{nextX, nextY};
                mutualBlockTimer++;
                if (mutualBlockTimer >= MUTUAL_BLOCK_THRESHOLD) {
                    if (isDebug) System.out.println("   -> mutualBlockTimer atteint : invalidation du chemin.");
                    cachedPath       = null;
                    mutualBlockTimer = 0;
                }
                if (++stuckCounter > 5) {
                    if (isDebug) System.out.println("   -> Agent bloqueur persistant : tryRandomStep !");
                    stuckCounter = 0;
                    blockedCell  = null;
                    tryRandomStep(buildStaticObstacleMap());
                }
                return;
            }
        }

        if (isDebug) System.out.println("   -> Voie libre. Execution de moveForward().");

        stuckCounter     = 0;
        mutualBlockTimer = 0;
        blockedCell      = null;
        cachedPathIndex++;
        moveForward();
    }


    private void tryRandomStep(boolean[][] obs) {
        int[][] dirs = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
        List<int[]> candidates = new ArrayList<>();
        for (int[] d : dirs) {
            int nx = getX() + d[0], ny = getY() + d[1];
            if (nx < 0 || nx >= rows || ny < 0 || ny >= columns) continue;
            if (obs[nx][ny]) continue;
            Cell c = env.getGrid()[nx][ny];
            if (c == null || c.getContent() == null) candidates.add(new int[]{nx, ny});
        }
        if (candidates.isEmpty()) return;

        // Préférer la cellule la plus éloignée du bloquant connu
        int[] chosen;
        if (blockedCell != null) {
            chosen = candidates.getFirst();
            int maxDist = -1;
            for (int[] cand : candidates) {
                int d = distanceManhattan(cand[0], cand[1], blockedCell[0], blockedCell[1]);
                if (d > maxDist) { maxDist = d; chosen = cand; }
            }
        } else {
            chosen = candidates.get(rnd.nextInt(candidates.size()));
        }

        int dx = chosen[0] - getX(), dy = chosen[1] - getY();
        if      (dx == -1) orientation = Orientation.up;
        else if (dx ==  1) orientation = Orientation.down;
        else if (dy == -1) orientation = Orientation.left;
        else if (dy ==  1) orientation = Orientation.right;
        moveForward();
    }

    // ================================================================== //
    //  Messagerie (réservations naïves)
    // ================================================================== //

    private void broadcast(String content) {
        sendMessage(new Message(getId(), content));
    }

    // ================================================================== //
    //  Arbitrage de conflit basé sur le score (batterie / distance)
    // ================================================================== //

    /**
     * Détermine si ce robot doit céder la ressource à l'autre.
     */
    private boolean shouldYield(int otherId, int otherPriority, int myPriority) {
        if (otherPriority > myPriority) return true;
        if (otherPriority == myPriority) return otherId < getId();
        return false;
    }

    // ================================================================== //
    //  Abandon de mission (priorité perdue)
    // ================================================================== //

    /**
     * Abandonne la mission courante et libère ses propres réservations.
     * Les messages RELEASE incluent l'ID de l'émetteur ("<type>:zoneId:robotId").
     * Le handler ne supprime l'entrée que si reservedMap.get(zoneId) == robotId,
     * ce qui empêche un PERDANT d'effacer la réservation du GAGNANT.
     */
    private void yieldMission() {
        switch (etat) {
            case MOVING_TO_PACKAGE:
                if (targetSourceId != null) {
                    if (targetSourceIsTransit)
                        broadcast("RELEASE_PICKUP:" + targetSourceId + ":" + getId());
                    else
                        broadcast("RELEASE_START:"  + targetSourceId + ":" + getId());
                }
                if (targetDestIsTransit && targetDestTransitId != null)
                    broadcast("RELEASE_DEPOSIT:" + targetDestTransitId + ":" + getId());
                break;
            case TRANSPORT_TO_GOAL:
                if (targetDestIsTransit && targetDestTransitId != null)
                    broadcast("RELEASE_DEPOSIT:" + targetDestTransitId + ":" + getId());
                break;
            case MOVING_TO_CHARGE:
                broadcast("RELEASE_CHARGE:" + destX + "," + destY);
                break;
            default:
                break;
        }
        targetSourceId      = null;
        targetDestTransitId = null;
        mySourcePriority    = Integer.MIN_VALUE;
        myDepositPriority   = Integer.MIN_VALUE;
        myChargePriority    = Integer.MIN_VALUE;
        cachedPath          = null;
        positionHistory.clear();
        oscillationWaitCounter = 0;
        etat                = Etat.IDLE;
    }

    // ================================================================== //
    //  Gestionnaires d'états
    // ================================================================== //

    /** IDLE : choisit la meilleure action selon le niveau de batterie et l'état des zones. */
    private void handleIdle() {

        int[] recharge = findClosestRechargePosition();
        if (recharge == null) return;

        int cx = getX(), cy = getY();

        // ---- Cas 1 : trajet complet  départ → goal → recharge ----
        ColorStartZone sz = findBestStartZone();
        if (sz != null) {
            int sx = sz.getX(), sy = sz.getY();
            int distToStart = distanceManhattan(cx, cy, sx, sy);

            for (Map.Entry<Integer, int[]> ge : goalPositions.entrySet()) {
                int[] gp = ge.getValue();
                int cost = distToStart
                         + distanceManhattan(sx, sy, gp[0], gp[1])
                         + distanceManhattan(gp[0], gp[1], recharge[0], recharge[1]);
                // Ici, on utilise les informations récupérées par les messages de réservation.
                if (chargeLevel >= cost + SAFETY_MARGIN && !BlockedStart) {
                    targetSourceIsTransit = false;
                    targetDestIsTransit   = false;
                    targetGoalId          = ge.getKey();
                    destX = sx; destY = sy;
                    etat  = Etat.MOVING_TO_PACKAGE;
                    mySourcePriority  = chargeLevel - distToStart;
                    myDepositPriority = Integer.MIN_VALUE;
                    broadcast("RESERVE_START:" + targetSourceId + ":" + getId()
                            + ":" + chargeLevel + ":" + distToStart);
                    idleNoMissionCycles = 0;
                    return;
                }
            }

            // ---- Cas 2 : trajet initial départ → transit → recharge ----
            ColorTransitZone tz = findBestTransitZoneWithCapacity();
            if (tz != null) {
                int tx = tz.getX(), ty = tz.getY();
                int distToTransitViaStart = distToStart + distanceManhattan(sx, sy, tx, ty);
                int cost = distToStart
                         + distanceManhattan(sx, sy, tx, ty)
                         + distanceManhattan(tx, ty, recharge[0], recharge[1]);
                // Ici, on utilise les informations récupérées par les messages de réservation.
                if (chargeLevel >= cost + SAFETY_MARGIN) {
                    targetSourceIsTransit = false;
                    targetDestIsTransit   = true;
                    destX = sx; destY = sy;
                    etat  = Etat.MOVING_TO_PACKAGE;
                    mySourcePriority  = chargeLevel - distToStart;
                    myDepositPriority = chargeLevel - distToTransitViaStart;
                    broadcast("RESERVE_START:"   + targetSourceId   + ":" + getId()
                            + ":" + chargeLevel + ":" + distToStart);
                    broadcast("RESERVE_DEPOSIT:" + targetDestTransitId + ":" + getId()
                            + ":" + chargeLevel + ":" + distToTransitViaStart);
                    idleNoMissionCycles = 0;
                    return;
                }
            }
        }

        // ---- Cas 3 : trajet final  transit → goal → recharge ----
        ColorTransitZone tsrc = findBestTransitZoneWithPackages();
        if (tsrc != null) {
            int tx = tsrc.getX(), ty = tsrc.getY();
            int distToTransit = distanceManhattan(cx, cy, tx, ty);
            for (Map.Entry<Integer, int[]> ge : goalPositions.entrySet()) {
                int[] gp = ge.getValue();
                int cost = distToTransit
                         + distanceManhattan(tx, ty, gp[0], gp[1])
                         + distanceManhattan(gp[0], gp[1], recharge[0], recharge[1]);
                if (chargeLevel >= cost + SAFETY_MARGIN && !BlockedTransitPickup) {
                    targetSourceIsTransit = true;
                    targetDestIsTransit   = false;
                    targetGoalId          = ge.getKey();
                    destX = tx; destY = ty;
                    etat  = Etat.MOVING_TO_PACKAGE;
                    mySourcePriority  = chargeLevel - distToTransit;
                    myDepositPriority = Integer.MIN_VALUE;
                    broadcast("RESERVE_PICKUP:" + targetSourceId + ":" + getId()
                            + ":" + chargeLevel + ":" + distToTransit);
                    idleNoMissionCycles = 0;
                    return;
                }
            }
        }

        // ---- Cas 4 : recharge -----------------------------------------------
        // Si aucune mission n'est réalisable (cas 1/2/3 ont échoué) ET que la
        // batterie n'est pas pleine, on recharge.
        // Bug 2 corrigé : l'ancien seuil fixe à 80 % bloquait les robots dont
        // la batterie était entre 80 % et MAX_CHARGE sans mission disponible.
        // Désormais, tout robot inactif avec batterie < MAX_CHARGE va recharger.
        idleNoMissionCycles++;   // aucune mission n'a été prise ce cycle
        if (chargeLevel < MAX_CHARGE * 0.8) {
            int[] r = findBestAvailableRechargePosition();
            if (r != null) { // Oh, une place s'est libérée !
                destX = r[0]; destY = r[1];
                String key = destX + "," + destY;
                robotToCharge.put(getId(), key);
                broadcast("RESERVE_CHARGE:" + key + ":" + getId() + ":" + chargeLevel);
                etat = Etat.MOVING_TO_CHARGE;
                cachedPath = null;
            }
        } else {
            boolean onStation = false;
            for (int[] pos : rechargePositions) {
                if (isAt(pos[0], pos[1])) {
                    onStation = true;
                    break;
                }
            }

            if (onStation) {
                // On récupère la carte des obstacles
                boolean[][] escapeObs = buildStaticObstacleMap();

                // On remet TOUTES les stations comme des obstacles temporaires
                // pour forcer tryRandomStep à choisir une vraie case de couloir (sol plat)
                for (int[] pos : rechargePositions) {
                    if (pos[0] >= 0 && pos[0] < rows && pos[1] >= 0 && pos[1] < columns) {
                        escapeObs[pos[0]][pos[1]] = true;
                    }
                }

                // Le robot fait un pas pour descendre de la station !
                tryRandomStep(escapeObs);
            }
        }
    }

    /** MOVING_TO_PACKAGE : se déplace puis ramasse le colis quand proche de la zone source. */
    private void handleMovingToPackage() {
        if (isNearZone(destX, destY)) {
            ColorPackage pack = null;

            if (!targetSourceIsTransit) {
                ColorStartZone z = startZonesMap.get(targetSourceId);
                if (z != null && !z.getPackages().isEmpty()) {
                    pack = z.getPackages().getFirst();
                    z.removePackage(pack);
                    pack.setState(PackageState.DEPART);
                    carriedPackage = pack;
                    broadcast("RELEASE_START:" + targetSourceId + ":" + getId());
                }
            } else {
                ColorTransitZone z = transitZonesMap.get(targetSourceId);
                if (z != null && !z.getPackages().isEmpty()) {
                    pack = z.getPackages().getFirst();
                    z.removePackage(pack);
                    pack.setState(PackageState.TRANSIT);
                    carriedPackage = pack;
                    broadcast("RELEASE_PICKUP:" + targetSourceId + ":" + getId());
                }
            }

            if (pack != null) {
                if (targetDestIsTransit) {
                    ColorTransitZone tz = transitZonesMap.get(targetDestTransitId);
                    if (tz != null) { destX = tz.getX(); destY = tz.getY(); }
                } else {
                    int gid = (pack.getDestinationGoalId() > 0)
                            ? pack.getDestinationGoalId() : targetGoalId;
                    int[] gp = goalPositions.getOrDefault(gid,
                            goalPositions.values().iterator().next());
                    destX = gp[0]; destY = gp[1];
                    targetGoalId = gid;
                }
                cachedPath = null; // nouvelle destination → invalider le cache
                etat = Etat.TRANSPORT_TO_GOAL;
            } else {
                broadcast("RELEASE_START:"  + targetSourceId + ":" + getId());
                broadcast("RELEASE_PICKUP:" + targetSourceId + ":" + getId());
                if (targetDestTransitId != null)
                    broadcast("RELEASE_DEPOSIT:" + targetDestTransitId + ":" + getId());
                cachedPath = null;
                etat = Etat.IDLE;
            }
        } else {
            moveOneStepTo(destX, destY);
        }
    }

    /** TRANSPORT_TO_GOAL : déplace et dépose le colis à destination. */
    private void handleTransportToGoal() {
        if (isNearZone(destX, destY)) {
            if (targetDestIsTransit) {
                ColorTransitZone z = transitZonesMap.get(targetDestTransitId);
                if (z != null && !z.isFull()) {
                    if (carriedPackage != null) {
                        carriedPackage.setState(PackageState.TRANSIT);
                        z.addPackage(carriedPackage);
                        carriedPackage = null;
                    }
                    broadcast("RELEASE_DEPOSIT:" + targetDestTransitId + ":" + getId());
                    cachedPath = null;
                    etat = Etat.IDLE;
                } else {
                    // Transit plein → chercher autre transit ou livrer directement
                    String oldDepositId = targetDestTransitId;
                    ColorTransitZone alt = findBestTransitZoneWithCapacity();
                    if (alt != null) {
                        int distToAlt = distanceManhattan(getX(), getY(), alt.getX(), alt.getY());
                        myDepositPriority = chargeLevel - distToAlt;
                        broadcast("RELEASE_DEPOSIT:" + oldDepositId + ":" + getId());
                        broadcast("RESERVE_DEPOSIT:" + targetDestTransitId + ":" + getId()
                                + ":" + chargeLevel + ":" + distToAlt);
                        destX = alt.getX(); destY = alt.getY();
                        cachedPath = null;
                    } else {
                        // Issue 6 : vérifier que la batterie permet la livraison directe
                        int gid = (carriedPackage != null && carriedPackage.getDestinationGoalId() > 0)
                                ? carriedPackage.getDestinationGoalId() : targetGoalId;
                        int[] gp = goalPositions.getOrDefault(gid,
                                goalPositions.values().iterator().next());
                        int[] rechargeEst = findClosestRechargePosition();
                        int directCost = distanceManhattan(getX(), getY(), gp[0], gp[1])
                                + (rechargeEst != null
                                    ? distanceManhattan(gp[0], gp[1], rechargeEst[0], rechargeEst[1])
                                    : 0);
                        if (chargeLevel >= directCost) {
                            broadcast("RELEASE_DEPOSIT:" + oldDepositId + ":" + getId());
                            targetDestIsTransit = false;
                            destX = gp[0]; destY = gp[1];
                            cachedPath = null;
                        }
                        // else : batterie insuffisante → attendre ce cycle (transit peut se libérer)
                    }
                }
            } else {
                // Livraison finale au point de sortie
                if (carriedPackage != null) {
                    carriedPackage.setState(PackageState.ARRIVED);
                    deliveredByThisRobot++;
                    MySimFactory.deliveredCount++;
                    MySimFactory.deliveredPerGoal.merge(targetGoalId, 1, Integer::sum);
                    carriedPackage = null;
                }
                cachedPath = null;
                etat = Etat.IDLE;
            }
        } else {
            moveOneStepTo(destX, destY);
        }
    }

    /** MOVING_TO_CHARGE : se déplace jusqu'à la station ; redirige si occupée. */
    private void handleMovingToCharge() {
        if (isAt(destX, destY)) {
            cachedPath = null;
            etat = Etat.CHARGING;
            return;
        }
        moveOneStepTo(destX, destY); // Avance naturellement
    }

    /** CHARGING : recharge la batterie jusqu'à MAX_CHARGE. */
    private void handleCharging() {
        chargeLevel = Math.min(MAX_CHARGE, chargeLevel + CHARGE_GAIN);

        if (chargeLevel == MAX_CHARGE) {
            // 1. Libérer la station pour les autres
            String chargeKey = destX + "," + destY;
            broadcast("RELEASE_CHARGE:" + chargeKey + ":" + getId());
            robotToCharge.remove(getId());

            // 2. Vérifier si on avait mis une livraison en pause
            if (carriedPackage != null) {

                if (targetDestIsTransit) {
                    // On tentait de déposer en transit. On cherche une nouvelle place libre.
                    ColorTransitZone alt = findBestTransitZoneWithCapacity();
                    if (alt != null) {
                        int distToAlt = distanceManhattan(getX(), getY(), alt.getX(), alt.getY());
                        myDepositPriority = chargeLevel - distToAlt;
                        destX = alt.getX(); destY = alt.getY();

                        robotToTransitDeposit.put(getId(), targetDestTransitId);
                        broadcast("RESERVE_DEPOSIT:" + targetDestTransitId + ":" + getId() + ":" + chargeLevel + ":" + distToAlt);

                        etat = Etat.TRANSPORT_TO_GOAL;
                    } else {
                        // Plus aucune place en transit, on prend l'initiative de livrer directement à la sortie !
                        targetDestIsTransit = false;
                        int gid = (carriedPackage.getDestinationGoalId() > 0) ? carriedPackage.getDestinationGoalId() : targetGoalId;
                        int[] gp = goalPositions.getOrDefault(gid, goalPositions.values().iterator().next());
                        destX = gp[0]; destY = gp[1];
                        etat = Etat.TRANSPORT_TO_GOAL;
                    }
                } else {
                    // C'était une livraison directe à la zone de sortie (Z1 ou Z2)
                    int gid = (carriedPackage.getDestinationGoalId() > 0) ? carriedPackage.getDestinationGoalId() : targetGoalId;
                    int[] gp = goalPositions.getOrDefault(gid, goalPositions.values().iterator().next());
                    destX = gp[0]; destY = gp[1];
                    etat = Etat.TRANSPORT_TO_GOAL;
                }

                cachedPath = null; // Important pour calculer le chemin depuis la station jusqu'à la livraison

            } else {
                // Aucun colis en main, on repasse en IDLE pour chercher du travail
                etat = Etat.IDLE;
            }
        }
    }


    // ================================================================== //
    //  step() – boucle principale
    // ================================================================== //
    @Override
    public void step() {
        readMessages();

        // Consommation de batterie lors des déplacements
        // Consommation de batterie
        boolean isWaitingForCharge = (etat == Etat.MOVING_TO_CHARGE && !hasChargePriority());
        boolean isOnRechargeZone = false;
        for (int[] r : rechargePositions) {
            if (r[0] == getX() && r[1] == getY()) {
                isOnRechargeZone = true; break;
            }
        }

        if ((etat == Etat.MOVING_TO_PACKAGE || etat == Etat.TRANSPORT_TO_GOAL || etat == Etat.MOVING_TO_CHARGE)
                && !isWaitingForCharge && !isOnRechargeZone) {
            chargeLevel = Math.max(0, chargeLevel - CHARGE_COST);
        }

        // --- NOUVEAU SEUIL D'URGENCE DYNAMIQUE ---
        int[] closestCharge = findClosestRechargePosition();
        int distToCharge = (closestCharge != null)
                ? distanceManhattan(getX(), getY(), closestCharge[0], closestCharge[1])
                : 0;

        // Seuil d'urgence : la distance réelle + marge pour les détours A*
        int dynamicEmergencyThreshold = distToCharge + 15;

        // Urgence : batterie critique → aller recharger immédiatement
        if (chargeLevel <= dynamicEmergencyThreshold
                && etat != Etat.CHARGING
                && etat != Etat.MOVING_TO_CHARGE) {

            // 1. GESTION DU COLIS (On le garde dans les mains !)
            if (carriedPackage != null) {
                // On a un colis. On annule juste la réservation de notre point de chute
                // pour ne pas bloquer une place de transit pendant qu'on charge.
                if (targetDestIsTransit && targetDestTransitId != null) {
                    broadcast("RELEASE_DEPOSIT:" + targetDestTransitId + ":" + getId());
                    robotToTransitDeposit.remove(getId());
                }
            } else {
                // Pas de colis en main (on allait en chercher un), on peut abandonner la mission
                yieldMission();
            }

            // 2. RECHERCHE DE STATION
            int[] r = findBestAvailableRechargePosition();
            if (r != null) { // Une place est libre !
                destX = r[0]; destY = r[1];
                String key = destX + "," + destY;
                robotToCharge.put(getId(), key);
                broadcast("RESERVE_CHARGE:" + key + ":" + getId() + ":" + chargeLevel);
                etat = Etat.MOVING_TO_CHARGE;
                cachedPath = null;
            } else {
                // TOUT EST PLEIN ! On passe en IDLE pour attendre sagement sur place.
                etat = Etat.IDLE;
            }
        }

        // Issue 2 : mise à jour de l'historique et détection d'oscillation
        updatePositionHistory();
        if (detectOscillation()) {
            cachedPath             = null;
            blockedCell            = null;
            positionHistory.clear();
            // Pause aléatoire de 2-4 cycles pour laisser l'autre robot avancer
            oscillationWaitCounter = 2 + rnd.nextInt(3);
        }

        switch (etat) {
            case IDLE:              handleIdle();             break;
            case MOVING_TO_PACKAGE: handleMovingToPackage();  break;
            case TRANSPORT_TO_GOAL: handleTransportToGoal();  break;
            case MOVING_TO_CHARGE:  handleMovingToCharge();   break;
            case CHARGING:          handleCharging();         break;
        }
    }

    // ================================================================== //
    //  Gestion des messages – arbitrage basé sur batterie / distance
    // ================================================================== //
    @Override
    public void handleMessage(Message msg) {
        if (msg == null || msg.getContent() == null) return;
        String c = msg.getContent();

        // ── RESERVE_START:zoneId:otherId:battery:distance ─────────────────
        if (c.startsWith("RESERVE_START:")) {
            String[] p = c.substring("RESERVE_START:".length()).split(":");
            if (p.length < 4) return;
            String zoneId = p[0]; int otherId = Integer.parseInt(p[1]);
            int otherPri = Integer.parseInt(p[2]) - Integer.parseInt(p[3]);

            robotToStartZone.put(otherId, zoneId); // Enregistre l'intention de l'autre robot

            boolean conflict = zoneId.equals(targetSourceId) && !targetSourceIsTransit && etat == Etat.MOVING_TO_PACKAGE;
            if (conflict) {
                ColorStartZone z = startZonesMap.get(zoneId);
                long resCount = robotToStartZone.values().stream().filter(id -> id.equals(zoneId)).count();

                // On cède UNIQUEMENT s'il y a plus de robots en route que de colis restants
                if (z != null && resCount > z.getPackages().size()) {
                    // Pas de marge de tolérance ici, on compare strictement
                    if (otherPri > mySourcePriority || (otherPri == mySourcePriority && otherId < getId())) {
                        BlockedStart=true; // Plus de changement d'état ici !
                    }
                }
            }

// ── RELEASE_START ─────────────────────────────────
        } else if (c.startsWith("RELEASE_START:")) {
            String[] p = c.substring("RELEASE_START:".length()).split(":", 2);
            if (p.length == 2) {
                try {
                    int rid = Integer.parseInt(p[1]);
                    robotToStartZone.remove(rid); // Ne supprime que SON ID, ne casse pas la zone pour les autres !
                } catch (NumberFormatException ignored) {}
            }

            // ── RESERVE_PICKUP:zoneId:otherId:battery:distance ────────────────
        } else if (c.startsWith("RESERVE_PICKUP:")) {
            String[] p = c.substring("RESERVE_PICKUP:".length()).split(":");
            if (p.length < 4) return;
            String zoneId = p[0]; int otherId = Integer.parseInt(p[1]);
            int otherPri = Integer.parseInt(p[2]) - Integer.parseInt(p[3]);

            robotToTransitPickup.put(otherId, zoneId); // Enregistre l'intention

            boolean conflict = zoneId.equals(targetSourceId) && targetSourceIsTransit && etat == Etat.MOVING_TO_PACKAGE;
            if (conflict) {
                ColorTransitZone z = transitZonesMap.get(zoneId);
                long resCount = robotToTransitPickup.values().stream().filter(id -> id.equals(zoneId)).count();

                // On cède s'il y a plus de robots en route que de colis disponibles
                if (z != null && resCount > z.getPackages().size()) {
                    if (otherPri > mySourcePriority || (otherPri == mySourcePriority && otherId < getId())) {
                        BlockedTransitPickup=true; // Plus de changement d'état ici !
                    }
                }
            }

            // ── RELEASE_PICKUP:zoneId:robotId ────────────────────────────────
        } else if (c.startsWith("RELEASE_PICKUP:")) {
            String[] p = c.substring("RELEASE_PICKUP:".length()).split(":", 2);
            if (p.length == 2) {
                try {
                    int rid = Integer.parseInt(p[1]);
                    robotToTransitPickup.remove(rid); // Libère uniquement SA réservation
                } catch (NumberFormatException ignored) {}
            }

            // ── RESERVE_DEPOSIT:zoneId:otherId:battery:distance ───────────────
        } else if (c.startsWith("RESERVE_DEPOSIT:")) {
            String[] p = c.substring("RESERVE_DEPOSIT:".length()).split(":");
            if (p.length < 4) return;
            String zoneId = p[0]; int otherId = Integer.parseInt(p[1]);
            int otherPri = Integer.parseInt(p[2]) - Integer.parseInt(p[3]);

            robotToTransitDeposit.put(otherId, zoneId); // Enregistre l'intention

            boolean conflict = zoneId.equals(targetDestTransitId)
                    && targetDestIsTransit
                    && (etat == Etat.MOVING_TO_PACKAGE || etat == Etat.TRANSPORT_TO_GOAL);
            if (conflict) {
                long resCount = robotToTransitDeposit.values().stream().filter(id -> id.equals(zoneId)).count();

                // La capacité des zones de transit est de 1. Si plus d'un robot cible la même, on arbitre.
                if (resCount > 1) {
                    if (otherPri > myDepositPriority || (otherPri == myDepositPriority && otherId < getId())) {
                        yieldMission();
                    }
                }
            }

            // ── RELEASE_DEPOSIT:zoneId:robotId ───────────────────────────────
        } else if (c.startsWith("RELEASE_DEPOSIT:")) {
            String[] p = c.substring("RELEASE_DEPOSIT:".length()).split(":", 2);
            if (p.length == 2) {
                try {
                    int rid = Integer.parseInt(p[1]);
                    robotToTransitDeposit.remove(rid); // Libère uniquement SA réservation
                } catch (NumberFormatException ignored) {}
            }

        // ── RESERVE_CHARGE:x,y:otherId:battery ───────────────────────────
            // ── RESERVE_CHARGE:x,y:otherId:battery ───────────────────────────
        } else if (c.startsWith("RESERVE_CHARGE:")) {
            String[] p = c.substring("RESERVE_CHARGE:".length()).split(":");
            if (p.length < 3) return;
            robotToCharge.put(Integer.parseInt(p[1]), p[0]);

            // ── RELEASE_CHARGE:x,y:robotId ───────────────────────────────────────────
        } else if (c.startsWith("RELEASE_CHARGE:")) {
            String[] p = c.substring("RELEASE_CHARGE:".length()).split(":", 2);
            if (p.length == 2) {
                try {
                    robotToCharge.remove(Integer.parseInt(p[1]));
                } catch (NumberFormatException ignored) {}
            }
        }
    }

    @Override
    public void move(int step) {
        step();
    }

    // ================================================================== //
    //  Diagnostic
    // ================================================================== //

    @Override
    public String toString() {
        return getName() + " [" + etat + "] bat=" + chargeLevel
                + " pos=(" + getX() + "," + getY() + ")"
                + (carriedPackage != null ? " porte=1" : "");
    }

    private void updateReservationIfBetter(String zoneId, int id, int pri, Map<String, Integer> resMap, Map<String, Integer> priMap) {
        int currentPri = priMap.getOrDefault(zoneId, Integer.MIN_VALUE);
        if (pri > currentPri || (pri == currentPri && id < resMap.getOrDefault(zoneId, Integer.MAX_VALUE))) {
            resMap.put(zoneId, id);
            priMap.put(zoneId, pri);
        }
    }
}
