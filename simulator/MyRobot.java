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
 *
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
    private  static final int EMERGENCY_THRESHOLD = 5; // seuil d'urgence

    // ------------------------------------------------------------------ //
    //  Connaissances (zones)
    // ------------------------------------------------------------------ //
    protected Map<String, ColorStartZone>   startZonesMap;
    protected Map<String, ColorTransitZone> transitZonesMap;
    protected Map<Integer, int[]>           goalPositions;   // id → [x,y]
    protected List<int[]>                   rechargePositions;

    // ------------------------------------------------------------------ //
    //  Réservations (zone_id → id du robot réservant)
    // ------------------------------------------------------------------ //
    private final Map<String, Integer> reservedStartZones    = new HashMap<>();
    private final Map<String, Integer> reservedTransitPickup = new HashMap<>();
    private final Map<String, Integer> reservedTransitDeposit= new HashMap<>();
    private final Map<String, Integer> reservedRechargeZones = new HashMap<>();

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

    /** Marge minimale d'avantage pour déclencher un yieldMission(). */
    private static final int YIELD_MARGIN = 5;

    /** Compteur de cycles consécutifs sans mouvement (anti-blocage). */
    private int stuckCounter = 0;

    /** Nombre de cycles consécutifs en IDLE sans mission trouvée. */
    private int idleNoMissionCycles = 0;

    /**
     * Seuil au-delà duquel les maps de réservation sont purgées pour casser
     * un éventuel deadlock dû à des réservations périmées.
     */
    private static final int IDLE_STALE_THRESHOLD = 20;

    /** Nombre de livraisons accomplies par ce robot. */
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
    private final List<int[]> positionHistory       = new ArrayList<>();
    private static final int  POSITION_HISTORY_SIZE = 6;
    private int               oscillationWaitCounter = 0;

    // ------------------------------------------------------------------ //
    //  Issues 3 & 8 : Cellule bloquante + obstacles temporaires
    // ------------------------------------------------------------------ //
    private int[] blockedCell              = null;
    private int   mutualBlockTimer         = 0;
    private static final int MUTUAL_BLOCK_THRESHOLD = 3;

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

    /** Zone de départ la plus proche, non vide et non réservée. Met à jour targetSourceId. */
    protected ColorStartZone findBestStartZone() {
        ColorStartZone best = null;
        int minDist = Integer.MAX_VALUE;
        for (Map.Entry<String, ColorStartZone> e : startZonesMap.entrySet()) {
            ColorStartZone z = e.getValue();
            if (!z.getPackages().isEmpty() && !reservedStartZones.containsKey(e.getKey())) {
                int d = distanceManhattan(getX(), getY(), z.getX(), z.getY());
                if (d < minDist) { minDist = d; best = z; targetSourceId = e.getKey(); }
            }
        }
        return best;
    }

    /** Zone de transit la plus proche avec de la capacité, non réservée pour dépôt. Met à jour targetDestTransitId. */
    protected ColorTransitZone findBestTransitZoneWithCapacity() {
        ColorTransitZone best = null;
        int minDist = Integer.MAX_VALUE;
        for (Map.Entry<String, ColorTransitZone> e : transitZonesMap.entrySet()) {
            ColorTransitZone z = e.getValue();
            if (!z.isFull() && !reservedTransitDeposit.containsKey(e.getKey())) {
                int d = distanceManhattan(getX(), getY(), z.getX(), z.getY());
                if (d < minDist) { minDist = d; best = z; targetDestTransitId = e.getKey(); }
            }
        }
        return best;
    }

    /** Zone de transit la plus proche non vide, non réservée pour ramassage. Met à jour targetSourceId. */
    protected ColorTransitZone findBestTransitZoneWithPackages() {
        ColorTransitZone best = null;
        int minDist = Integer.MAX_VALUE;
        for (Map.Entry<String, ColorTransitZone> e : transitZonesMap.entrySet()) {
            ColorTransitZone z = e.getValue();
            if (!z.getPackages().isEmpty() && !reservedTransitPickup.containsKey(e.getKey())) {
                int d = distanceManhattan(getX(), getY(), z.getX(), z.getY());
                if (d < minDist) { minDist = d; best = z; targetSourceId = e.getKey(); }
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

        // Passe 1 : non réservée ET cellule physiquement libre
        int minDist = Integer.MAX_VALUE;
        for (int[] pos : rechargePositions) {
            String key = pos[0] + "," + pos[1];
            if (reservedRechargeZones.containsKey(key)) continue;
            Cell c = env.getGrid()[pos[0]][pos[1]];
            if (c != null && c.getContent() != null) continue;
            int d = distanceManhattan(getX(), getY(), pos[0], pos[1]);
            if (d < minDist) { minDist = d; best = pos; }
        }
        if (best != null) return best;

        // Passe 2 : non réservée (peut être en route)
        int minDist2 = Integer.MAX_VALUE;
        for (int[] pos : rechargePositions) {
            String key = pos[0] + "," + pos[1];
            if (reservedRechargeZones.containsKey(key)) continue;
            int d = distanceManhattan(getX(), getY(), pos[0], pos[1]);
            if (d < minDist2) { minDist2 = d; best = pos; }
        }
        if (best != null) return best;

        // Passe 3 : cellule physiquement libre (réservation peut être périmée)
        int minDist3 = Integer.MAX_VALUE;
        for (int[] pos : rechargePositions) {
            Cell c = env.getGrid()[pos[0]][pos[1]];
            if (c != null && c.getContent() != null) continue;
            int d = distanceManhattan(getX(), getY(), pos[0], pos[1]);
            if (d < minDist3) { minDist3 = d; best = pos; }
        }
        if (best != null) return best;

        // Passe 4 : fallback absolu
        return findClosestRechargePosition();
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
            }
        }
        return obs;
    }

    // ================================================================== //
    //  Issue 2 : Historique de position et détection d'oscillation
    // ================================================================== //

    /**
     * Enregistre la position courante dans l'historique,
     * UNIQUEMENT si elle a changé depuis la dernière entrée.
     *
     * Correction du bug critique : sans ce garde, un robot stationnaire (IDLE)
     * remplit l'historique avec la même position, ce qui déclenche une fausse
     * détection d'oscillation dès qu'il reçoit une mission. Le compteur
     * oscillationWaitCounter se retrouve alors réinitialisé en boucle, bloquant
     * le robot à jamais dans MOVING_TO_PACKAGE.
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

    // ================================================================== //
    //  Déplacement
    // ================================================================== //

    /**
     * Déplace le robot d'un pas vers (targetX, targetY) via A*.
     *
     * Améliorations :
     *  1. Cache du chemin A* : recalcule uniquement quand la cible change ou que
     *     le chemin est invalidé (position hors cache, blocage persistant).
     *  2. Obstacle temporaire : la cellule bloquante (blockedCell) est passée en
     *     extra-obstacle au PathFinder après MUTUAL_BLOCK_THRESHOLD cycles bloqués,
     *     forçant un contournement.
     *  3. Compteur d'attente oscillation : pause de quelques cycles après détection.
     */
    protected void moveOneStepTo(int targetX, int targetY) {
        if (isAt(targetX, targetY)) { cachedPath = null; return; }

        // Pause forcée après détection d'oscillation
        if (oscillationWaitCounter > 0) {
            oscillationWaitCounter--;
            return;
        }

        // Invalider le cache si la cible a changé
        if (targetX != lastTargetX || targetY != lastTargetY) {
            cachedPath   = null;
            lastTargetX  = targetX;
            lastTargetY  = targetY;
        }

        // Valider le cache : la position courante doit coïncider avec path[cachedPathIndex]
        if (cachedPath != null && (cachedPathIndex >= cachedPath.length
                || cachedPath[cachedPathIndex][0] != getX()
                || cachedPath[cachedPathIndex][1] != getY())) {
            cachedPath = null;
        }

        // Recalcul du chemin si nécessaire
        if (cachedPath == null) {
            boolean[][] obs = buildStaticObstacleMap();
            // Issue 8 : passer la cellule bloquante comme obstacle temporaire
            List<int[]> extra = (blockedCell != null)
                    ? Collections.singletonList(blockedCell) : null;

            boolean targetIsObstacle = targetX >= 0 && targetX < rows
                    && targetY >= 0 && targetY < columns && obs[targetX][targetY];

            if (targetIsObstacle) {
                List<int[]> adjList = new ArrayList<>();
                for (int[] d : new int[][]{{-1,0},{1,0},{0,-1},{0,1}}) {
                    int nx = targetX + d[0], ny = targetY + d[1];
                    if (nx >= 0 && nx < rows && ny >= 0 && ny < columns && !obs[nx][ny])
                        adjList.add(new int[]{nx, ny});
                }
                if (adjList.isEmpty()) return;
                for (int[] adj : adjList) {
                    if (isAt(adj[0], adj[1])) { cachedPath = null; return; }
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
        }

        // Invalider le chemin si length insuffisante ou index en débordement
        if (cachedPath != null && (cachedPath.length < 2 || cachedPathIndex + 1 >= cachedPath.length)) {
            cachedPath = null;
        }
        if (cachedPath == null) {
            // Aucun chemin trouvé (même avec contournement)
            if (++stuckCounter > 3) {
                stuckCounter  = 0;
                blockedCell   = null;
                tryRandomStep(buildStaticObstacleMap());
            }
            return;
        }

        int nextX = cachedPath[cachedPathIndex + 1][0];
        int nextY = cachedPath[cachedPathIndex + 1][1];

        // Orientation vers la prochaine cellule
        int dx = nextX - getX(), dy = nextY - getY();
        if      (dx == -1) orientation = Orientation.up;
        else if (dx ==  1) orientation = Orientation.down;
        else if (dy == -1) orientation = Orientation.left;
        else if (dy ==  1) orientation = Orientation.right;

        // Vérification temps-réel : obstacle dynamique (autre robot / worker)
        Cell nextCell = env.getGrid()[nextX][nextY];
        if (nextCell != null && nextCell.getContent() != null) {
            // Issue 3 : suivre la cellule bloquante
            blockedCell = new int[]{nextX, nextY};
            mutualBlockTimer++;
            if (mutualBlockTimer >= MUTUAL_BLOCK_THRESHOLD) {
                // Forcer recomputation avec la cellule bloquante comme obstacle
                cachedPath       = null;
                mutualBlockTimer = 0;
            }
            if (++stuckCounter > 5) {
                stuckCounter = 0;
                blockedCell  = null;
                tryRandomStep(buildStaticObstacleMap());
            }
            return;
        }

        // Mouvement réussi
        stuckCounter     = 0;
        mutualBlockTimer = 0;
        blockedCell      = null;
        cachedPathIndex++;
        moveForward();
    }


    /**
     * Issue 10 : Choisit une cellule adjacente libre pour se désengager d'un blocage.
     * Préfère la cellule qui s'éloigne le plus de la cellule bloquante.
     * Fallback : cellule libre aléatoire.
     */
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
        if (otherPriority > myPriority + YIELD_MARGIN) return true;
        if (otherPriority == myPriority)               return otherId < getId();
        return false;
    }

    // ================================================================== //
    //  Abandon de mission (priorité perdue)
    // ================================================================== //

    /**
     * Abandonne la mission courante et libère ses propres réservations.
     *
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
        // ── Bug 3 : purge des réservations périmées ──────────────────────────
        // Si le robot reste IDLE trop longtemps sans trouver de mission, les maps
        // de réservation contiennent probablement des entrées périmées (RELEASE
        // manqué). On les efface pour débloquer la situation.
        if (idleNoMissionCycles > IDLE_STALE_THRESHOLD) {
            System.out.println(getName() + " [WARN] Réservations périmées purgées (idle="
                    + idleNoMissionCycles + ")");
            reservedStartZones.clear();
            reservedTransitPickup.clear();
            reservedTransitDeposit.clear();
            reservedRechargeZones.clear();
            idleNoMissionCycles = 0;
        }

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
                if (chargeLevel >= cost) {
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

            // ---- Cas 2 : trajet initial  départ → transit → recharge ----
            ColorTransitZone tz = findBestTransitZoneWithCapacity();
            if (tz != null) {
                int tx = tz.getX(), ty = tz.getY();
                int distToTransitViaStart = distToStart + distanceManhattan(sx, sy, tx, ty);
                int cost = distToStart
                         + distanceManhattan(sx, sy, tx, ty)
                         + distanceManhattan(tx, ty, recharge[0], recharge[1]);
                if (chargeLevel >= cost) {
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
                if (chargeLevel >= cost) {
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
        if (chargeLevel < MAX_CHARGE) {
            int[] rechargeTarget = findBestAvailableRechargePosition();
            if (rechargeTarget != null) {
                destX = rechargeTarget[0];
                destY = rechargeTarget[1];
                myChargePriority = MAX_CHARGE - chargeLevel;
                broadcast("RESERVE_CHARGE:" + destX + "," + destY + ":" + getId()
                        + ":" + chargeLevel);
                etat = Etat.MOVING_TO_CHARGE;
                idleNoMissionCycles = 0;
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

        Cell targetCell = env.getGrid()[destX][destY];
        if (targetCell != null && targetCell.getContent() != null) {
            String oldKey = destX + "," + destY;
            Integer savedVal = reservedRechargeZones.put(oldKey, Integer.MAX_VALUE);
            int[] alt = findBestAvailableRechargePosition();
            if (savedVal == null) reservedRechargeZones.remove(oldKey);
            else                  reservedRechargeZones.put(oldKey, savedVal);

            if (alt != null && !(alt[0] == destX && alt[1] == destY)) {
                broadcast("RELEASE_CHARGE:" + oldKey);
                destX = alt[0];
                destY = alt[1];
                myChargePriority = MAX_CHARGE - chargeLevel;
                broadcast("RESERVE_CHARGE:" + destX + "," + destY + ":" + getId()
                        + ":" + chargeLevel);
                cachedPath = null;
            }
            return;
        }

        moveOneStepTo(destX, destY);
    }

    /** CHARGING : recharge la batterie jusqu'à MAX_CHARGE. */
    private void handleCharging() {
        chargeLevel = Math.min(MAX_CHARGE, chargeLevel + CHARGE_GAIN);
        if (chargeLevel == MAX_CHARGE) {
            broadcast("RELEASE_CHARGE:" + destX + "," + destY);
            etat = Etat.IDLE;
        }
    }

    // ================================================================== //
    //  step() – boucle principale
    // ================================================================== //
    @Override
    public void step() {
        readMessages();

        // Consommation de batterie lors des déplacements
        if (etat == Etat.MOVING_TO_PACKAGE
                || etat == Etat.TRANSPORT_TO_GOAL
                || etat == Etat.MOVING_TO_CHARGE) {
            chargeLevel = Math.max(0, chargeLevel - CHARGE_COST);
        }

        // Urgence : batterie critique → abandonner mission et recharger
        if (chargeLevel <= EMERGENCY_THRESHOLD
                && etat != Etat.CHARGING
                && etat != Etat.MOVING_TO_CHARGE) {
            if (carriedPackage != null) carriedPackage = null;
            yieldMission();
            int[] r = findBestAvailableRechargePosition();
            if (r != null) {
                destX = r[0]; destY = r[1];
                myChargePriority = MAX_CHARGE - chargeLevel;
                broadcast("RESERVE_CHARGE:" + destX + "," + destY + ":" + getId()
                        + ":" + chargeLevel);
                etat  = Etat.MOVING_TO_CHARGE;
                idleNoMissionCycles = 0;
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

            boolean conflict = zoneId.equals(targetSourceId)
                    && !targetSourceIsTransit && etat == Etat.MOVING_TO_PACKAGE;
            if (conflict) {
                if (shouldYield(otherId, otherPri, mySourcePriority)) {
                    yieldMission(); reservedStartZones.put(zoneId, otherId);
                }
            } else { reservedStartZones.put(zoneId, otherId); }

        // ── RELEASE_START:zoneId:robotId ─────────────────────────────────
        } else if (c.startsWith("RELEASE_START:")) {
            String[] p = c.substring("RELEASE_START:".length()).split(":", 2);
            if (p.length == 2) {
                // Format moderne avec ID : ne libère que si c'est le bon détenteur
                try {
                    int rid = Integer.parseInt(p[1]);
                    if (reservedStartZones.getOrDefault(p[0], -1) == rid)
                        reservedStartZones.remove(p[0]);
                } catch (NumberFormatException ignored) {}
            } else {
                reservedStartZones.remove(p[0]);
            }

        // ── RESERVE_PICKUP:zoneId:otherId:battery:distance ────────────────
        } else if (c.startsWith("RESERVE_PICKUP:")) {
            String[] p = c.substring("RESERVE_PICKUP:".length()).split(":");
            if (p.length < 4) return;
            String zoneId = p[0]; int otherId = Integer.parseInt(p[1]);
            int otherPri = Integer.parseInt(p[2]) - Integer.parseInt(p[3]);

            boolean conflict = zoneId.equals(targetSourceId)
                    && targetSourceIsTransit && etat == Etat.MOVING_TO_PACKAGE;
            if (conflict) {
                if (shouldYield(otherId, otherPri, mySourcePriority)) {
                    yieldMission(); reservedTransitPickup.put(zoneId, otherId);
                }
            } else { reservedTransitPickup.put(zoneId, otherId); }

        // ── RELEASE_PICKUP:zoneId:robotId ────────────────────────────────
        } else if (c.startsWith("RELEASE_PICKUP:")) {
            String[] p = c.substring("RELEASE_PICKUP:".length()).split(":", 2);
            if (p.length == 2) {
                try {
                    int rid = Integer.parseInt(p[1]);
                    if (reservedTransitPickup.getOrDefault(p[0], -1) == rid)
                        reservedTransitPickup.remove(p[0]);
                } catch (NumberFormatException ignored) {}
            } else {
                reservedTransitPickup.remove(p[0]);
            }

        // ── RESERVE_DEPOSIT:zoneId:otherId:battery:distance ───────────────
        } else if (c.startsWith("RESERVE_DEPOSIT:")) {
            String[] p = c.substring("RESERVE_DEPOSIT:".length()).split(":");
            if (p.length < 4) return;
            String zoneId = p[0]; int otherId = Integer.parseInt(p[1]);
            int otherPri = Integer.parseInt(p[2]) - Integer.parseInt(p[3]);

            boolean conflict = zoneId.equals(targetDestTransitId)
                    && targetDestIsTransit
                    && (etat == Etat.MOVING_TO_PACKAGE || etat == Etat.TRANSPORT_TO_GOAL);
            if (conflict) {
                if (shouldYield(otherId, otherPri, myDepositPriority)) {
                    yieldMission(); reservedTransitDeposit.put(zoneId, otherId);
                }
            } else { reservedTransitDeposit.put(zoneId, otherId); }

        // ── RELEASE_DEPOSIT:zoneId:robotId ───────────────────────────────
        } else if (c.startsWith("RELEASE_DEPOSIT:")) {
            String[] p = c.substring("RELEASE_DEPOSIT:".length()).split(":", 2);
            if (p.length == 2) {
                try {
                    int rid = Integer.parseInt(p[1]);
                    if (reservedTransitDeposit.getOrDefault(p[0], -1) == rid)
                        reservedTransitDeposit.remove(p[0]);
                } catch (NumberFormatException ignored) {}
            } else {
                reservedTransitDeposit.remove(p[0]);
            }

        // ── RESERVE_CHARGE:x,y:otherId:battery ───────────────────────────
        } else if (c.startsWith("RESERVE_CHARGE:")) {
            String[] p = c.substring("RESERVE_CHARGE:".length()).split(":");
            if (p.length < 3) return;
            String key = p[0]; int otherId = Integer.parseInt(p[1]);
            int otherBattery   = Integer.parseInt(p[2]);
            int otherChargePri = MAX_CHARGE - otherBattery;

            boolean conflict = key.equals(destX + "," + destY)
                    && etat == Etat.MOVING_TO_CHARGE;
            if (conflict) {
                if (shouldYield(otherId, otherChargePri, myChargePriority)) {
                    yieldMission(); reservedRechargeZones.put(key, otherId);
                }
            } else { reservedRechargeZones.put(key, otherId); }

        // ── RELEASE_CHARGE:x,y ───────────────────────────────────────────
        } else if (c.startsWith("RELEASE_CHARGE:")) {
            reservedRechargeZones.remove(c.substring("RELEASE_CHARGE:".length()));
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
}
