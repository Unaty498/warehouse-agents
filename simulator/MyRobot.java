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

    // Temps de départ/arrivée (pour statistiques futures)
    protected long tempsDepart;
    protected long tempsArrivee;

    // Verbose (affichage de debug dans la console)
    protected boolean verbose;

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
    //  Format messages : RESERVE_START:A1:3  /  RELEASE_START:A1
    //                    RESERVE_PICKUP:T1:3 /  RELEASE_PICKUP:T1   (ramassage transit)
    //                    RESERVE_DEPOSIT:T1:3/ RELEASE_DEPOSIT:T1   (dépôt en transit)
    //                    RESERVE_CHARGE:x,y:3/ RELEASE_CHARGE:x,y
    //  Protocole priorité : ID le plus bas gagne ; le perdant appelle yieldMission().
    // ------------------------------------------------------------------ //
    private final Map<String, Integer> reservedStartZones    = new HashMap<>(); // start → robot_id
    private final Map<String, Integer> reservedTransitPickup = new HashMap<>(); // transit pickup → robot_id
    private final Map<String, Integer> reservedTransitDeposit= new HashMap<>(); // transit deposit → robot_id
    private final Map<String, Integer> reservedRechargeZones = new HashMap<>(); // "x,y" → robot_id

    // ------------------------------------------------------------------ //
    //  Suivi de mission courante
    // ------------------------------------------------------------------ //
    private String  targetSourceId;         // ID zone source (A1, T1, …)
    private String  targetDestTransitId;    // ID zone transit destination
    private boolean targetSourceIsTransit;  // true : ramasse dans transit
    private boolean targetDestIsTransit;    // true : livre dans transit
    private int     targetGoalId;           // ID goal de livraison (1 ou 2)

    // ------------------------------------------------------------------ //
    //  Priorités de mission (calculées au moment de la réservation)
    //  Zone    : score = batterie − distance_à_la_zone  (plus grand = meilleur)
    //  Recharge: score = MAX_CHARGE − batterie          (plus grand = plus urgent)
    //  Arbitrage final en cas d'égalité : ID le plus bas gagne.
    // ------------------------------------------------------------------ //
    private int mySourcePriority  = Integer.MIN_VALUE;
    private int myDepositPriority = Integer.MIN_VALUE;
    private int myChargePriority  = Integer.MIN_VALUE;

    /** Marge minimale d'avantage pour déclencher un yieldMission(). */
    private static final int YIELD_MARGIN = 5;

    /** Compteur de cycles consécutifs sans mouvement (anti-blocage). */
    private int stuckCounter = 0;

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
     * Accepte distance Manhattan ≤ 1 :
     *  - distance 0 : le robot est SUR la cellule (possible pour les goals non-obstacles)
     *  - distance 1 : le robot est dans l'un des 4 voisins orthogonaux (N/S/E/W)
     * Cela garantit que le pickup et le dépôt fonctionnent depuis n'importe quel
     * voisin orthogonal, quelle que soit la direction d'approche.
     */
    protected boolean isNearZone(int row, int col) {
        return distanceManhattan(getX(), getY(), row, col) <= 1;
    }

    /** Conservé pour compatibilité (vérifie distance == 1 strictement). */
    protected boolean isAdjacentTo(int row, int col) {
        return distanceManhattan(getX(), getY(), row, col) == 1;
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
     * Ignore les réservations (on veut la distance minimale théorique).
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
     * Passe 1 : non réservée ET cellule libre.
     * Passe 2 : non réservée (peut être en route).
     * Passe 3 : cellule physiquement libre (même si "réservée" — réservation peut être périmée).
     * Passe 4 : fallback absolu (la plus proche).
     */
    protected int[] findBestAvailableRechargePosition() {
        int[] best = null;
        int minDist;

        // Passe 1 : non réservée ET cellule physiquement libre
        minDist = Integer.MAX_VALUE;
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
        minDist = Integer.MAX_VALUE;
        for (int[] pos : rechargePositions) {
            String key = pos[0] + "," + pos[1];
            if (reservedRechargeZones.containsKey(key)) continue;
            int d = distanceManhattan(getX(), getY(), pos[0], pos[1]);
            if (d < minDist) { minDist = d; best = pos; }
        }
        if (best != null) return best;

        // Passe 3 : cellule physiquement libre (réservation peut être périmée)
        minDist = Integer.MAX_VALUE;
        for (int[] pos : rechargePositions) {
            Cell c = env.getGrid()[pos[0]][pos[1]];
            if (c != null && c.getContent() != null) continue;
            int d = distanceManhattan(getX(), getY(), pos[0], pos[1]);
            if (d < minDist) { minDist = d; best = pos; }
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
     * Exclut volontairement les autres robots :
     *   - Inclure les robots provoque des oscillations (le chemin change à chaque
     *     cycle selon les voisins en mouvement, causant des zigzags perpétuels).
     *   - La présence d'un robot sur la prochaine cellule est vérifiée en temps
     *     réel juste avant moveForward() ; le robot attend simplement ce cycle.
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
    //  Déplacement
    // ================================================================== //

    /**
     * Déplace le robot d'un pas vers (targetX, targetY) via A*.
     *
     * Stratégie :
     *  1. Carte statique uniquement pour A* → chemin stable, pas d'oscillation.
     *  2. Si la cellule cible est un obstacle statique (zone), on vise une
     *     cellule adjacente libre (la plus proche du robot).
     *  3. Vérification temps-réel sur la cellule suivante avant moveForward() :
     *     si un autre robot s'y trouve, on attend ce cycle (pas de mouvement aléatoire).
     *  4. Anti-blocage : après stuckCounter > 5 cycles immobiles consécutifs,
     *     on choisit un pas latéral libre pour se désengager.
     */
    protected void moveOneStepTo(int targetX, int targetY) {
        if (isAt(targetX, targetY)) return;

        boolean[][] obs = buildStaticObstacleMap();

        int[][] path;

        boolean targetIsObstacle = targetX >= 0 && targetX < rows
                && targetY >= 0 && targetY < columns && obs[targetX][targetY];

        if (targetIsObstacle) {
            // ---- cible = zone obstacle : collecter les 4 voisins libres comme buts ----
            // Utiliser A* multi-cibles : le robot trouve le chemin optimal vers
            // n'importe lequel des voisins orthogonaux accessibles, quelle que
            // soit la direction d'approche.
            List<int[]> adjList = new ArrayList<>();
            for (int[] d : new int[][]{{-1,0},{1,0},{0,-1},{0,1}}) {
                int nx = targetX + d[0], ny = targetY + d[1];
                if (nx >= 0 && nx < rows && ny >= 0 && ny < columns && !obs[nx][ny])
                    adjList.add(new int[]{nx, ny});
            }
            if (adjList.isEmpty()) return; // zone entièrement entourée d'obstacles statiques

            // Si déjà dans l'un des voisins → pas besoin de bouger
            for (int[] adj : adjList) {
                if (isAt(adj[0], adj[1])) return;
            }

            path = PathFinder.aStarToAny(
                    new int[]{getX(), getY()},
                    adjList.toArray(new int[0][]),
                    rows, columns, obs);
        } else {
            // ---- cible = cellule libre (goal position, station de recharge…) ----
            if (isAt(targetX, targetY)) return;
            path = PathFinder.aStar(
                    new int[]{getX(), getY()},
                    new int[]{targetX, targetY},
                    rows, columns, obs);
        }

        if (path == null || path.length < 2) {
            if (++stuckCounter > 3) { stuckCounter = 0; tryRandomStep(obs); }
            return;
        }

        int nextX = path[1][0];
        int nextY = path[1][1];

        // Orientation vers la prochaine cellule
        int dx = nextX - getX(), dy = nextY - getY();
        if      (dx == -1) orientation = Orientation.up;
        else if (dx ==  1) orientation = Orientation.down;
        else if (dy == -1) orientation = Orientation.left;
        else if (dy ==  1) orientation = Orientation.right;

        // Vérification temps-réel : obstacle dynamique (autre robot / worker)
        Cell nextCell = env.getGrid()[nextX][nextY];
        if (nextCell != null && nextCell.getContent() != null) {
            if (++stuckCounter > 5) { stuckCounter = 0; tryRandomStep(obs); }
            return;
        }

        stuckCounter = 0;
        moveForward();
    }

    /**
     * Retourne la cellule adjacente à (zoneX, zoneY) la plus proche du robot,
     * non bloquée dans la carte statique.
     */
    private int[] findBestAdjacentFreeCell(int zoneX, int zoneY, boolean[][] obs) {
        int[][] dirs = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
        int[] best = null;
        int minDist = Integer.MAX_VALUE;
        for (int[] d : dirs) {
            int nx = zoneX + d[0], ny = zoneY + d[1];
            if (nx >= 0 && nx < rows && ny >= 0 && ny < columns && !obs[nx][ny]) {
                int dist = distanceManhattan(getX(), getY(), nx, ny);
                if (dist < minDist) { minDist = dist; best = new int[]{nx, ny}; }
            }
        }
        return best;
    }

    /**
     * Choisit aléatoirement une cellule adjacente libre (statiquement et dynamiquement)
     * et s'y déplace. Utilisé pour briser un blocage persistant.
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
        int[] chosen = candidates.get(rnd.nextInt(candidates.size()));
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
     * L'autre gagne si son score dépasse le mien d'au moins YIELD_MARGIN,
     * ou si les scores sont égaux et son ID est inférieur au mien.
     *
     * @param otherId       ID de l'autre robot
     * @param otherPriority score de priorité de l'autre (batterie−distance ou MAX−batterie)
     * @param myPriority    score de priorité de ce robot (stocké au moment de la réservation)
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
     * Abandonne la mission courante et libère toutes les réservations associées.
     * Appelé quand un robot de priorité supérieure (ID inférieur) revendique
     * la même ressource dans le cycle précédent.
     */
    private void yieldMission() {
        switch (etat) {
            case MOVING_TO_PACKAGE:
                if (targetSourceId != null) {
                    if (targetSourceIsTransit) broadcast("RELEASE_PICKUP:" + targetSourceId);
                    else                       broadcast("RELEASE_START:"  + targetSourceId);
                }
                if (targetDestIsTransit && targetDestTransitId != null)
                    broadcast("RELEASE_DEPOSIT:" + targetDestTransitId);
                break;
            case TRANSPORT_TO_GOAL:
                // Libérer la réservation de dépôt transit si abandonnée (ex : urgence batterie)
                if (targetDestIsTransit && targetDestTransitId != null)
                    broadcast("RELEASE_DEPOSIT:" + targetDestTransitId);
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
                // Pas de facteur ×1.5 : cohérent avec les cas 2 et 3, évite le "no-man's land"
                // où batterie ∈ [80%, coût×1.5) → robot bloqué en IDLE sans se recharger.
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
                    return;
                }
            }
        }

        // ---- Cas 4 : recharge (seulement si besoin réel) ----
        if (chargeLevel < MAX_CHARGE * 0.8) {
            int[] rechargeTarget = findBestAvailableRechargePosition();
            if (rechargeTarget != null) {
                destX = rechargeTarget[0];
                destY = rechargeTarget[1];
                myChargePriority = MAX_CHARGE - chargeLevel; // urgence : batterie basse → score élevé
                broadcast("RESERVE_CHARGE:" + destX + "," + destY + ":" + getId()
                        + ":" + chargeLevel);
                etat = Etat.MOVING_TO_CHARGE;
            }
        }
        // batterie suffisante + aucune tâche → IDLE, les zones se libèreront au prochain cycle
    }

    /** MOVING_TO_PACKAGE : se déplace puis ramasse le colis quand proche de la zone source. */
    private void handleMovingToPackage() {
        if (isNearZone(destX, destY)) {
            ColorPackage pack = null;

            if (!targetSourceIsTransit) {
                // Ramasse dans zone de départ
                ColorStartZone z = startZonesMap.get(targetSourceId);
                if (z != null && !z.getPackages().isEmpty()) {
                    pack = z.getPackages().getFirst();
                    z.removePackage(pack);
                    pack.setState(PackageState.DEPART);
                    carriedPackage = pack;
                    broadcast("RELEASE_START:" + targetSourceId);
                }
            } else {
                // Ramasse dans zone de transit
                ColorTransitZone z = transitZonesMap.get(targetSourceId);
                if (z != null && !z.getPackages().isEmpty()) {
                    pack = z.getPackages().getFirst();
                    z.removePackage(pack);
                    pack.setState(PackageState.TRANSIT);
                    carriedPackage = pack;
                    broadcast("RELEASE_PICKUP:" + targetSourceId);
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
                etat = Etat.TRANSPORT_TO_GOAL;
            } else {
                // Zone vide entre-temps → annuler réservations et revenir à IDLE
                broadcast("RELEASE_START:"   + targetSourceId);
                broadcast("RELEASE_PICKUP:"  + targetSourceId);
                broadcast("RELEASE_DEPOSIT:" + targetDestTransitId);
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
                    broadcast("RELEASE_DEPOSIT:" + targetDestTransitId);
                    etat = Etat.IDLE;
                } else {
                    // Transit plein → chercher autre transit ou livrer directement
                    String oldDepositId = targetDestTransitId;
                    ColorTransitZone alt = findBestTransitZoneWithCapacity(); // met à jour targetDestTransitId
                    if (alt != null) {
                        broadcast("RELEASE_DEPOSIT:" + oldDepositId);
                        broadcast("RESERVE_DEPOSIT:" + targetDestTransitId + ":" + getId());
                        destX = alt.getX(); destY = alt.getY();
                    } else {
                        broadcast("RELEASE_DEPOSIT:" + oldDepositId);
                        targetDestIsTransit = false;
                        int gid = (carriedPackage != null && carriedPackage.getDestinationGoalId() > 0)
                                ? carriedPackage.getDestinationGoalId() : targetGoalId;
                        int[] gp = goalPositions.getOrDefault(gid,
                                goalPositions.values().iterator().next());
                        destX = gp[0]; destY = gp[1];
                    }
                }
            } else {
                // Livraison finale au point de sortie
                if (carriedPackage != null) {
                    carriedPackage.setState(PackageState.ARRIVED);
                    MySimFactory.deliveredCount++;
                    System.out.println(getName() + " → livraison #"
                            + MySimFactory.deliveredCount + " (batterie=" + chargeLevel + ")");
                    carriedPackage = null;
                }
                etat = Etat.IDLE;
            }
        } else {
            moveOneStepTo(destX, destY);
        }
    }

    /** MOVING_TO_CHARGE : se déplace jusqu'à la station ; redirige si occupée. */
    private void handleMovingToCharge() {
        if (isAt(destX, destY)) {
            etat = Etat.CHARGING;
            return;
        }

        // Si la station cible est occupée par un autre agent (robot/travailleur),
        // chercher une alternative pour éviter l'interblocage.
        Cell targetCell = env.getGrid()[destX][destY];
        if (targetCell != null && targetCell.getContent() != null) {
            String oldKey = destX + "," + destY;
            // Masquer temporairement la station pour forcer le filtre dans findBest...
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
            }
            // Sinon : aucune alternative → attendre que la station se libère
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

        if (debug == 1) {
            System.out.println(getName() + " | Etat: " + etat
                    + " | Position: (" + getX() + "," + getY() + ")"
                    + " | Batterie: " + chargeLevel
                    + " | Colis: " + (carriedPackage != null ? carriedPackage.getId() : "None"));

        }

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
            // Libérer les réservations de la mission abandonnée
            yieldMission();
            int[] r = findBestAvailableRechargePosition();
            if (r != null) {
                destX = r[0]; destY = r[1];
                myChargePriority = MAX_CHARGE - chargeLevel;
                broadcast("RESERVE_CHARGE:" + destX + "," + destY + ":" + getId()
                        + ":" + chargeLevel);
                etat  = Etat.MOVING_TO_CHARGE;
            }
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
            int otherPri = Integer.parseInt(p[2]) - Integer.parseInt(p[3]); // battery - distance

            boolean conflict = zoneId.equals(targetSourceId)
                    && !targetSourceIsTransit && etat == Etat.MOVING_TO_PACKAGE;
            if (conflict) {
                if (shouldYield(otherId, otherPri, mySourcePriority)) {
                    yieldMission(); reservedStartZones.put(zoneId, otherId);
                }
            } else { reservedStartZones.put(zoneId, otherId); }

        // ── RELEASE_START:zoneId ──────────────────────────────────────────
        } else if (c.startsWith("RELEASE_START:")) {
            reservedStartZones.remove(c.substring("RELEASE_START:".length()));

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

        // ── RELEASE_PICKUP:zoneId ─────────────────────────────────────────
        } else if (c.startsWith("RELEASE_PICKUP:")) {
            reservedTransitPickup.remove(c.substring("RELEASE_PICKUP:".length()));

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

        // ── RELEASE_DEPOSIT:zoneId ────────────────────────────────────────
        } else if (c.startsWith("RELEASE_DEPOSIT:")) {
            reservedTransitDeposit.remove(c.substring("RELEASE_DEPOSIT:".length()));

        // ── RESERVE_CHARGE:x,y:otherId:battery ───────────────────────────
        } else if (c.startsWith("RESERVE_CHARGE:")) {
            String[] p = c.substring("RESERVE_CHARGE:".length()).split(":");
            if (p.length < 3) return;
            String key = p[0]; int otherId = Integer.parseInt(p[1]);
            int otherBattery  = Integer.parseInt(p[2]);
            int otherChargePri = MAX_CHARGE - otherBattery; // urgence

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
    public String getEtatName() { return etat.name(); }

    @Override
    public String toString() {
        return getName() + " [" + etat + "] bat=" + chargeLevel
                + " pos=(" + getX() + "," + getY() + ")"
                + (carriedPackage != null ? " porte=1" : "");
    }
}
