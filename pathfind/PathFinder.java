package pathfind;

import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Set;

/**
 * Implémentation correcte de A* avec :
 *  - closed set explicite (HashSet<String>) → jamais de nœud retraité
 *  - lazy deletion (on re-ajoute un meilleur nœud dans la PQ,
 *    on ignore les entrées périmées à la dépile grâce au closed set)
 *  - création de nouveaux objets Node → pas de mutation dans la PriorityQueue
 */
public class PathFinder {

    public static int[][] aStar(int[] start, int[] goal,
                                int rows, int columns,
                                boolean[][] obstacles,
                                java.util.List<int[]> extraObstacles) {

        // Validation de base
        if (start[0] < 0 || start[0] >= rows || start[1] < 0 || start[1] >= columns) return null;
        if (goal[0]  < 0 || goal[0]  >= rows || goal[1]  < 0 || goal[1]  >= columns) return null;
        if (obstacles[start[0]][start[1]]) return null; // départ bloqué
        // Si le goal est un obstacle A* ne le trouvera pas (normal : on navigue vers une cellule adj)

        if (start[0] == goal[0] && start[1] == goal[1])
            return new int[][]{{start[0], start[1]}}; // déjà sur place

        // ---- structures A* ----
        // PriorityQueue : tri par f croissant
        PriorityQueue<Node> openSet = new PriorityQueue<>(Comparator.comparingInt(n -> n.f));
        // closed set : cellules définitivement traitées
        Set<String> closedSet = new HashSet<>();
        // meilleur g connu par cellule (pour comparer avant d'ajouter)
        Map<String, Integer> bestG = new HashMap<>();

        Node startNode = new Node(start[0], start[1]);
        startNode.g = 0;
        startNode.h = heuristic(start, goal);
        startNode.f = startNode.h;
        openSet.add(startNode);
        bestG.put(key(start[0], start[1]), 0);

        while (!openSet.isEmpty()) {
            Node current = openSet.poll();
            String ck = key(current.x, current.y);

            // Lazy deletion : si ce nœud a déjà été traité (version périmée), on saute
            if (closedSet.contains(ck)) continue;
            closedSet.add(ck);

            // Objectif atteint
            if (current.x == goal[0] && current.y == goal[1])
                return reconstructPath(current);

            for (int[] dir : new int[][]{{-1,0},{1,0},{0,-1},{0,1}}) {
                int nx = current.x + dir[0];
                int ny = current.y + dir[1];

                if (nx < 0 || nx >= rows || ny < 0 || ny >= columns) continue;
                if (obstacles[nx][ny]) continue;

                boolean isExtra = false;
                if (extraObstacles != null) {
                    for (int[] eo : extraObstacles) {
                        if (eo[0] == nx && eo[1] == ny) {
                            isExtra = true;
                            break;
                        }
                    }
                }
                if (isExtra) continue;

                String nk = key(nx, ny);
                if (closedSet.contains(nk)) continue;

                int tentativeG = current.g + 1;
                Integer knownG = bestG.get(nk);

                if (knownG == null || tentativeG < knownG) {
                    // Meilleur chemin trouvé vers (nx,ny) : créer un NOUVEAU nœud
                    // (ne jamais muter un nœud déjà dans la PQ)
                    Node neighbor = new Node(nx, ny);
                    neighbor.g      = tentativeG;
                    neighbor.h      = heuristic(new int[]{nx, ny}, goal);
                    neighbor.f      = neighbor.g + neighbor.h;
                    neighbor.parent = current;
                    bestG.put(nk, tentativeG);
                    openSet.add(neighbor); // lazy : l'ancienne entrée sera ignorée via closedSet
                }
            }
        }
        return null; // aucun chemin
    }

    // ------------------------------------------------------------------ //
    //  A* vers UNE CIBLE parmi plusieurs (multi-goal)
    // ------------------------------------------------------------------ //

    /**
     * A* qui se termine dès qu'il atteint l'un des buts fournis.
     * Utilisé pour naviguer vers la cellule la plus accessible parmi les 4
     * voisins orthogonaux d'une zone-obstacle, quelle que soit la direction.
     *
     * @param start     position de départ
     * @param goals     tableau de positions cibles (toutes valides)
     * @param rows      nombre de lignes de la grille
     * @param columns   nombre de colonnes de la grille
     * @param obstacles carte des obstacles
     * @return chemin optimal vers le but le plus proche, ou null si inaccessible
     */
    public static int[][] aStarToAny(int[] start, int[][] goals,
                                     int rows, int columns,
                                     boolean[][] obstacles,
                                     java.util.List<int[]> extraObstacles) {
        if (goals == null || goals.length == 0) return null;
        if (start[0] < 0 || start[0] >= rows || start[1] < 0 || start[1] >= columns) return null;
        if (obstacles[start[0]][start[1]]) return null;

        // Ensemble des buts pour lookup O(1)
        Set<String> goalSet = new HashSet<>();
        for (int[] g : goals) {
            if (g[0] >= 0 && g[0] < rows && g[1] >= 0 && g[1] < columns)
                goalSet.add(key(g[0], g[1]));
        }
        if (goalSet.isEmpty()) return null;

        // Si déjà sur un but
        if (goalSet.contains(key(start[0], start[1])))
            return new int[][]{{start[0], start[1]}};

        PriorityQueue<Node> openSet = new PriorityQueue<>(Comparator.comparingInt(n -> n.f));
        Set<String>          closedSet = new HashSet<>();
        Map<String, Integer> bestG    = new HashMap<>();

        Node startNode = new Node(start[0], start[1]);
        startNode.g = 0;
        startNode.h = minHeuristicToAny(start, goals);
        startNode.f = startNode.h;
        openSet.add(startNode);
        bestG.put(key(start[0], start[1]), 0);

        while (!openSet.isEmpty()) {
            Node current = openSet.poll();
            String ck = key(current.x, current.y);
            if (closedSet.contains(ck)) continue;
            closedSet.add(ck);

            // But atteint
            if (goalSet.contains(ck)) return reconstructPath(current);

            for (int[] dir : new int[][]{{-1,0},{1,0},{0,-1},{0,1}}) {
                int nx = current.x + dir[0];
                int ny = current.y + dir[1];
                if (nx < 0 || nx >= rows || ny < 0 || ny >= columns) continue;
                if (obstacles[nx][ny]) continue;

                boolean isExtra = false;
                if (extraObstacles != null) {
                    for (int[] eo : extraObstacles) {
                        if (eo[0] == nx && eo[1] == ny) {
                            isExtra = true;
                            break;
                        }
                    }
                }
                if (isExtra) continue;

                String nk = key(nx, ny);
                if (closedSet.contains(nk)) continue;

                int tentativeG = current.g + 1;
                Integer knownG = bestG.get(nk);
                if (knownG == null || tentativeG < knownG) {
                    Node neighbor = new Node(nx, ny);
                    neighbor.g      = tentativeG;
                    neighbor.h      = minHeuristicToAny(new int[]{nx, ny}, goals);
                    neighbor.f      = neighbor.g + neighbor.h;
                    neighbor.parent = current;
                    bestG.put(nk, tentativeG);
                    openSet.add(neighbor);
                }
            }
        }
        return null;
    }

    /** Heuristique minimale vers l'un des buts (admissible). */
    private static int minHeuristicToAny(int[] pos, int[][] goals) {
        int min = Integer.MAX_VALUE;
        for (int[] g : goals) {
            int h = heuristic(pos, g);
            if (h < min) min = h;
        }
        return min;
    }


    private static String key(int x, int y) {
        return x + "," + y;
    }

    private static int[][] reconstructPath(Node goal) {
        // goal.g = longueur du chemin (nombre de pas)
        int[][] path = new int[goal.g + 1][2];
        int idx = goal.g;
        Node cur = goal;
        while (cur != null) {
            path[idx--] = new int[]{cur.x, cur.y};
            cur = cur.parent;
        }
        return path;
    }

    private static int heuristic(int[] a, int[] b) {
        return Math.abs(a[0] - b[0]) + Math.abs(a[1] - b[1]); // Manhattan
    }
}
