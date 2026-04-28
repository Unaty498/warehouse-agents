package pathfind;

public class Node {
    int x, y; // Coordonnées du nœud
    int g;    // Coût du début jusqu'à ce nœud
    int h;    // Heuristique jusqu'à l'objectif
    int f;    // Coût total (g + h)
    Node parent; // Nœud parent pour la reconstruction du chemin

    public Node(int x, int y) {
        this.x = x;
        this.y = y;
        this.g = Integer.MAX_VALUE; // Initialisé à l'infini
        this.h = 0;
        this.f = Integer.MAX_VALUE; // Initialisé à l'infini
        this.parent = null;
    }
}
