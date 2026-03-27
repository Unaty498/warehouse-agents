package pathfind;

public class Node {
    int x, y; // Coordinates of the node
    int g; // Cost from start to current node
    int h; // Heuristic cost to goal
    int f; // Total cost (g + h)
    Node parent; // Parent node for path reconstruction

    public Node(int x, int y) {
        this.x = x;
        this.y = y;
        this.g = Integer.MAX_VALUE; // Initialize g to infinity
        this.h = 0;
        this.f = Integer.MAX_VALUE; // Initialize f to infinity
        this.parent = null;
    }
}
