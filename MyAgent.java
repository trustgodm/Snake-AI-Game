import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
 
import java.util.List;
import java.util.PriorityQueue;
 
import java.util.Random;

 
import java.util.Comparator;
import java.util.HashMap;
import java.util.Map;

import za.ac.wits.snake.DevelopmentAgent;

public class MyAgent extends DevelopmentAgent {

    private int directionToApple = 5; // Default to "Straight"
    private int currentX = 0;
    private int currentY = 0;
    int tailX = 0;
    int tailY = 0;
    private int appleX = 0;
    private int appleY = 0;
    private int mySnakeNum = 0;
    private int[][] grid = new int[50][50];
    private List<int[]> otherSnakesHeads = new ArrayList<>();
    private List<int[]> zombiesHeads = new ArrayList<>();

    // Class to represent a point in the grid
    class Point {
        int x, y;
        Point parent;

        Point(int x, int y, Point parent) {
            this.x = x;
            this.y = y;
            this.parent = parent;
        }
    }

    // Node class for A* algorithm
    class Node {
        int x, y;
        int gCost; // Cost from start node
        int hCost; // Heuristic cost to goal
        Node parent;

        Node(int x, int y, int gCost, int hCost, Node parent) {
            this.x = x;
            this.y = y;
            this.gCost = gCost;
            this.hCost = hCost;
            this.parent = parent;
        }

        int getFCost() {
            return gCost + hCost;
        }
    }

    // Initialize the grid with zeros
    public void fillGrid() {
        for (int i = 0; i < 50; i++) {
            for (int j = 0; j < 50; j++) {
                grid[i][j] = 0;
            }
        }
    }

    // Draw a line on the grid using Bresenham's Line Algorithm
    public void drawLine(String a, String b, int n) {
        String[] startCoords = a.split(",");
        String[] endCoords = b.split(",");
        int x1 = Integer.parseInt(startCoords[0]);
        int y1 = Integer.parseInt(startCoords[1]);
        int x2 = Integer.parseInt(endCoords[0]);
        int y2 = Integer.parseInt(endCoords[1]);

        int dx = Math.abs(x2 - x1);
        int dy = Math.abs(y2 - y1);
        int sx = x1 < x2 ? 1 : -1;
        int sy = y1 < y2 ? 1 : -1;
        int err = dx - dy;

        while (true) {
            if (x1 >= 0 && x1 < 50 && y1 >= 0 && y1 < 50) {
                if (grid[x1][y1] == 0) {
                    grid[x1][y1] = n; // Represent the line with the value n
                }
            }

            if (x1 == x2 && y1 == y2) {
                break;
            }
            int e2 = err * 2;
            if (e2 > -dy) {
                err -= dy;
                x1 += sx;
            }
            if (e2 < dx) {
                err += dx;
                y1 += sy;
            }
        }
    }

    @Override
    public void run() {
        try (BufferedReader br = new BufferedReader(new InputStreamReader(System.in))) {
            String initString = br.readLine();
            String[] temp = initString.split(" ");
            int nSnakes = Integer.parseInt(temp[0]);

            while (true) {
                fillGrid();
                String line = br.readLine();
                if (line.contains("Game Over")) {
                    System.out.println("log Game over");
                    break;
                }

                // Process apple position
                String[] appleCoords = line.split(" ");
                if (appleCoords.length >= 2) {
                    appleX = Integer.parseInt(appleCoords[0]);
                    appleY = Integer.parseInt(appleCoords[1]);
                    grid[appleX][appleY] = 6; // 6 represents apple

                    // Read obstacles
                    int nObstacles = 3; // Number of obstacles
                    for (int obstacle = 0; obstacle < nObstacles; obstacle++) {
                        String obs = br.readLine();
                        String[] obsCoords = obs.split(" ");
                        for (int i = 0; i < obsCoords.length - 1; i++) {
                            drawLine(obsCoords[i], obsCoords[i + 1], 4); // 4 represents obstacles
                        }
                    }

                    // Read zombies
                    int nZombies = 3; // Number of zombies
                    zombiesHeads.clear();
                    for (int zombie = 0; zombie < nZombies; zombie++) {
                        String zom = br.readLine();
                        String[] zomCoords = zom.split(" ");
                        int headX = Integer.parseInt(zomCoords[0].split(",")[0]);
                        int headY = Integer.parseInt(zomCoords[0].split(",")[1]);

                        grid[headX][headY] = 8;
                        for (int i = 0; i < zomCoords.length - 1; i++) {
                            drawLine(zomCoords[i], zomCoords[i + 1], 7); // 7 represents zombies
                        }
                    }

                    // Get the snake's index and details
                    mySnakeNum = Integer.parseInt(br.readLine());
                    otherSnakesHeads.clear();
                    for (int i = 0; i < nSnakes; i++) {
                        String snakeLine = br.readLine();
                        if (i == mySnakeNum) {
                            // Process details for your own snake
                            String[] snakeCoords = snakeLine.split(" ");

                            currentX = Integer.parseInt(snakeCoords[3].split(",")[0]);
                            currentY = Integer.parseInt(snakeCoords[3].split(",")[1]);
                             
                            for (int k = 3; k < snakeCoords.length - 1; k++) {
                                drawLine(snakeCoords[k], snakeCoords[k + 1], 9); // 9 represents my snake
                                if(k == snakeCoords.length - 2) {
                                    tailX = Integer.parseInt(snakeCoords[3].split(",")[0]);
                                    tailY = Integer.parseInt(snakeCoords[3].split(",")[1]);
                                }
                            }
                        } else {
                            String[] snakeCoords = snakeLine.split(" ");
                            if (!snakeCoords[0].equals("dead")) {

                                int headX = Integer.parseInt(snakeCoords[3].split(",")[0]);
                                int headY = Integer.parseInt(snakeCoords[3].split(",")[1]);
                                grid[headX][headY] = 8;
                                otherSnakesHeads.add(new int[]{headX, headY});
                                for (int k = 3; k < snakeCoords.length - 1; k++) {
                                    drawLine(snakeCoords[k], snakeCoords[k + 1], 7); // 7 represents other snakes
                                }
                            }
                        }
                    }

                    // Decide the move
                    int move = decideMove(appleX, appleY);
                    System.out.println(move);
                } else {
                    // Handle case where apple position line is malformed
                    System.out.println(directionToApple);
                }
            }
        } catch (IOException e) {
            e.printStackTrace();
        } catch (NumberFormatException e) {
            System.out.println("log Error parsing number: " + e.getMessage());
        }
    }

    // Perform A* to find the shortest path to the apple
    private Node aStar(int startX, int startY, int goalX, int goalY) {
        PriorityQueue<Node> openSet = new PriorityQueue<>(Comparator.comparingInt(Node::getFCost));
        Map<String, Node> allNodes = new HashMap<>();
        boolean[][] closedSet = new boolean[50][50];

        Node startNode = new Node(startX, startY, 0, heuristic(startX, startY, goalX, goalY), null);
        openSet.add(startNode);
        allNodes.put(startX + "," + startY, startNode);

        int[] dx = {-1, 1, 0, 0}; // Left, Right, Up, Down
        int[] dy = {0, 0, -1, 1};

        while (!openSet.isEmpty()) {
            Node currentNode = openSet.poll();

            // If we reach the goal, return the current node
            if (currentNode.x == goalX && currentNode.y == goalY) {
                return currentNode;
            }

            closedSet[currentNode.x][currentNode.y] = true;

            // Explore neighbors
            for (int i = 0; i < 4; i++) {
                int newX = currentNode.x + dx[i];
                int newY = currentNode.y + dy[i];

                if (isValidMove(newX, newY) && !closedSet[newX][newY]) {
                    int newGCost = currentNode.gCost + 1;
                    int newHCost = heuristic(newX, newY, goalX, goalY);

                    Node neighbor = allNodes.get(newX + "," + newY);
                    if (neighbor == null || newGCost < neighbor.gCost) {
                        neighbor = new Node(newX, newY, newGCost, newHCost, currentNode);
                        allNodes.put(newX + "," + newY, neighbor);
                        openSet.add(neighbor);
                    }
                }
            }
        }

        return null; // No path found
    }

    // Heuristic function for A* (Manhattan distance)
    private int heuristic(int x1, int y1, int x2, int y2) {
        return Math.abs(x1 - x2) + Math.abs(y1 - y2);
    }

    // Decide the move based on the direction of the snake and position of the apple
    private int decideMove(int appleX, int appleY) {
        // Use A* to find the shortest path to the apple
        Node pathToApple = aStar(currentX, currentY, appleX, appleY);
          
        // If a path is found, backtrack to determine the first move
        if (pathToApple != null) {
            Node current = pathToApple;
            while (current.parent != null && current.parent.parent != null) {
                current = current.parent;
            }

            if (otherSnakeNearMe(current.x, current.y)) {
                grid[current.x][current.y] = 4;
                return decideMove(appleX, appleY); // Recursively decide the move
            }  else if (!otherSnakeNearMe(current.x, current.y)) {
                if (current.x < currentX) {
                    directionToApple = 2; // Left
                } else if (current.x > currentX) {
                    directionToApple = 3; // Right
                } else if (current.y < currentY) {
                    directionToApple = 0; // Up
                } else if (current.y > currentY) {
                    directionToApple = 1; // Down
                }
            }
        } else {
            directionToApple = moveAlongBody();
        }

        return directionToApple;
    }

    public boolean gettingInbetween(int x, int y) {
        if (x - 1 >= 0 && y - 1 >= 0 && x + 1 < 50 && y + 1 < 50) {
            if (grid[x + 1][y] == 7 && grid[x][y - 1] == 7) {
                System.out.println("log avoided going between the snakes");
                return true;
            }
        }
        return false;
    }

    private int moveAlongBody() {
        for (int[] direction : new int[][]{{0, -1}, {0, 1}, {-1, 0}, {1, 0}}) {
            int newX = currentX + direction[0];
            int newY = currentY + direction[1];

            if (isValidMove(newX, newY)) {
                if (direction[0] == 0 && newY < currentY) return 0; // Move Up
                if (direction[0] == 0 && newY > currentY) return 1; // Move Down
                if (direction[1] == 0 && newX < currentX) return 2; // Move Left
                if (direction[1] == 0 && newX > currentX) return 3; // Move Right
            }
        }

        // If no valid move along the body is found, move randomly (as a last resort)
        Random rand = new Random();
        return rand.nextInt(4);
    }
    
    // Check if another snake is near
    public boolean otherSnakeNearMe(int x, int y) {
        boolean close = false;

        if (x - 1 >= 0 && y - 1 >= 0 && x + 1 < 50 && y + 1 < 50) {
            if (grid[x][y - 1] == 8 || grid[x][y + 1] == 8 || grid[x - 1][y] == 8 || grid[x + 1][y] == 8) {
                close = true;
            }
        }

        return close;
    }
 
    public boolean farFromApple() {
        for (int[] head : otherSnakesHeads) {
            int headX = head[0];
            int headY = head[1];
              
            if ((Math.abs(headX - appleX) > Math.abs(currentX - appleX)) && (Math.abs(headY - appleY) > Math.abs(currentY - appleY))) {
                return true;
            }
        }
        return false;
    }

    // Check if a move is valid
    private boolean isValidMove(int x, int y) {
        return x >= 0 && x < 50 && y >= 0 && y < 50 && (grid[x][y] == 0 || grid[x][y] == 6);
    }

    // Main method to start the agent
    public static void main(String[] args) {
        MyAgent agent = new MyAgent();
        MyAgent.start(agent, args);
    }
}
