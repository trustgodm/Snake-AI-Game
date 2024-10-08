 import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;

import java.util.List;
import java.util.PriorityQueue;
 
 

// Import necessary classes for A* algorithm
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
    int count =0;
    private int[][] grid = new int[50][50];
    private List<int[]> otherSnakesHeads = new ArrayList<>();
    private List<int[]> zombiesHeads = new ArrayList<>();
    private List<int[]>  snakeBody = new ArrayList<>();

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
                            drawLine(obsCoords[i], obsCoords[i + 1], 7); // 4 represents obstacles
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
                        int[] temps = {headX,headY};
                        zombiesHeads.add(temps);
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
                            	String start = snakeCoords[k];
                                String end = snakeCoords[k + 1];

                                drawLine(start, end, 9); // 9 represents my snake

                                // Extract the coordinates of the current segment and add to snakeBody
                                int segmentX = Integer.parseInt(end.split(",")[0]);
                                int segmentY = Integer.parseInt(end.split(",")[1]);
                                snakeBody.add(new int[]{segmentX, segmentY}); // 9 represents my snake
                              
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

                     
                    decideMoveSafety() ;
                    int move = decideMove(appleX, appleY);
                    System.out.println(move);
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
    	 ZombieBlock();
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
           
            for (int i = 0; i < 4; i++) {
                int newX = currentNode.x + dx[i];
                int newY = currentNode.y + dy[i];
                
                
                if (isValidMove(newX, newY) && !closedSet[newX][newY] && !gettingInbetween(newX, newY) && !dontEatApple() && !otherSnakeNearMe(newX, newY)) {
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
       // clearZombiesNextMove();
        return null; // No path found
    }
   
    private void decideMoveSafety() {
    	  Node pathToApple = aStar(currentX, currentY, appleX, appleY);
          if (pathToApple != null) {
              Node current = pathToApple;
              while (current.parent != null && current.parent.parent != null) {
                  current = current.parent;
              }
              if(!otherSnakeNearMe(current.x, current.y)) {
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
          }  
    }
    // Heuristic function for A* (Manhattan distance)
    private int heuristic(int x1, int y1, int x2, int y2) {
        return Math.abs(x1 - x2) + Math.abs(y1 - y2);
    }

    // Decide the move based on the direction of the snake and position of the apple
    private int decideMove(int appleX, int appleY) {
    	Node pathToApple = aStar(currentX, currentY, appleX, appleY);
        // Check if we are the closest to the apple
        if (!isClosestToApple() ) {
        	if(moveAlongBodySafety() !=-1) {
        		return  moveAlongBodySafety();
        	}
        	else if(escape() != -1 && pathToApple != null ) {
        		 
        		return escape();
        		
        	}
        	else {
        		 
        		return  moveAlongBody();
        	}
 
        }else {

        // If closest to apple, use A* to find the shortest path to the apple
        
        if (pathToApple != null) {
            Node current = pathToApple;
            while (current.parent != null && current.parent.parent != null) {
                current = current.parent;
            }
          if (!otherSnakeNearMe(current.x, current.y) && !dontEatAppleTwo()) {
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
         
            // If no path is found, try moving along the snake's body as a fallback
            return moveAlongBody();
        }

        return directionToApple;
    }
    }
    public boolean avoidZombieHeadToHead() {
        int[][] directions = {{0, -1, 0}, {0, 1, 1}, {-1, 0, 2}, {1, 0, 3}}; // Up, Down, Left, Right

        for (int[] zombieHead : zombiesHeads) {
            int zombieX = zombieHead[0];
            int zombieY = zombieHead[1];

            // Calculate Manhattan distance between the snake's head and the zombie's head
            int distance = Math.abs(currentX - zombieX) + Math.abs(currentY - zombieY);

            // Check if the snake is within 5 units distance
            if (distance <= 5) {
                for (int[] direction : directions) {
                    int newX = currentX + direction[0];
                    int newY = currentY + direction[1];

                    // Check if the new position moves toward the zombie's head
                    if (newX == zombieX && newY == zombieY) {
                        return true; // Avoid this move
                    }
                }
            }
        }

        return false; // No zombies are too close or directly ahead
    }
   

    private boolean isClosestToApple() {
        int myDistance =  heuristic(currentX, currentY, appleX, appleY);
        

        // Compare distance from each other snake's head to the apple
        for (int[] head : otherSnakesHeads) {
            int otherDistance =  heuristic(head[0], head[1], appleX, appleY);

            // If another snake is closer, increment the counter
            if (otherDistance < myDistance) {
                return false;
            }
        }

        // Return true if there are fewer than 2 snakes closer to the apple
        return true;
    }

 public boolean gettingInbetween(int x, int y) { // getting between myself and other snakes
        if (x - 1 >= 0 && y - 1 >= 0 && x + 1 < 50 && y + 1 < 50) {
            if (grid[x + 1][y] == 7 && grid[x-1][y] == 7) { // facing up or down
                
                return true;
            }
            if (grid[x][y-1] == 7 && grid[x][y+1] == 7) {
            	return true;
            }
           
           
        }
    
        return false;
    }
    public boolean gettingInbetweenSafety(int x, int y) { // getting between myself and other snakes
        if (x - 1 >= 0 && y - 1 >= 0 && x + 1 < 50 && y + 1 < 50) {
            if (grid[x + 1][y] !=0 && grid[x-1][y] !=0) { // facing up or down
                
                return true;
            }
            if (grid[x][y-1] !=0 && grid[x][y+1] !=0) {
            	return true;
            }
           
           
        }
    
        return false;
    }
   private int moveAlongBody() {
        int maxSpace = -1; // To track the maximum space found
        int bestMove = -1; // To track the best move direction

        // Define the possible directions with corresponding movement codes
        int[][] directions = {{0, -1, 0}, {0, 1, 1}, {-1, 0, 2}, {1, 0, 3}};

        // Iterate through all directions to find the one with the most space
        for (int[] direction : directions) {
            int newX = currentX + direction[0];
            int newY = currentY + direction[1];

            // Check if the initial move is valid and measure available space in that direction
            if (isValidMove(newX, newY) && !gettingInbetween(newX, newY) && !otherSnakeNearMe(newX, newY)) {
                int space = measureAvailableSpace(newX, newY, direction);

                // Update the best move if this direction has more space
                if (space > maxSpace) {
                    maxSpace = space;
                    bestMove = direction[2];
                }
            }
        }

        // If a valid move with space is found, return that direction
        if (bestMove != -1) {
            return bestMove;
        } 
        
        return bestMove;
         }
  
       private int moveAlongBodySafety() {
	   int maxSpace = -1; // To track the maximum space found
       int bestMove = -1; // To track the best move direction

       // Define the possible directions with corresponding movement codes
       int[][] directions = {{0, -1, 0}, {0, 1, 1}, {-1, 0, 2}, {1, 0, 3}};

       // Iterate through all directions to find the one with the most space
       for (int[] direction : directions) {
           int newX = currentX + direction[0];
           int newY = currentY + direction[1];
           if(newX - 1 >= 0 && newY - 1 >= 0 && newX + 1 < 50 &&  newY + 1 < 50) {
           Node pathToApple = aStar(newX, newY, appleX, appleY);
           // Check if the initial move is valid and measure available space in that direction
           if (isValidMove(newX, newY) && !gettingInbetweenSafety(newX, newY) && !otherSnakeNearMe(newX, newY) && pathToApple != null) {
               int space = measureAvailableSpace(newX, newY, direction);

               // Update the best move if this direction has more space
               if (space > maxSpace) {
                   maxSpace = space;
                   bestMove = direction[2];
               }
           }
           }
       }

       // If a valid move with space is found, return that direction
       if (bestMove != -1) {
           return bestMove;
       } 
        
       return bestMove;
        	} 
 
        	// Helper function to calculate the minimum distance to any other snake's head
        	 

    public void ZombieBlock() {
        	    // Define possible movement directions: up, down, left, right
        	    int[][] directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}}; // Up, Down, Left, Right

        	    for (int[] head : zombiesHeads) {
        	        int x = head[0];
        	        int y = head[1];

        	        // Check each direction for potential next move
        	        for (int[] direction : directions) {
        	            int nextX = x + direction[0];
        	            int nextY = y + direction[1];

        	            // Ensure the next position is within bounds
        	            if (isValidMove(nextX, nextY)) {
        	                grid[nextX][nextY] = 8; // Mark as a block
        	            }
        	        }
        	    }
        	 
        	}
 
    private int escape() {
        // Define the four corners of the grid
        int[][] corners = {{0, 0}, {0, 49}, {49, 0}, {49, 49}};
        
        int bestCornerX = -1;
        int bestCornerY = -1;
        int minTraffic = Integer.MAX_VALUE;
        Node pathToApple = aStar(currentX, currentY, appleX, appleY);
        if (pathToApple != null) {
        // Iterate through all corners
        for (int[] corner : corners) {
            int cornerX = corner[0];
            int cornerY = corner[1];
            
            // Measure the traffic at each corner
            int traffic = measureTraffic(cornerX, cornerY);
            
            // Find the corner with the least traffic
            if (traffic < minTraffic) {
                minTraffic = traffic;
                bestCornerX = cornerX;
                bestCornerY = cornerY;
            }
        }
        
        
        // Use A* or a similar pathfinding algorithm to move towards the selected corner
        Node pathToCorner = aStar(currentX, currentY, bestCornerX, bestCornerY);
        if (pathToCorner != null) {
            Node current = pathToCorner;
            while (current.parent != null && current.parent.parent != null) {
                current = current.parent;
            }
            if (!otherSnakeNearMe(current.x, current.y)) {
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
        }
        }
        return -1;
        // If no valid path, return a default value
    }

    // Helper function to measure the traffic around a given position
    private int measureTraffic(int x, int y) {
        int traffic = 0;
        
        // Check nearby cells within a certain radius (e.g., 5 cells around the corner)
        int radius = 5;
        for (int i = Math.max(0, x - radius); i < Math.min(50, x + radius); i++) {
            for (int j = Math.max(0, y - radius); j < Math.min(50, y + radius); j++) {
                // Increase traffic count if the cell is occupied by a zombie, snake, or obstacle
                if (grid[i][j] != 0) {
                    traffic++;
                }
            }
        }
        
        return traffic;
    }
   
   


     public boolean dontEatApple() {
   if (appleX - 1 >= 0 && appleY - 1 >= 0 && appleX + 1 < 50 &&  appleY + 1 < 50) {
              if (grid[appleX][appleY - 1] != 0 && grid[appleX][appleY + 1] != 0 && grid[appleX - 1][appleY] != 0 ) return true;
            if(grid[appleX][ appleY + 1] != 0 && grid[appleX - 1][ appleY] != 0 && grid[appleX + 1][appleY] != 0)  return true;
            if(grid[appleX][appleY - 1] != 0 && grid[appleX - 1][ appleY] != 0 && grid[appleX + 1][appleY] != 0 )  return true;
             if(grid[appleX][appleY - 1] != 0 && grid[appleX][ appleY + 1] != 0 && grid[appleX + 1][ appleY] != 0) return true;
          }

          return false;
     }
     public boolean dontEatAppleTwo() {
    	    
    	 // Run A* for each snake's head to see if any other snake can reach the apple
    	 int count =0;
        for (int[] otherSnakeHead : otherSnakesHeads) {
            Node path = aStar(otherSnakeHead[0], otherSnakeHead[1], appleX, appleY);
            if (path == null) {
            	count++;
            	if (count == 3) {
                return false; 
            	}// Another snake can reach the apple, so it's safe to go for it
            }
        }

        // If no other snake can reach the apple but your snake can, avoid it
        Node myPath = aStar(currentX, currentY, appleX, appleY);
        if (myPath != null) {
            return true; // Only your snake can reach the apple, avoid eating it
        }
    	    
        for(int[] head : zombiesHeads) {
        	int otherDistance = heuristic(head[0], head[1], appleX, appleY);
            if (otherDistance < 2 ) {
                 return false; // There's another snake closer to the apple
            }
        }
    	    return false; // The apple is not trapped, safe to eat
    	}

    // Measure the amount of free space in a given direction
    private int measureAvailableSpace(int startX, int startY, int[] direction) {
        int spaceCount = 0;
        int stepsToCheck = 50; // Measure up to the edge of the grid

        for (int i = 1; i <= stepsToCheck; i++) {
            int checkX = startX + direction[0] * i;
            int checkY = startY + direction[1] * i;

            // If any step is invalid, stop counting space
            if (!isValidMove(checkX, checkY) || gettingInbetween(checkX, checkY) || otherSnakeNearMe(checkX, checkY)) {
                break;
            }

            spaceCount++;
        }

        return spaceCount;
    }
  public boolean otherSnakeNearMe(int x, int y) {
        boolean close = false;

        if (x - 1 >= 0 && y - 1 >= 0 && x + 1 < 50 && y + 1 < 50) {
            if (grid[x][y - 1] == 8 || grid[x][y + 1] == 8 || grid[x - 1][y] == 8 || grid[x + 1][y] == 8) {
                close = true;
            }
        }
        for(int[] head : zombiesHeads) {
        	int headX = head[0];
        	int headY = head[1];
        	int distance = heuristic(headX,headY,currentX,currentY);
        	if(distance < 2) {
        		return true;
        	}
        }

        return close;
    }
 
  private boolean isValidMove(int x, int y) {
        return x >= 0 && x < 50 && y >= 0 && y < 50 && (grid[x][y] == 0 || grid[x][y] == 6);
    }

    // Main method to start the agent
    public static void main(String[] args) {
        MyAgent agent = new MyAgent();
        MyAgent.start(agent, args);
    }
}


