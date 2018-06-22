/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
class DistanceComparator implements Comparator<NodeWithDistance>
{
    @Override
    public int compare(NodeWithDistance x, NodeWithDistance y)
    {
 
        if (x.getDistance() < y.getDistance())
        {
            return -1;
        }
        if (x.getDistance() > y.getDistance())
        {
            return 1;
        }
        return 0;
    }

}

class NodeWithDistance{
	private GeographicPoint coordinates;
	private Double distance;
	
	public NodeWithDistance(GeographicPoint coordinates, Double distance)
	{
		this.coordinates=coordinates;
		this.setDistance(distance);
	}

	public GeographicPoint getCoordinates() {
		return coordinates;
	}

	public Double getDistance() {
		return distance;
	}
	
	public String toString()
	{
		return coordinates+":"+distance;
	}

	public void setDistance(Double distance) {
		this.distance = distance;
	}

}
class Street {
	private GeographicPoint from;
	private GeographicPoint to;
	private String roadName;
	private String roadType; 
	private double length;
	
	public Street(GeographicPoint from, GeographicPoint to, 
				String roadName, String roadType, double length)
	{
		this.from = from;
		this.to= to;
		this.roadName = roadName;
		this.roadType = roadType;
		this.length = length;
	}
	
	public GeographicPoint getFrom() {
		return this.from;
	}
	public GeographicPoint getTo() {
		return this.to;
	}
	public String getRoadName() {
		return this.roadName;
	}
	public String getRoatType() {
		return this.roadType;
	}
	public double getLength() {
		return this.length;
	}
	
	public String toString(){
		return this.from+", "+this.to+": "+length+"\n";
	}
}
public class MapGraph {
	//TODO: Add your member variables here in WEEK 3
	Set<GeographicPoint> vertices;
	List<Street> streets;
	private int numVertices;
	private int numEdges;
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		vertices = new HashSet<GeographicPoint>();
		streets = new LinkedList<Street>();
		numVertices=0;
		numEdges=0;
		// TODO: Implement in this constructor in WEEK 3
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 3
		return numVertices;
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 3
		return vertices;
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 3
		return numEdges;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 3
		if(vertices.add(location))
			return true;
		else
			return false;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		//TODO: Implement this method in WEEK 3
		double dist = from.distance(to);
		Street street = new Street(from,to,roadName,roadType,dist);
		streets.add(street);
	}
	
	public List<GeographicPoint> constructPath(GeographicPoint start, GeographicPoint goal, 
							HashMap<GeographicPoint, GeographicPoint> parentMap )
	{
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		
		GeographicPoint curr = goal;
		while(curr!=start)
		{
			path.addFirst(curr);
			curr = parentMap.get(curr);
		}
		path.addFirst(curr);
		
		return path;
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		if(vertices.contains(start)==false || vertices.contains(goal)==false)
		{
			System.out.println("Start or Goal is Null. No Path exists!");
			return null;
		}
		
		HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
		HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>();
		Queue<GeographicPoint> queue = new LinkedList<GeographicPoint>();
		queue.add(start);
		boolean found = false;
		while(!queue.isEmpty())
		{
			GeographicPoint curr = queue.remove();
			
			if(curr.equals(goal))
			{
				found = true;
				break;
			}
			
			for(Street next: streets)
			{
				if(next.getFrom().equals(curr) && !visited.contains(next.getTo()))
				{
					nodeSearched.accept(next.getTo());
					queue.add(next.getTo());
					visited.add(curr);
					parentMap.put(next.getTo(), curr);
				}
			}
		}
		
		if(!found)
		{
			System.out.println("No Path exists!");
			return new LinkedList<GeographicPoint>();
		}
				
		return constructPath(start, goal, parentMap);
	}
	

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		//System.out.println(streets);
		if(vertices.contains(start)==false || vertices.contains(goal)==false)
		{
			System.out.println("Start or Goal is Null. No Path exists!");
			return null;
		}
		
		HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
		HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>();
		HashMap<GeographicPoint, Double> distances = new HashMap<GeographicPoint, Double>();
		
		
		for(GeographicPoint each: vertices)
			distances.put(each, Double.POSITIVE_INFINITY);
		
		Comparator<NodeWithDistance> comparator = new DistanceComparator();
		PriorityQueue<NodeWithDistance> queue = new PriorityQueue<NodeWithDistance>(11, comparator);
		
		distances.put(start, 0.0);
		queue.add(new NodeWithDistance(start, distances.get(start)));
		
		boolean found = false;
		while(!queue.isEmpty())
		{
			/*
			System.out.println("-----");
			 
			System.out.println(parentMap);
			for(NodeWithDistance each: queue)
				System.out.print(each+" ,");
			System.out.println("\n"+visited);
			*/
			NodeWithDistance curr = queue.remove();
			double curr_dist = curr.getDistance();
			if(!visited.contains(curr.getCoordinates()))
			{
				visited.add(curr.getCoordinates());
				if(curr.getCoordinates().equals(goal))
				{
					found = true;
					break;
				}
				
				for(Street next: streets)
				{					
					if(next.getFrom().equals(curr.getCoordinates()) && !visited.contains(next.getTo()))
					{
						if((next.getLength() + curr_dist) < distances.get(next.getTo()))
						{
							nodeSearched.accept(next.getTo());
							distances.put(next.getTo(), (next.getLength() + curr_dist));
							parentMap.put(next.getTo(), curr.getCoordinates());
							distances.put(next.getTo(), (next.getLength() + curr_dist));
							queue.add(new NodeWithDistance(next.getTo(), distances.get(next.getTo())));
						}
					}
				}
			}	
			
		}
		
		if(!found)
		{
			System.out.println("No Path exists!");
			return new LinkedList<GeographicPoint>();
		}
		System.out.println(parentMap);
				
		return constructPath(start, goal, parentMap);
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}

	
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		
		// You can use this method for testing.  
		GeographicPoint start = new GeographicPoint(1.0, 1.0);
		GeographicPoint end = new GeographicPoint(8.0, -1.0);
		
		
		List<GeographicPoint> route = firstMap.bfs(start,end);
		
		System.out.println(route);
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		System.out.println(testroute);
		
		/*
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		System.out.println(testroute);
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		System.out.println(testroute);
		
		
		/* Use this code in Week 3 End of Week Quiz */
		/*MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
	}
	
}
