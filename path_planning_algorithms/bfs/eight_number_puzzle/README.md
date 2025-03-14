

## 8 bit puzzle solver using Breadth First Search
1) Set the initial node in the solve_puzzle.py
    Modify this array to desired initial mode row wise
    Example initial state :
    8 6 7
    2 5 4
    3 0 1
    initial_node = np.array([[8,6,7],[2,5,4],[3,0,1]])
2) To run the program, after downloading file, run command: python3 
solve_puzzle.py
3) After running, text files Nodes.txt, NodesInfo.txt and nodePath.txt will be 
generated.
   All elemnts of nodes are being stored column wise in files i.e. for this state 1
4 7 2 5 8 3 6 0, the eight puzzle state is
    1 2 3
    4 5 6
    7 8 0
   Nodes.txt:All the explored states present as a list
   NodesInfo.txt: Information about all nodes explored.
    First column: Node Index
    Second Column: Parent Node Index
    Third column: Node
   nodePath.txt: nodePath.txt
    The solution to the puzzle as solved by your code. 
The order of the states is from start node to the goal node.
5) To animate solution , run command :python3 Animate.py
4) Libraries used: numpy,collection,json