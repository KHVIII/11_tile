
#states are represented by a list of 12 2-tuples, the index represents the numerical value of the tile and the 2-tuple stored at the index represents the (row, column) position of the tile.
#the tuples are in the form (i,j) where 1<=i<=3, 1<=j<=4
#eg:

#[(1,1),(1,2),(1,3),(1,4),(2,1),(2,2),(2,3),(2,4),(3,1),(3,2),(3,3),(3,4)]
# is the same as
#0 1 2 3
#4 5 6 7
#8 9 10 11



#All the possible actions for 0 tile to move, the costs are all 1
ACTIONS = {(-1,0):'U',(1,0):'D',(0,-1):"L",(0,1):"R"}
class Node:
    #frontier
    openList = []
    #list to keep track of repeated states
    closedList = []
    
    def __init__(self, state=None, parent=None,g = []):
        self.state = state #list of 12 2-tuples
        self.parent = parent #parent node
        self.g = g #actions done from root to current node, the length of this list is the path cost
        self.h = 0 #manhattan distance to goal node
        self.f = 0 
    
    #checking to see if two nodes have same states
    #(Node,Node) -> Boolean 
    def __eq__(self, other): 
        return (self.state == other.state)

    #checking to see the sum of the manhattan distance between two node states
    #(Node,Node) -> Integer
    def __sub__(self, other):
        sumOMDist = 0 #sum of manhattan distance of the tile differences excluding the 0 tile
        for tileNumber in range(1,12):
            selfPos = self.state[tileNumber] #position of the tile number in self state
            otherPos = other.state[tileNumber] #position of the tile number in other state

            #manhattan distance is the absolute value of (selfI-otherI) plus the absolute value of (selfJ - otherJ)
            manhattan_dist = abs(selfPos[0]-otherPos[0]) + abs(selfPos[1]-otherPos[1])
            sumOMDist += manhattan_dist
        return sumOMDist

    #representation of self, this is for debugging
    #Node -> String
    def __repr__(self):
        return("State: "+str(self.state)+", g: "+str(self.g)+", h: "+str(self.h) + ", f: "+str(self.f))

    #string representation of self, this is formatted according to the requirements
    #Node -> String
    def __str__(self):
        outString = str(len(self.g)) + '\n' #number of actions performed to achieve current node from root node.
        outString += str (len(self.openList) + len(self.closedList)) + '\n' #number of nodes generated is equal to the size of the open list and closed list combined
        outString += (' '.join(self.g)) + '\n' #actions performed to lead to this node, should be equal to f at goal node
        
        fString = str(self.f)
        currNode = self
        for i in range(self.f):
            currNode = currNode.parent
            fString = str(currNode.f) + ' ' + fString

        outString += fString

        
            
        return(outString)

#given a list of nodes, give the index of the node that has the lowest f value.
#list<Node> -> Integer
def getLowestFNodeIndex(nodeList):
    lowestF = 100000
    lowestIndex = -1
    for i in range(len(nodeList)):
        if nodeList[i].f < lowestF:
            lowestF = nodeList[i].f 
            lowestIndex = i
    return lowestIndex

#given a state and 2 lists of nodes, check to see if any node in any of the lists contain that state.
#(list<tuple>, list<Node>, list<Node>) -> Boolean
def checkUnique(state,list1,list2):
    for node in list1:
        if (state == node.state):
            return False
    for node in list2:
        if (state == node.state):
            return False
    
    return True


#given the initial state and the goal state, use A* search to find and return the goal node
#(list<tuple>,list<tuple>) -> Node
def getSolutionNode(rootState,goalState):
    rootNode = Node(rootState)
    goalNode = Node(goalState)
    rootNode.h = rootNode - goalNode
    rootNode.f = rootNode.h
    Node.openList.append(rootNode)

    while (len(Node.openList)>0):
        #pop lowest f node from frontier
        currentNode = Node.openList.pop(getLowestFNodeIndex(Node.openList))

        #debug
        ##print(currentNode.state[0],currentNode.f)
        
        #check if it is a goal node
        if (currentNode.state == goalState):
            return currentNode

        #expansion of the currentNode 
        zeroPosition = currentNode.state[0] #get position of the 0 tile in (i,j) format
        for change in ACTIONS: #0 tile tries to swap with Up, Down, Left, Right (in that order)
            newZeroPosition = (zeroPosition[0] + change[0],zeroPosition[1]+change[1])
            if (newZeroPosition[0] < 1 or newZeroPosition[0] > 3 or newZeroPosition[1] < 1 or newZeroPosition[1] > 4): #invalid new zero position:
                continue
            else: #new zeroPosition is valid
                childState = currentNode.state[:] #make copy of parent state
                movingTile = childState.index(newZeroPosition) #this is the tile that will switch with the 0 tile
                childState[movingTile], childState[0] = childState[0],childState[movingTile] #swapping the 0 tile with the other tile
                if checkUnique(childState,Node.openList,Node.closedList): #if the childstate is valid AND unique
                    #this is where we prepare the new node
                    childG = currentNode.g[:]
                    childG.append(ACTIONS[change]) #appending either U,D,L,R, depending on the change to get the current state
                    childNode = Node(childState,currentNode,childG) #constructing the child node
                    childNode.h = childNode - goalNode #set child node h value
                    childNode.f = childNode.h + len(childNode.g) #set child node f value
                    #now append the new child node into the openList
                    Node.openList.append(childNode)
        Node.closedList.append(currentNode)

    #if openlist is empty and no solution has been found
    return None

#given the filename, read the file and return the rootState and goalState as a pair of 12 elements lists with each element containing a 2-tuple.
#String -> (list<tuple>,list<tuple>)
def readInput(filename):
    rootState = [None]*12
    goalState = [None]*12
    inF = open(filename,'r')
    
    #iterate through 3 lines
    for i in range(1,4):
        #split current line into a list of 4 integers
        currLine = inF.readline().split()
        #iterate through each integer
        for j in range (4):
            #the list at the index of the value of the tile (currLine[j]) stores the position of the tile (i,j+1) 
            rootState[int(currLine[j])] = (i,j+1)

    inF.readline()
    for i in range(1,4):
        currLine = inF.readline().split()
        for j in range(4):
            goalState[int(currLine[j])] = (i,j+1)

    inF.close()
    return (rootState,goalState)

    
    
    


    


def main():
    #get the name of the input file
    filename = input("Please type in the filename of the input (e.g. Sample_Input.txt): ")

    #get the root state and goal state in the 'list of 12 2-tuples' format from the input file
    try: 
        (rs,gs) = readInput(filename)
    except OSError: #if input filename is bad, quit
        print('The filename you entered was not valid. Please check whether or not the file is in the same directory as the program and try again.')
        print('Program ended')
        return
        
    #use A* to find the goal node
    goalNode = getSolutionNode(rs,gs)

    #if goal node cannot be found AKA the solution is impossible to achieve, end the program without creating a solution file
    if goalNode is None:
        print('Beep boop, it seems that it is impossible to get from the initial state to the goal state in this input file. No solution file was created, please try a different input file instead.')
        print('Program ended')
        return

    #if a goal node was found, copy the content from the input file onto a newly created text file called 'output_+INPUT_FILE_NAME'and add the string representation of the goal node (in the required format) underneath it.
    with open(filename) as inF:
        with open(input('Please type in how you would like to name the output file (e.g. Sample_Output.txt): '),'w') as outF:
            for line in inF:
                outF.write(line)
            outF.write('\n\n' + str(goalNode))
    print('The solution file has been created')
    print('Program ended')

    #this step will clear the frontier and the repeated list as to free up memory in case of future use of solving multiple input files with one program execution. 
    Node.openList.clear()
    Node.closedList.clear()


    
#debug test for getSolutionNode function
def testGetSolutionNode():
    #sample input / output
    sampleRootState = [(1,3),(3,4),(3,1),(3,2),(3,3),(1,1),(1,2),(1,4),(2,1),(2,2),(2,3),(2,4)]
    sampleGoalState = [(2,3),(3,4),(3,1),(2,2),(3,2),(1,1),(1,3),(1,4),(2,1),(1,2),(3,3),(2,4)]
    solutionNode = getSolutionNode(sampleRootState,sampleGoalState)
    print(solutionNode)
    

#debug test for getLowest function
def testGetLowest():
    testNodeList = []

    node1 = Node ([(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0)])
    node1.f = 5
    testNodeList.append(node1)

    node2 = Node ([(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0)])
    node2.f = 3
    testNodeList.append(node2)

    node3 = Node ([(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0)])
    node3.f = 6
    testNodeList.append(node3)

    node4 = Node ([(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0)])
    node4.f = 4
    testNodeList.append(node4)

    print(getLowestFNodeIndex(testNodeList))
    print(testNodeList)






#calls the main function
main()
        

        









    


