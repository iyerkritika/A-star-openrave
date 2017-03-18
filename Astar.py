import numpy as n
import math
from Queue import PriorityQueue
import time
import openravepy
# defining step sizes
handles=[]
stepx=0.25
stepy=0.18
steptheta=n.pi/2;
# heuristic function
def heuristic(a,b,t):# t is the type of heuristic (1 is manhattan and 2 is Euclidean)
    if t==1:
        return abs(a[0]-b[0])+abs(a[1]-b[1])+abs(a[2]-b[2])
    if t==2:
        return math.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2+(a[2]-b[2])**2)
# getting neighbors, co is the number of neighbors (1 is 4 and 2 is 8)
def neigh_conn(env,robot,x,co):
    a=[]
    if co==1:
        a=[[x[0]+stepx,x[1],x[2]], [x[0],x[1]+stepy,x[2]], [x[0]-stepx,x[1],x[2]], [x[0],x[1]-stepy,x[2]],[x[0],x[1],(x[2]+steptheta)%(2*n.pi)],[x[0],x[1],(x[2]-steptheta)%(2*n.pi)]]
    if co==2:
        for i in range(-1,2):
            for j in range (-1,2):
                for k in range (-1,2):
                    if i==0 and j==0 and k==0:
                        continue
                    a.append([x[0]+i*stepx,x[1]+j*stepy,x[2]+k*steptheta])
    b=[]
    for next in a:
        # checking for collision , if collides does not return as neighbor and prints in red
        robot.SetActiveDOFValues([next[0],next[1],next[2]])
        if env.CheckCollision(robot):
            handles.append(env.plot3(points=n.array(((next[0],next[1],0.1))),pointsize=0.05,colors=n.array(((1,0,0))),drawstyle=1))
            continue
        else:
            # otherwise return as neighbor and prints in blue
            handles.append(env.plot3(points=n.array(((next[0],next[1],0.1))),pointsize=0.05,colors=n.array(((0,0,1))),drawstyle=1))
            b.append(next)
    return b

class node:
    # have defined a class consisting of points , cost to reach node and the node's parent
    def __init__(self,p,c,mom):
        self.points=p
        self.cost=c
        self.parent=mom

# searching through already visited nodes
def srch(q,p):
    for next in range (0,len(q)):
        if q[next].points==p:
            return q[next]
    return None
# the main astar algorith
def Astar(env,robot,start,goal,t,co):
    new_cost=0
    current_cost=0
    visited=[]
    node_list=[]
    # defining start and end nodes
    st=node(start,0,None)
    gol=node(goal,0,None)
    # settinga priority queue for extracting nodes with least cost
    nodes=PriorityQueue()
    nodes.put((0,st))
    while not nodes.empty():
        # getting node and adding it to nodes list and giving points to visited list
        current_node=nodes.get()[1]
        visited.append(current_node.points)
        node_list.append(current_node)
        # if current_node.points==gol.points, check values are compared:
        checkx=current_node.points[0]-gol.points[0]
        checky=current_node.points[1]-gol.points[1]
        checktheta=abs(current_node.points[2])-abs(current_node.points[2])
        #goal reach condition
        if math.sqrt(round(checkx,1)**2 +round(checky,1)**2) <= math.sqrt(stepx**2+stepy**2):
            gol.parent=current_node.parent
            print 'yaay goal'
            return gol
        # if goal is not reached more neighbors are explored
        for next in neigh_conn(env,robot,current_node.points,co):
            new_cost=current_node.cost+0.05 # cost to get to next is 0.05 in all directions
            if next not in visited:
                # add new node
                new_node=node(next,new_cost,current_node)
                evalfn=new_cost+heuristic(new_node.points,gol.points,t)
                nodes.put((evalfn,new_node))
                continue
            # else search
            temp=srch(node_list,next)
            if new_cost<temp.cost:
                # better way to reach that node
                temp.cost=new_cost
                evalfn=new_cost+heuristic(temp.points,gol.points,t)
                nodes.put((evalfn,temp))
                # resetting parent
                temp.parent=current_node
# path extracting
def path (env,robot,nod,s):
    traj_path=[]
    # from goal node, to start position going to the parent of each node
    while nod.parent is not None:
        handles.append(env.plot3(points=n.array(((nod.points[0],nod.points[1],0.2))),pointsize=0.05,colors=n.array(((0,0,0))),drawstyle=1))
        traj_path.append([nod.points[0],nod.points[1],nod.points[2]])
        nod=nod.parent
    handles.append(env.plot3(points=n.array(((s[0],s[1],0.2))),pointsize=0.05,colors=n.array(((0,0,0))),drawstyle=1))
    traj_path.append([s[0],s[1],s[2]])
    # created list of points in trajectory and reversed to move from start to goal
    traj_path.reverse()
    return traj_path

# nothing i needed to be written in main function
if __name__ == "__main__":
    raw_input("Press enter to exit...")
