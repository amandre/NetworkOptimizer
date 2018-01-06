import networkx as nx
import xml.etree.ElementTree as ET
import math
import matplotlib.pyplot as plt
import numpy

'''
 - setupCost = 0 (for existing edge) or setupCost = calculate_cost(city1,city2) (for new edge)
 - capacity = 1 (by default each path could use many edges and may consist of many vertices)
 - algorithm design:
    > firstly reserves the capacity for the most distant vertices
    > firstly the capacity is reserved for the shortest path
    > if the shortest path doesn't have any modules left to install,
        the algorithm is trying to install modules on the next path
'''

cap = 5 #1 # module capacity [Gbps]
u_max = 10 #10 #max number of modules per link are allowed
budget = 100000


def calculate_cost (graph, city1, city2):
    # calculates cost of a link added between 2 locations
    # calculates distance in kilometers based on Cartesian coordinates
    # x describes East or West, y describes North or South
    x1 = graph.node[city1]['x']
    y1 = graph.node[city1]['y']
    x2 = graph.node[city2]['x']
    y2 = graph.node[city2]['y']

    R = 6371e3 # radius in meters
    fi1 = math.radians(y1)
    fi2 = math.radians(y2)
    dfi = math.radians(y2-y1)
    ddelta = math.radians(x2-x1)

    a = math.sin(dfi/2) * math.sin(dfi/2) + math.cos(fi1) * math.cos(fi2) * math.sin(ddelta/2) * math.sin(ddelta/2);
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    distance = int(R * c/1000) # distance in kilometers
    return distance^2 # cost of a newly installed link between 2 locations


def draw_graph(xml_file, cap):
    # builds a graph with existing vertices and edges
    topo = ET.parse(xml_file)
    root = topo.getroot()
    graph = nx.DiGraph() #for directed simple graph
    i = 0

    for v in root.iter('{http://sndlib.zib.de/network}node'):
        i += 1
        graph.add_node(v.attrib['id'], no=i, x=float(v[0][0].text), y=float(v[0][1].text))

    n = len(graph.nodes)
    demand = numpy.zeros((n,n))

    for vs in graph.nodes:
        vsrc_id = graph.node[vs]["no"]
        for vd in graph.nodes:
            vdst_id = graph.node[vd]["no"]
            if vsrc_id != vdst_id:
                demand[vsrc_id-1][vdst_id-1] = abs(vdst_id-vsrc_id)*10 # [Gbps]

    for e in root.iter('{http://sndlib.zib.de/network}link'):
        # creating a pair of edges (with attributes such as an id, sd, td, number of modules,setupCost and amount of free capacity)
        num0 = graph.node[e[0].text]['no']-1
        num1 = graph.node[e[1].text]['no']-1
        graph.add_edge(e[0].text,e[1].text,d=demand[num0][num1],mod_no=1,setupCost=0,free_cap=cap)
        graph.add_edge(e[1].text,e[0].text,d=demand[num1][num0],mod_no=1,setupCost=0,free_cap=cap)
    nx.draw(graph)
    return graph


def calculate_path(graph, budget, demands):
    # calculates optimal paths for all cities given in demands list
    pot_budget = budget
    for city1, city2, attr in demands:
        pot_paths = nx.all_simple_paths(graph, city1, city2)
        l_pot_paths = list(pot_paths)
        paths = sorted(l_pot_paths, key=lambda x: len(x))
        path_dict = []
        for p in paths:
            p_row = {'path': p, 'demand': attr['d']}
            path_dict.append(p_row)
        if len(paths) == 1: #if there is only one existing path
            modules_no = 1
            for i in range(0, len(path_dict['path'])-1):
                free_cap = graph[paths[i]][paths[i+1]]['free_cap']
                if attr['d'] > free_cap: #if the demand is greater than free capacity on any of the links in the path, add modules to handle the traffic

                    lack_of_cap = attr['d'] - free_cap
                    while lack_of_cap > 0:
                        if modules_no < u_max:
                            if graph[paths[i]][paths[i+1]]['free_cap'] > 0:
                                res = (free_cap if lack_of_cap >= free_cap else lack_of_cap)
                                update_module(graph, paths[i], paths[i+1], lack_of_cap, res)
                            else:
                                res = (cap if lack_of_cap >= cap else lack_of_cap)
                                modules_no += 1
                                add_module(graph, paths[i], paths[i+1], pot_budget, res)
                            lack_of_cap -= res
                        elif graph[paths[i]][paths[i+1]]['free_cap'] <= 0:
                            raise Exception("Problem unsolvable, the only exisitng path lacks of capacity")
                        else:
                            raise Exception("Problem unsolvable, all ", u_max, " modules are already used")
                    # print pot_budget
                    budget -= pot_budget
                else:
                    # add path between the links, and decrease the free capacity on this module
                    print "Link is able to handle the demand.\nCreating a direct link for (", city1, ",", city2, ")..."
                    graph.add_path([paths[i], paths[i+1]])
                    graph[paths[i]][paths[i+1]]['free_cap'] -= attr['d']
        elif len(paths) == 0:
            print "There isn't any existing path for this connection. It needs to be created."
        else:
            # case for multiple paths
            pot_budget_left = divide_traffic(graph, path_dict, pot_budget, demands)
            pot_budget = pot_budget_left
    return graph, pot_budget


def add_module(graph, src, dst, pot_budget, res):
    # installs modules for link between src and dst nodes
    pot_budget -= calculate_cost(graph, src, dst) # calculated amount of budget left when the direct link between two cities is needed
    graph[src][dst]['free_cap'] += cap - res
    graph[src][dst]['mod_no'] += 1
    if pot_budget < 0:
        raise Exception("Problem unsolvable, the budget limit has been exceeded")
    return pot_budget


def update_module(graph, src, dst, res):
    # updates existing module - updates left capacity
    graph[src][dst]['free_cap'] -= res


def divide_traffic(graph, paths, budget, demands):
    # divide the traffic between paths when it cannot be handled by only one path
    # paths - dictionary variable which consists of path, demand and left capacity
    pot_budget = budget
    for p in paths:
        i=0
       # ## TODO - fixing p_row - continuation
       # free_cap=[]
       # for c1, c2, attr in demands:
       #     free_cap.append(attr['free_cap'] if c1 == p['path'][i] and c2 == p['path'][i+1])
       #     i += 1
        modules_no = 1
        if p['demand'] > p['free_cap']:
            lack_of_cap = p['demand'] - p['free_cap']
            while lack_of_cap > 0:
                for i in range(0, len(p['path'])):
                    if lack_of_cap >= cap:
                        if modules_no < u_max and i+1 < len(p['path']):
                            res = (cap if lack_of_cap >= cap else lack_of_cap)
                            lack_of_cap -= res
                            modules_no += 1
                            add_module(graph, p['path'][i], p['path'][i+1], lack_of_cap, pot_budget, res)
                        else:
                            print 'This path lacks of modules. Moving to the next path'
                            break
    print 'budget after for loop', pot_budget
    return pot_budget

def sort_demands_desc(graph):
    # sorts the list of edges based on the demand value in descending order
    return sorted(graph.edges.data(), key=lambda x: x[-1], reverse=True)

if __name__ == "__main__":
    graph = draw_graph('polska.xml', cap)
    sorted_demands = sort_demands_desc(graph)
#    new_graph, new_budget = calculate_path(graph, budget, sorted_demands)
#    money_spent = budget - new_budget
#    print '#The spent budget =', money_spent

#    plt.show()